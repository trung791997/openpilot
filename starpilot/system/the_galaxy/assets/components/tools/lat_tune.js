import { html, reactive } from "/assets/vendor/arrow-core.js";

// NRDR PID Tuning (item 117): per-speed-band P trim for the NRDR PID lateral controller on the
// modified-EPS Honda. Same trial workflow as FLM (routes -> offroad analysis -> apply / revert) but a
// separate tool: it only ever proposes LatPScaleLowSpeed/Standard/Highway. Styling reuses the FLM
// (flm*) and long-maneuver (longManeuver*) classes so both lateral tools look like one family.

const STATUS_POLL_MS = 3000;
const MAX_ROUTES = 8;
const MAX_RENDERED_ROUTES = 250;
const DONE_STATES = ["complete", "failed", "cancelled_onroad", "cancelled"];

const state = reactive({
  loadingRoutes: false,
  runningAction: "",
  error: "",
  routes: [],
  selectedRoutes: [],
  routeProgress: 0,
  routeTotal: 0,
  connectDongleId: "",
  workspace: { trials: [], activeStack: [], currentSchedule: "", currentFingerprint: "", currentBands: [], status: {} },
  status: { isOnroad: false, running: false, state: "" },
  detailId: "",
  detail: null,
});

let pollTimer = null;
let initialized = false;

function isActive() {
  return window.location.pathname.startsWith("/lat_tune");
}

function notify(message, level) {
  if (typeof window.showSnackbar === "function") window.showSnackbar(message, level);
}

async function requestJson(url, opts = {}) {
  const res = await fetch(url, { headers: { "Content-Type": "application/json" }, ...opts });
  let payload = {};
  try { payload = await res.json(); } catch (_) { payload = {}; }
  if (!res.ok) {
    const err = new Error(payload.error || `${res.status} ${res.statusText}`);
    err.status = res.status;
    err.payload = payload;
    throw err;
  }
  return payload;
}

// ---------------------------------------------------------------- formatting

const safeCount = (v) => (Number.isFinite(Number(v)) ? Number(v) : 0);
const fmt = (v, d = 2) => (typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "–");
const when = (ts) => (ts ? new Date(ts * 1000).toLocaleString() : "");
const bandRange = (b) => (b.highMph == null ? `${b.lowMph}+ mph` : `${b.lowMph}–${b.highMph} mph`);
const isBandTrial = (t) => t.schemaVersion === 2;
const shortBand = (name) => ({ LowSpeed: "Low", Standard: "Std", Highway: "Hwy" }[name] || name);
const BAND_NAMES = ["LowSpeed", "Standard", "Highway"];

function formatTimestamp(value) {
  if (!value) return "Unknown route";
  const parsed = new Date(value);
  return Number.isNaN(parsed.getTime()) ? String(value) : parsed.toLocaleString();
}

function formatRouteLength(route) {
  const segments = Math.max(0, Math.round(safeCount(route?.segmentCount)));
  if (!segments) return "Length unavailable";
  const minutes = Math.max(1, Math.round(safeCount(route?.approxDurationSeconds) / 60) || segments);
  const duration = minutes >= 60 ? `~${Math.floor(minutes / 60)}h ${minutes % 60}m` : `~${minutes} min`;
  return `${segments} segment${segments === 1 ? "" : "s"} (${duration})`;
}

function formatStatusAge(updatedAt) {
  const updated = Number(updatedAt);
  if (!Number.isFinite(updated) || updated <= 0) return "unknown";
  const age = Math.max(0, Math.round(Date.now() / 1000 - updated));
  if (age < 5) return "just now";
  if (age < 60) return `${age}s ago`;
  if (age < 3600) return `${Math.round(age / 60)}m ago`;
  return `${Math.round(age / 3600)}h ago`;
}

function connectRouteUrl(routeName) {
  const dongleId = String(state.connectDongleId || "").trim();
  return dongleId && routeName ? `https://connect.comma.ai/${encodeURIComponent(dongleId)}/${encodeURIComponent(routeName)}` : "";
}

function sortedRoutes() {
  return [...state.routes].sort((a, b) => {
    const at = Date.parse(a.timestamp);
    const bt = Date.parse(b.timestamp);
    if (Number.isFinite(at) && Number.isFinite(bt)) return bt - at;
    return String(b.timestamp || "").localeCompare(String(a.timestamp || ""));
  });
}

function stateLabel(st) {
  return ({ queued: "Queued", starting: "Starting", analyzing: "Analyzing", complete: "Complete", failed: "Failed",
            cancelled: "Cancelled", cancelled_onroad: "Cancelled (went onroad)" }[st] || "Idle");
}

// ---------------------------------------------------------------- data

async function fetchRoutes() {
  state.loadingRoutes = true;
  state.routes = [];
  state.routeProgress = 0;
  state.routeTotal = 0;
  const seen = new Set();
  try {
    const tz = Intl.DateTimeFormat().resolvedOptions().timeZone || "UTC";
    const res = await fetch(`/api/routes?timezone=${encodeURIComponent(tz)}`);
    const reader = res.body.getReader();
    const decoder = new TextDecoder();
    let buffer = "";
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const events = buffer.split("\n\n");
      buffer = events.pop();
      for (const ev of events) {
        const line = ev.split("\n").find((l) => l.startsWith("data:"));
        if (!line) continue;
        let data = {};
        try { data = JSON.parse(line.slice(5)); } catch (_) { continue; }
        if (typeof data.connectDongleId === "string") state.connectDongleId = data.connectDongleId;
        if (Array.isArray(data.routes) && data.routes.length) {
          const fresh = data.routes.filter((r) => r && r.name && !seen.has(r.name));
          fresh.forEach((r) => seen.add(r.name));
          state.routes = [...state.routes, ...fresh].slice(0, MAX_RENDERED_ROUTES);
        }
        state.routeProgress = data.progress || 0;
        state.routeTotal = data.total || 0;
      }
    }
  } catch (e) {
    state.error = `Could not list routes: ${e.message}`;
  } finally {
    state.loadingRoutes = false;
  }
}

async function fetchWorkspace() {
  try {
    state.workspace = await requestJson("/api/lat_tune/workspace");
  } catch (e) {
    state.error = e.message;
  }
}

async function fetchStatus() {
  try {
    const payload = await requestJson("/api/lat_tune/status");
    const prev = state.status.state;
    state.status = { ...payload.status, isOnroad: !!payload.isOnroad, running: !!payload.status.running };
    if (prev !== state.status.state && DONE_STATES.includes(state.status.state)) {
      await fetchWorkspace();
      if (state.status.state === "complete" && state.status.trialId) openDetail(state.status.trialId);
    }
  } catch (e) {
    state.error = e.message;
  }
}

function ensurePolling() {
  if (pollTimer) return;
  const tick = async () => {
    pollTimer = null;
    if (!isActive()) return;
    if (document.visibilityState === "visible") await fetchStatus();
    pollTimer = setTimeout(tick, STATUS_POLL_MS);
  };
  pollTimer = setTimeout(tick, STATUS_POLL_MS);
}

async function initialize() {
  if (initialized) { ensurePolling(); return; }
  initialized = true;
  await fetchWorkspace();
  await fetchStatus();
  ensurePolling();
  fetchRoutes();
}

function refreshAll() {
  state.error = "";
  fetchRoutes();
  fetchWorkspace();
  fetchStatus();
  if (state.detailId) openDetail(state.detailId);
}

// ---------------------------------------------------------------- actions

function toggleRoute(name) {
  if (state.selectedRoutes.includes(name)) {
    state.selectedRoutes = state.selectedRoutes.filter((r) => r !== name);
    return;
  }
  if (state.selectedRoutes.length >= MAX_ROUTES) {
    notify(`Pick at most ${MAX_ROUTES} routes.`, "error");
    return;
  }
  state.selectedRoutes = [...state.selectedRoutes, name];
}

function selectLatest() {
  state.selectedRoutes = sortedRoutes().slice(0, MAX_ROUTES).map((r) => r.name);
}

async function runAction(label, fn, okMessage) {
  state.runningAction = label;
  state.error = "";
  try {
    const payload = await fn();
    notify(payload.message || okMessage);
    await fetchWorkspace();
    await fetchStatus();
    if (state.detailId) await openDetail(state.detailId);
    return payload;
  } catch (e) {
    state.error = e.message;
    notify(e.message, "error");
    throw e;
  } finally {
    state.runningAction = "";
  }
}

function runAnalyze() {
  return runAction("analyze", () => requestJson("/api/lat_tune/analyze", { method: "POST", body: JSON.stringify({ routes: state.selectedRoutes }) })).catch(() => {});
}

function stopAnalyze() {
  return runAction("stop", () => requestJson("/api/lat_tune/analyze/stop", { method: "POST" })).catch(() => {});
}

async function applyTrial(trialId) {
  if (!window.confirm(`Apply trial ${trialId}?\n\nEach band that moves gets its P step applied to the device's current P (LatPScaleLowSpeed / Standard / Highway), and a band the sim moved gets its I step applied to the device's current I (LatIScale*); held bands are not written. A P term in a valid LatGainSchedule is dropped. F is not changed. You can revert it here.`)) return;
  try {
    await runAction("apply", () => requestJson(`/api/lat_tune/trial/${encodeURIComponent(trialId)}/apply`, { method: "POST", body: JSON.stringify({}) }));
  } catch (e) {
    if (e.status === 409 && /fingerprint/i.test(e.message) &&
        window.confirm("Your manual lateral tuning changed since these routes were driven, so the proposal was measured on a different tune.\n\nApply anyway (force)?")) {
      await runAction("apply", () => requestJson(`/api/lat_tune/trial/${encodeURIComponent(trialId)}/apply`, { method: "POST", body: JSON.stringify({ force: true }) })).catch(() => {});
    }
  }
}

function revertTrial(trialId) {
  if (!window.confirm(`Revert trial ${trialId}? The previous P band scales and LatGainSchedule are restored.`)) return;
  return runAction("revert", () => requestJson(`/api/lat_tune/trial/${encodeURIComponent(trialId)}/revert`, { method: "POST" })).catch(() => {});
}

function deleteTrial(trialId) {
  if (!window.confirm(`Delete trial ${trialId}?`)) return;
  if (state.detailId === trialId) closeDetail();
  return runAction("delete", () => requestJson(`/api/lat_tune/trial/${encodeURIComponent(trialId)}`, { method: "DELETE" })).catch(() => {});
}

async function openDetail(trialId) {
  state.detailId = trialId;
  try {
    const trial = await requestJson(`/api/lat_tune/trial/${encodeURIComponent(trialId)}`);
    if (state.detailId === trialId) state.detail = trial;
  } catch (e) {
    state.error = e.message;
  }
}

function closeDetail() {
  state.detailId = "";
  state.detail = null;
}

function toggleDetail(trialId) {
  if (state.detailId === trialId) closeDetail();
  else openDetail(trialId);
}

// ---------------------------------------------------------------- rendering

function renderCurrentGains() {
  const bands = state.workspace.currentBands || [];
  if (!bands.length) return html`<p class="longManeuverMuted">Current band gains unavailable.</p>`;
  return html`
    <div class="latTuneBandGrid">
      ${bands.map((b) => html`
        <div class="latTuneBandTile">
          <div class="latTuneBandTileHead"><strong>${b.name}</strong><span>${bandRange(b)}</span></div>
          <div class="latTuneGains">
            <div><span>P</span><b>${b.p}</b></div>
            <div><span>I</span><b>${b.i}</b></div>
            <div><span>F</span><b>${b.f}</b></div>
          </div>
        </div>`)}
    </div>`;
}

function trialChanges(t) {
  const moved = (now, next, i) => next !== undefined && next[i] !== undefined && next[i] !== (now || [])[i];
  return BAND_NAMES.map((_, i) => ({ p: moved(t.currentP, t.proposedP, i), i: moved(t.currentI, t.proposedI, i) }));
}

function proposalSummary(t) {
  const nowP = t.currentP || [];
  const nextP = t.proposedP || [];
  const nowI = t.currentI || [];
  const nextI = t.proposedI || [];
  const changes = trialChanges(t);
  return BAND_NAMES.map((name, i) => {
    const c = changes[i];
    const sim = (t.sources || [])[i] === "sim";
    return html`<span class="latTunePill ${c.p || c.i ? "changed" : ""}" title="${sim ? "closed-loop sim" : "log rules"}">${shortBand(name)}${sim ? "·sim" : ""}
      P ${nowP[i] ?? "–"}${c.p ? html` → <b>${nextP[i]}</b>` : ""}${c.i ? html` · I ${nowI[i]} → <b>${nextI[i]}</b>` : ""}</span>`;
  });
}

function renderTrialRow(t) {
  const stack = state.workspace.activeStack || [];
  const isTop = stack.length > 0 && stack[stack.length - 1] === t.trialId;
  const busy = () => !!state.runningAction || !!state.status.isOnroad;
  const ready = (t.readyBands || []).length > 0;
  const anyChange = trialChanges(t).some((c) => c.p || c.i);
  return html`
    <div class="flmWorkspaceRow">
      <div class="${() => `flmWorkspaceItem latTuneTrialItem ${t.applied ? "applied" : ""} ${state.detailId === t.trialId ? "open" : ""}`}">
        <strong>${when(t.createdAt) || t.trialId}
          ${t.applied ? html`<em class="latTuneBadge applied">Applied</em>` : ""}
          ${isBandTrial(t) && !anyChange ? html`<em class="latTuneBadge">No change</em>` : ""}
        </strong>
        <small>${t.trialId} · ${t.routeNames.length} route${t.routeNames.length === 1 ? "" : "s"}</small>
        ${isBandTrial(t)
          ? html`<div class="latTunePills">${proposalSummary(t)}</div>
                 <small>${ready ? `Ready bands: ${t.readyBands.join(", ")}` : "No band has 3 min of data yet"}</small>`
          : html`<small class="latTuneWarning">Pre-band trial (20/30/40/50 mph knots). Re-analyze for NRDR PID band values.</small>`}
        ${(t.warnings || []).length ? html`<small class="latTuneWarning">${t.warnings.length} warning${t.warnings.length === 1 ? "" : "s"} — see details</small>` : ""}
      </div>
      <div class="flmSavedTuneActions">
        <button class="longManeuverButton" @click="${() => toggleDetail(t.trialId)}">${() => (state.detailId === t.trialId ? "Hide" : "Details")}</button>
        ${t.applied
          ? html`<button class="longManeuverButton danger" disabled="${() => busy() || !isTop}"
                   title="${isTop ? "" : "Only the most recently applied trial can be reverted"}"
                   @click="${() => revertTrial(t.trialId)}">Revert</button>`
          : html`<button class="longManeuverButton" disabled="${() => busy() || !isBandTrial(t) || !ready || !anyChange}"
                   @click="${() => applyTrial(t.trialId)}">Apply</button>`}
        <button class="longManeuverButton danger" disabled="${() => !!state.runningAction || !!t.applied}" @click="${() => deleteTrial(t.trialId)}">Delete</button>
      </div>
    </div>`;
}

function renderSim(b) {
  const s = b.sim;
  if (!s) return "";
  const row = (label, m) => (m ? html`<span>${label}: err ${fmt(m.err_rms)} · straight ${fmt(m.straight_rms)} · curve ${fmt(m.curve_ratio, 3)} · sign ${fmt(m.sign_hyst)}/s</span>` : "");
  return html`
    <div class="flmTrackingMeta latTuneSim">
      <span><em class="latTuneBadge ${s.trusted ? "ready" : "notReady"}">${s.trusted ? "Sim trusted" : "Sim untrusted"}</em> ${s.trust}</span>
      ${row("log", s.log)}
      ${row("sim, driven", s.simDriven)}
      ${row("sim, proposed", s.simProposed)}
    </div>`;
}

function renderBandCard(b) {
  const changed = b.proposed.p !== b.current.p;
  const direction = b.proposed.p > b.current.p ? "up" : "down";
  const iChanged = b.proposed.i !== b.current.i;
  const iDirection = b.proposed.i > b.current.i ? "up" : "down";
  return html`
    <article class="flmTrackingCard latTuneBandCard ${changed || iChanged ? "changed" : ""}">
      <div class="flmTrackingCardHeader">
        <div><strong>${b.name}</strong><span>${bandRange(b)}</span></div>
        <em class="latTuneBadge ${b.ready ? "ready" : "notReady"}">${b.ready ? "Ready" : "Needs 3 min"}</em>
      </div>
      <div class="latTunePChange">
        <span>P</span><b>${b.current.p}</b>
        <span class="latTuneArrow">→</span>
        <b class="${changed ? `latTuneChanged ${direction}` : ""}">${b.proposed.p}</b>
        <small>×${fmt(b.factor)}</small>
      </div>
      <div class="latTunePChange">
        <span>I</span><b>${b.current.i}</b>
        <span class="latTuneArrow">→</span>
        <b class="${iChanged ? `latTuneChanged ${iDirection}` : ""}">${b.proposed.i}</b>
        <small>${b.source === "sim" ? "sim" : "rules"}</small>
      </div>
      <div class="flmTrackingMeta">
        <span>${fmt(b.minutes, 1)} min</span>
        <span>sign ${fmt(b.signRate)}/s</span>
        <span>curve ${fmt(b.curveRatio, 3)}</span>
        <span>entry ${fmt(b.curveRatioEntry, 2)} · steady ${fmt(b.curveRatioSteady, 2)} · exit ${fmt(b.curveRatioExit, 2)}</span>
        <span>overrides ${fmt(b.pressRate)}/min</span>
        <span>F ${b.current.f}</span>
      </div>
      ${renderSim(b)}
      <p class="latTuneReason">${b.reason || ""}</p>
    </article>`;
}

function renderApplied(applied) {
  if (!applied || !applied.writtenParams) return "";
  const keys = Object.keys(applied.writtenParams);
  return html`
    <div class="flmCardSubsection flmTuneComparison">
      <h4>Applied ${when(applied.at)}${applied.forced ? " (forced past a tuning change)" : ""}</h4>
      <div class="flmTuneComparisonTable">
        <div class="flmTuneComparisonHeader">Param</div>
        <div class="flmTuneComparisonHeader">Before</div>
        <div class="flmTuneComparisonArrow"></div>
        <div class="flmTuneComparisonHeader">Written</div>
        ${keys.map((k) => {
          const prior = (applied.priorParams || {})[k];
          const written = applied.writtenParams[k];
          return html`
            <div class="flmTuneComparisonLabel">${k}</div>
            <div>${prior === "" || prior == null ? "(unset)" : prior}</div>
            <div class="flmTuneComparisonArrow">&gt;</div>
            <div class="${String(prior) !== String(written) ? "flmTuneComparisonChanged" : ""}">${written}</div>`;
        })}
        <div class="flmTuneComparisonLabel">LatGainSchedule</div>
        <div class="latTuneCode">${applied.priorSchedule || "(none)"}</div>
        <div class="flmTuneComparisonArrow">&gt;</div>
        <div class="latTuneCode">${applied.writtenSchedule || "(none)"}</div>
      </div>
    </div>`;
}

function renderDetail() {
  const t = state.detail;
  if (!state.detailId) return "";
  if (!t || t.trialId !== state.detailId) {
    return html`<section class="flmCard"><p class="longManeuverMuted">Loading trial ${state.detailId}…</p></section>`;
  }
  return html`
    <section class="flmCard">
      <div class="flmCardHeader">
        <div>
          <h3>Trial ${t.trialId}</h3>
          <p class="longManeuverMuted">${when(t.createdAt)} · ${(t.routeNames || []).length} route(s), ${safeCount(t.segmentCount)} segment(s) · baseline fingerprint ${t.baseline?.fingerprint || "–"}
            ${t.baseline?.fingerprint && t.baseline.fingerprint !== state.workspace.currentFingerprint
              ? html`<span class="latTuneWarning"> (differs from the device's current tuning)</span>` : ""}</p>
        </div>
        <button class="longManeuverButton" @click="${closeDetail}">Close</button>
      </div>
      ${Array.isArray(t.bands)
        ? html`<div class="flmTrackingGrid">${t.bands.map(renderBandCard)}</div>`
        : html`<p class="latTuneWarning">This trial predates the NRDR PID speed bands; re-analyze the routes.</p>`}
      <div class="flmTrackingNotice">Now is the value logged on the newest route. The log rules step P only, bounded to a factor of 0.85–1.15.
        Where the closed-loop sim (plant fitted to this car's EPS image) reproduces a band's logged tracking, it replaces that band's step with the best P/I pair within ±10 % P and ±25 I of the driven values.
        On apply, each moving band's step is applied to the device's current P and I, so a forced apply keeps any manual change made since. F is never written. Sim evidence only: drive it and compare.
        ${t.sim ? html`<br>Sim step: ${t.sim.status || "–"}${t.sim.minutes ? ` · ${t.sim.minutes} min simulated` : ""}${t.sim.image ? ` · plant image ${t.sim.image}` : ""}` : ""}</div>
      ${renderApplied(t.applied)}
      ${(t.warnings || []).length ? html`
        <div class="flmCardSubsection">
          <h4>Warnings</h4>
          <ul class="latTuneWarnings">${t.warnings.map((w) => html`<li>${w}</li>`)}</ul>
        </div>` : ""}
      <p class="longManeuverMuted latTuneRoutes">Routes: ${(t.routeNames || []).join(", ")}</p>
    </section>`;
}

export function LatTune() {
  initialize();
  const s = () => state.status;
  const stack = () => state.workspace.activeStack || [];
  const canAnalyze = () => !state.runningAction && state.selectedRoutes.length > 0 && !s().isOnroad && !s().running;
  return html`
    <div class="longManeuverPage latTunePage">
      <h2>NRDR PID Tuning</h2>

      <div class="longManeuverCard">
        <p class="longManeuverIntro">
          Trims the NRDR PID lateral controller's P gain per speed band (Low 0–25, Standard 25–50, Highway 50+ mph) from your own drives on the modified-EPS Honda.
          Pick routes, analyze while parked, then apply one bounded P step, drive, and keep or revert it.
          This is separate from FLM (Lateral Tuning) and never touches I, F or the torque tune.
        </p>
        <p class="latTuneEvidence">Unit-test and log-replay evidence only; nothing here is road-validated. Change one step at a time.</p>

        <div class="longManeuverActions">
          <button class="longManeuverButton" disabled="${() => !canAnalyze()}" @click="${runAnalyze}">
            Analyze Selected Routes${() => (state.selectedRoutes.length ? ` (${state.selectedRoutes.length})` : "")}
          </button>
          <button class="longManeuverButton danger" disabled="${() => !!state.runningAction || !s().running}" @click="${stopAnalyze}">Stop Analysis</button>
          <button class="longManeuverButton" disabled="${() => !!state.runningAction}" @click="${refreshAll}">Refresh</button>
        </div>

        ${() => (state.error ? html`<p class="longManeuverError">${state.error}</p>` : "")}
        ${() => (s().state === "failed" && s().error ? html`<p class="longManeuverError">Analysis failed: ${s().error}</p>` : "")}
        ${() => (s().isOnroad ? html`<p class="longManeuverError">Analysis, apply and revert are offroad only. Park and go offroad first.</p>` : "")}

        <div class="longManeuverStatusGrid">
          <p><strong>Status:</strong> ${() => stateLabel(s().state)}</p>
          <p><strong>Onroad:</strong> ${() => (s().isOnroad ? "Yes" : "No")}</p>
          <p><strong>Updated:</strong> ${() => formatStatusAge(s().updatedAt)}</p>
          <p><strong>Selected Routes:</strong> ${() => `${state.selectedRoutes.length}/${MAX_ROUTES}`}</p>
          <p><strong>Progress:</strong> ${() => (s().total ? `${safeCount(s().progress)}/${safeCount(s().total)} segments` : "–")}</p>
          <p><strong>Applied Trial:</strong> ${() => stack()[stack().length - 1] || "None"}${() => (stack().length > 1 ? ` (+${stack().length - 1} below)` : "")}</p>
        </div>

        ${() => (s().running ? html`
          <div class="latTuneProgress"><div style="${`width: ${s().total ? Math.round(100 * safeCount(s().progress) / s().total) : 0}%`}"></div></div>` : "")}
        ${() => (s().currentSegment && s().running ? html`
          <div class="longManeuverCurrent"><p><strong>Current Segment:</strong> ${s().currentSegment}</p></div>` : "")}

        <div class="flmCardSubsection">
          <h3>Current NRDR PID gains</h3>
          ${renderCurrentGains}
          ${() => (state.workspace.currentSchedule ? html`
            <p class="latTuneWarning">LatGainSchedule is set and overrides the bands for the terms it names: <code>${state.workspace.currentSchedule}</code>. Apply removes its P term.</p>` : "")}
        </div>

        <div class="flmTwoColumn">
          <section class="flmCard">
            <div class="flmCardHeader">
              <div>
                <h3>Local Routes</h3>
                <p class="longManeuverMuted">Pick up to ${MAX_ROUTES} routes driven on the same tuning. rlogs are preferred; qlogs are used as a fallback.</p>
              </div>
              <div class="latTuneHeaderActions">
                <button class="longManeuverButton" disabled="${() => !state.routes.length}" @click="${selectLatest}">Latest ${MAX_ROUTES}</button>
                <button class="longManeuverButton" disabled="${() => !state.selectedRoutes.length}" @click="${() => { state.selectedRoutes = []; }}">Clear</button>
              </div>
            </div>
            ${() => (state.loadingRoutes ? html`<p class="longManeuverMuted">Loading local routes… ${state.routeTotal ? `${state.routeProgress}/${state.routeTotal}` : ""}</p>` : "")}
            ${() => (!state.loadingRoutes && !state.routes.length ? html`<p class="longManeuverMuted">No local routes found.</p>` : "")}
            <div class="flmRouteList">
              ${() => sortedRoutes().map((r) => html`
                <div class="flmRouteRow">
                  <label class="${() => `flmRouteItem ${state.selectedRoutes.includes(r.name) ? "latTuneSelected" : ""}`}">
                    <input type="checkbox" checked="${() => state.selectedRoutes.includes(r.name)}" @change="${() => toggleRoute(r.name)}" />
                    <span>
                      <strong>${formatTimestamp(r.timestamp)}</strong>
                      <small>${r.name}</small>
                      <small>${formatRouteLength(r)}</small>
                    </span>
                  </label>
                  ${() => (connectRouteUrl(r.name) ? html`
                    <a class="flmConnectLink" href="${connectRouteUrl(r.name)}" target="_blank" rel="noopener noreferrer">Connect</a>` : "")}
                </div>`)}
            </div>
          </section>

          <section class="flmCard">
            <div class="flmCardHeader">
              <div>
                <h3>Trials</h3>
                <p class="longManeuverMuted">Each analysis is a trial. Applied trials stack; only the top one can be reverted.</p>
              </div>
            </div>
            <div class="flmWorkspaceList">
              ${() => ((state.workspace.trials || []).length
                ? state.workspace.trials.map(renderTrialRow)
                : html`<p class="longManeuverMuted">No trials yet. Select routes and analyze.</p>`)}
            </div>
          </section>
        </div>

        ${renderDetail}
      </div>
    </div>`;
}
