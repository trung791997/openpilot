import { html, reactive } from "/assets/vendor/arrow-core.js";

const STATUS_POLL_MS = 3000;
const MAX_ROUTES = 8;
const MAX_RENDERED_ROUTES = 250;

const state = reactive({
  loadingRoutes: false,
  runningAction: "",
  error: "",
  routes: [],
  selectedRoutes: [],
  routeProgress: 0,
  routeTotal: 0,
  workspace: { trials: [], activeStack: [], currentSchedule: "", currentFingerprint: "", currentBands: [], status: {} },
  status: { isOnroad: false, running: false, state: "" },
  expanded: {},
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

async function fetchRoutes() {
  state.loadingRoutes = true;
  state.routes = [];
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
        if (Array.isArray(data.routes) && data.routes.length) {
          state.routes = [...state.routes, ...data.routes].slice(0, MAX_RENDERED_ROUTES);
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
    if (prev !== state.status.state && ["complete", "failed", "cancelled_onroad", "cancelled"].includes(state.status.state)) {
      await fetchWorkspace();
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

function selectLatest(n) {
  state.selectedRoutes = state.routes.slice(0, Math.min(n, MAX_ROUTES)).map((r) => r.name);
}

async function runAction(label, fn, okMessage) {
  state.runningAction = label;
  state.error = "";
  try {
    const payload = await fn();
    notify(payload.message || okMessage);
    await fetchWorkspace();
    await fetchStatus();
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
  if (!window.confirm(`Apply trial ${trialId}? This writes the PID band P scales LatPScaleLowSpeed/Standard/Highway (I and F unchanged) and drops any P term from LatGainSchedule. You can revert it here.`)) return;
  try {
    await runAction("apply", () => requestJson(`/api/lat_tune/trial/${trialId}/apply`, { method: "POST", body: JSON.stringify({}) }));
  } catch (e) {
    if (e.status === 409 && /fingerprint/i.test(e.message) &&
        window.confirm("Your manual lateral tuning changed since these routes were driven. Apply anyway (force)?")) {
      await runAction("apply", () => requestJson(`/api/lat_tune/trial/${trialId}/apply`, { method: "POST", body: JSON.stringify({ force: true }) })).catch(() => {});
    }
  }
}

function revertTrial(trialId) {
  if (!window.confirm(`Revert trial ${trialId}? The previous P band scales and LatGainSchedule are restored.`)) return;
  return runAction("revert", () => requestJson(`/api/lat_tune/trial/${trialId}/revert`, { method: "POST" })).catch(() => {});
}

function deleteTrial(trialId) {
  if (!window.confirm(`Delete trial ${trialId}?`)) return;
  return runAction("delete", () => requestJson(`/api/lat_tune/trial/${trialId}`, { method: "DELETE" })).catch(() => {});
}

async function toggleExpanded(trialId) {
  if (state.expanded[trialId]) {
    state.expanded = { ...state.expanded, [trialId]: null };
    return;
  }
  try {
    const trial = await requestJson(`/api/lat_tune/trial/${trialId}`);
    state.expanded = { ...state.expanded, [trialId]: trial };
  } catch (e) {
    state.error = e.message;
  }
}

const fmt = (v, d = 2) => (typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "–");
const when = (ts) => (ts ? new Date(ts * 1000).toLocaleString() : "");

const bandRange = (b) => (b.highMph == null ? `${b.lowMph}+ mph` : `${b.lowMph}–${b.highMph} mph`);
const pif = (g) => (g ? `${g.p} / ${g.i} / ${g.f}` : "–");
const isBandTrial = (t) => t.schemaVersion === 2;

function renderBands(trial) {
  if (!Array.isArray(trial.bands)) return html`<div class="latTuneWarning">⚠ This trial predates the StarPilot speed bands; re-analyze the routes.</div>`;
  return html`
    <table class="latTuneKnots">
      <thead><tr><th>Band</th><th>Minutes</th><th>Ready</th><th>Sign/s</th><th>Curve ratio</th><th>Overrides/min</th><th>Factor</th><th>P / I / F now</th><th>P new</th><th>Why</th></tr></thead>
      <tbody>
        ${() => trial.bands.map((b) => html`
          <tr>
            <td>${b.name}<br/><span class="latTuneMuted">${bandRange(b)}</span></td><td>${fmt(b.minutes, 1)}</td>
            <td class="${b.ready ? "latTuneReady" : "latTuneNotReady"}">${b.ready ? "yes" : "need 3 min"}</td>
            <td>${fmt(b.signRate)}</td><td>${fmt(b.curveRatio, 3)}</td><td>${fmt(b.pressRate)}</td>
            <td>${fmt(b.factor)}</td><td>${pif(b.current)}</td>
            <td class="${b.proposed.p !== b.current.p ? "latTuneReady" : ""}">${b.proposed.p}</td><td class="reason">${b.reason}</td>
          </tr>`)}
      </tbody>
    </table>`;
}

function renderTrial(t) {
  const stack = state.workspace.activeStack || [];
  const isTop = stack.length > 0 && stack[stack.length - 1] === t.trialId;
  const busy = () => !!state.runningAction || state.status.isOnroad;
  const full = () => state.expanded[t.trialId];
  return html`
    <div class="latTuneTrial ${t.applied ? "applied" : ""}">
      <div class="latTuneCardHeader">
        <div>
          <strong>${t.trialId}</strong> <span class="latTuneMuted">${when(t.createdAt)}</span>
          ${t.applied ? html`<span class="latTuneReady"> · applied ${when(t.applied.at)}</span>` : ""}
          <div class="latTuneMuted">${t.routeNames.length} route(s): ${t.routeNames.join(", ")}</div>
          ${isBandTrial(t)
            ? html`<div>Ready bands: ${(t.readyBands || []).join(", ") || "none"} · P (low / standard / highway) ${(t.currentP || []).join(" / ")} → ${(t.proposedP || []).join(" / ")}</div>`
            : html`<div class="latTuneWarning">⚠ Pre-band trial (20/30/40/50 mph knots); re-analyze to get StarPilot band values.</div>`}
          ${() => (t.warnings || []).map((w) => html`<div class="latTuneWarning">⚠ ${w}</div>`)}
        </div>
        <div class="latTuneActions">
          <button class="latTuneButton" @click="${() => toggleExpanded(t.trialId)}">${() => (full() ? "Hide" : "Details")}</button>
          <button class="latTuneButton primary" disabled="${() => busy() || !!t.applied || !isBandTrial(t) || (t.readyBands || []).length === 0}" @click="${() => applyTrial(t.trialId)}">Apply</button>
          <button class="latTuneButton danger" disabled="${() => busy() || !t.applied || !isTop}" @click="${() => revertTrial(t.trialId)}">Revert</button>
          <button class="latTuneButton" disabled="${() => !!state.runningAction || !!t.applied}" @click="${() => deleteTrial(t.trialId)}">Delete</button>
        </div>
      </div>
      ${() => (full() ? html`
        ${renderBands(full())}
        <div class="latTuneMuted">Band values from the newest route's logs (fingerprint ${full().baseline.fingerprint || "–"}). Only P is proposed; I and F stay as they are.</div>
        ${full().applied && full().applied.writtenParams ? html`<div class="latTuneCode">written: ${Object.entries(full().applied.writtenParams).map(([k, v]) => `${k}=${v}`).join(", ")}<br/>prior: ${Object.entries(full().applied.priorParams || {}).map(([k, v]) => `${k}=${v || "(unset)"}`).join(", ")}<br/>LatGainSchedule prior: ${full().applied.priorSchedule || "(none)"}</div>` : ""}
      ` : "")}
    </div>`;
}

export function LatTune() {
  initialize();
  const s = () => state.status;
  const canAnalyze = () => !state.runningAction && state.selectedRoutes.length > 0 && !s().isOnroad && !s().running;
  return html`
    <div class="latTunePage">
      <div class="latTuneCard">
        <div class="latTuneCardHeader">
          <h2>Lateral Tune (Honda modified-EPS P trim)</h2>
          <div class="latTuneActions">
            <button class="latTuneButton primary" disabled="${() => !canAnalyze()}" @click="${runAnalyze}">Analyze ${() => state.selectedRoutes.length} route(s)</button>
            <button class="latTuneButton danger" disabled="${() => !s().running}" @click="${stopAnalyze}">Stop</button>
            <button class="latTuneButton" @click="${() => { fetchWorkspace(); fetchStatus(); }}">Refresh</button>
          </div>
        </div>
        <p class="latTuneMuted">Pick up to ${MAX_ROUTES} routes. The analysis runs on the device while parked and proposes one P step per StarPilot PID speed band (0–25, 25–50, 50+ mph; factor 0.85–1.15, written on the 5 % grid). Each run is a trial you can apply and revert. Unit-test/replay evidence only; nothing here is road-validated.</p>
        <div class="latTuneStatusGrid">
          <div><span>State</span>${() => s().state || "idle"}</div>
          <div><span>Onroad</span>${() => (s().isOnroad ? "yes (parked only)" : "no")}</div>
          <div><span>Progress</span>${() => (s().total ? `${s().progress}/${s().total}` : "–")}</div>
          <div><span>Segment</span>${() => s().currentSegment || "–"}</div>
          <div><span>Applied stack</span>${() => (state.workspace.activeStack || []).join(" → ") || "none"}</div>
        </div>
        ${() => (s().state === "failed" ? html`<div class="latTuneError">${s().error}</div>` : "")}
        ${() => (state.error ? html`<div class="latTuneError">${state.error}</div>` : "")}
        <div class="latTuneCode">current P / I / F: ${() => (state.workspace.currentBands || []).map((b) => `${b.name} ${pif(b)}`).join(" · ") || "–"}<br/>current LatGainSchedule: ${() => state.workspace.currentSchedule || "(none)"}</div>
      </div>

      <div class="latTuneCard">
        <div class="latTuneCardHeader">
          <h3>Local routes ${() => (state.loadingRoutes ? `(loading ${state.routeProgress}/${state.routeTotal})` : `(${state.routes.length})`)}</h3>
          <div class="latTuneActions">
            <button class="latTuneButton" @click="${() => selectLatest(MAX_ROUTES)}">Select latest ${MAX_ROUTES}</button>
            <button class="latTuneButton" @click="${() => { state.selectedRoutes = []; }}">Clear</button>
          </div>
        </div>
        <div class="latTuneRouteList">
          ${() => state.routes.map((r) => html`
            <label class="latTuneRouteRow ${state.selectedRoutes.includes(r.name) ? "selected" : ""}">
              <input type="checkbox" checked="${() => state.selectedRoutes.includes(r.name)}" @change="${() => toggleRoute(r.name)}" />
              <span>${r.timestamp || r.startedAt || ""}</span>
              <span class="latTuneMuted">${r.name} · ${r.segmentCount} seg</span>
            </label>`)}
        </div>
      </div>

      <div class="latTuneCard">
        <h3>Trials</h3>
        ${() => (state.workspace.trials.length ? state.workspace.trials.map(renderTrial) : html`<div class="latTuneMuted">No trials yet.</div>`)}
      </div>
    </div>`;
}
