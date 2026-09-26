import { html, reactive } from "/assets/vendor/arrow-core.js"
import {
  HELP_TEXT, LIVE_POLL_MS, LiveBuffer, ZOOM_HALF_WINDOW_S,
  buildOverviewCharts, buildTrackingCharts, fmtDate, fmtDuration, fmtNum, keyNumbers,
  longStateName, sessionUrl, statusLabel, timeAtClick, toSeries,
} from "/assets/components/tools/drive_plots_shared.mjs"

const ADVANCED_TERMS_KEY = "plotsShowAdvancedTerms"
const SESSIONS_POLL_MS = 5000

const state = reactive({
  loading: true,
  error: "",
  paused: false,
  showAdvancedTerms: false,
  live: null,
  liveCharts: [],
  latest: null,
  busy: false,
  sessions: [],
  selectedId: "",
  detail: null,
  detailError: "",
  zoom: null,
})

const buffer = new LiveBuffer(60)
let initialized = false
let pollHandle = null
let lastSessionsFetch = 0
let visibilityListenerAttached = false

function isPlotsRouteActive() {
  return window.location.pathname === "/plots"
}

async function fetchJson(url, init) {
  const response = await fetch(url, init)
  const payload = await response.json().catch(() => ({}))
  if (!response.ok) throw new Error(payload.error || response.statusText || "Request failed")
  return payload
}

function rebuildLiveCharts() {
  state.liveCharts = buildTrackingCharts(buffer.view(), { advanced: state.showAdvancedTerms })
  state.latest = buffer.latest()
}

async function fetchLiveData() {
  const payload = await fetchJson(`/api/plots/live?since=${buffer.seq}`)
  const hadRecording = !!state.live?.recording
  state.live = payload
  state.error = ""
  state.loading = false
  if (buffer.ingest(payload)) rebuildLiveCharts()
  // A recording that just ended (stopped here, elsewhere, or auto-stopped offroad) shows up in the list.
  if (hadRecording !== !!payload.recording || Date.now() - lastSessionsFetch > SESSIONS_POLL_MS) {
    fetchSessions().catch(() => {})
  }
}

async function fetchSessions() {
  lastSessionsFetch = Date.now()
  const payload = await fetchJson("/api/plots/sessions")
  state.sessions = Array.isArray(payload.sessions) ? payload.sessions : []
  const selected = state.sessions.find((s) => s.id === state.selectedId)
  if (selected && state.detail && state.detail.meta?.status !== selected.status) {
    openSession(selected.id)
  }
}

function stopPolling() {
  if (!pollHandle) return
  clearTimeout(pollHandle)
  pollHandle = null
}

function ensurePolling() {
  if (pollHandle) return
  const poll = async () => {
    if (!isPlotsRouteActive()) {
      pollHandle = null
      return
    }
    if (document.visibilityState === "visible" && !state.paused) {
      try {
        await fetchLiveData()
      } catch (error) {
        state.error = error?.message || String(error)
        state.loading = false
      }
    }
    pollHandle = setTimeout(poll, LIVE_POLL_MS)
  }
  pollHandle = setTimeout(poll, LIVE_POLL_MS)
}

async function startRecording() {
  state.busy = true
  try {
    await fetchJson("/api/plots/recording/start", { method: "POST" })
    await fetchLiveData()
  } catch (error) {
    state.error = error?.message || String(error)
  } finally {
    state.busy = false
  }
}

async function stopRecording() {
  state.busy = true
  try {
    const payload = await fetchJson("/api/plots/recording/stop", { method: "POST" })
    await fetchLiveData()
    await fetchSessions()
    if (payload.stopped?.id) openSession(payload.stopped.id)
  } catch (error) {
    state.error = error?.message || String(error)
  } finally {
    state.busy = false
  }
}

async function openSession(id) {
  state.selectedId = id
  state.detailError = ""
  state.zoom = null
  try {
    state.detail = await fetchJson(sessionUrl(id))
  } catch (error) {
    state.detail = null
    state.detailError = error?.message || String(error)
  }
}

function closeSession() {
  state.selectedId = ""
  state.detail = null
  state.zoom = null
}

async function deleteSession(id) {
  if (!window.confirm("Delete this saved drive? This cannot be undone.")) return
  try {
    await fetchJson(sessionUrl(id), { method: "DELETE" })
    if (state.selectedId === id) closeSession()
    await fetchSessions()
  } catch (error) {
    state.detailError = error?.message || String(error)
  }
}

async function zoomAt(event, chart) {
  const t = timeAtClick(event, chart.geo)
  if (t === null || !state.selectedId) return
  const start = Math.max(0, t - ZOOM_HALF_WINDOW_S)
  const end = start + 2 * ZOOM_HALF_WINDOW_S
  try {
    const payload = await fetchJson(`${sessionUrl(state.selectedId, "/window")}?start=${start.toFixed(1)}&end=${end.toFixed(1)}`)
    const series = toSeries(payload.columns, payload.rows)
    state.zoom = { start, end, charts: buildTrackingCharts(series, { advanced: state.showAdvancedTerms, tMin: start, tMax: end }) }
  } catch (error) {
    state.detailError = error?.message || String(error)
  }
}

function togglePaused() {
  state.paused = !state.paused
  if (state.paused) {
    stopPolling()
    return
  }
  if (!isPlotsRouteActive()) return
  fetchLiveData().catch((error) => {
    state.error = error?.message || String(error)
  })
  ensurePolling()
}

function toggleAdvancedTerms() {
  state.showAdvancedTerms = !state.showAdvancedTerms
  try {
    localStorage.setItem(ADVANCED_TERMS_KEY, state.showAdvancedTerms ? "1" : "0")
  } catch (error) {
    console.warn("Failed to persist plots advanced terms preference", error)
  }
  rebuildLiveCharts()
}

function ChartSvg(chart, onClick) {
  const g = chart.geo
  if (g.empty) return html`<div class="plotEmpty">Waiting for data...</div>`
  const [l0, l1, l2, l3] = g.slots
  return html`
    <svg class="plotSvg ${onClick ? "plotSvgClickable" : ""}" viewBox="0 0 ${g.width} ${g.height}" preserveAspectRatio="none"
      role="img" aria-label="${chart.title}" @click="${(e) => onClick && onClick(e, chart)}">
      <path class="plotShade" d="${g.shadeD}"></path>
      <path class="plotGridLine" d="${g.gridD}"></path>
      <path class="plotZeroLine" d="${g.zeroD}"></path>
      <path class="plotLine ${l0.cls}" d="${l0.d}"></path>
      <path class="plotLine ${l1.cls}" d="${l1.d}"></path>
      <path class="plotLine ${l2.cls}" d="${l2.d}"></path>
      <path class="plotLine ${l3.cls}" d="${l3.d}"></path>
    </svg>
  `
}

function ChartCard(chart, onClick = null) {
  const g = chart.geo
  return html`
    <section class="plotCard plotChartCard">
      <div class="plotCardHeader">
        <h2>${chart.title}</h2>
        ${chart.unit ? html`<span class="plotSource">${chart.unit}</span>` : ""}
      </div>
      <div class="plotLegend">
        ${chart.legend.map((l) => html`
          <span class="plotLegendItem"><i class="plotLegendLine ${l.cls}"></i>${l.label}${l.value ? `: ${l.value}` : ""}</span>
        `)}
      </div>
      <div class="plotSvgWrap">${ChartSvg(chart, onClick)}</div>
      ${g.empty ? "" : html`
        <div class="plotRangeRow">
          ${g.xTicks.map((x) => html`<span>${x.label}</span>`)}
        </div>
        <div class="plotRangeRow plotYRange"><span>y: ${fmtNum(g.min, 1)} … ${fmtNum(g.max, 1)} ${chart.unit}</span></div>
      `}
    </section>
  `
}

function AnalysisBlock(title, axis, m, liveWindow = false) {
  if (!m) return ""
  return html`
    <div class="qualitySummaryRow">
      <p class="qualitySentence">${title}</p>
      <p class="plotSummary">${m.summary}</p>
      ${m.notes?.length ? html`<ul class="plotNotes">${m.notes.map((n) => html`<li>${n}</li>`)}</ul>` : ""}
      ${liveWindow ? "" : html`
        <div class="plotKeyGrid">
          ${keyNumbers(axis, m).map((k) => html`<span class="plotKeyLabel">${k.label}</span><span class="plotKeyValue">${k.value}</span>`)}
        </div>
      `}
    </div>
  `
}

function RecordingCard() {
  const live = state.live || {}
  const rec = live.recording
  return html`
    <section class="plotCard plotStatusCard">
      <p class="plotDescription">
        Press <strong>Start recording</strong> before a drive. Everything openpilot asks for and what the car does is saved
        at 20 Hz until you press Stop (or 30 s after the car goes offroad), then analyzed. Recordings live on the device.
      </p>
      <div class="plotActions">
        ${rec ? html`
          <button class="plotButton plotButtonStop" disabled="${() => state.busy}" @click="${stopRecording}">Stop recording</button>
          <span class="plotRecording"><i class="plotRecDot"></i>Recording ${fmtDuration(rec.elapsed_s)} · ${rec.rows} samples</span>
        ` : html`
          <button class="plotButton" disabled="${() => state.busy}" @click="${startRecording}">Start recording</button>
          ${live.isOnroad ? "" : html`<span class="plotMuted">The car is offroad; recording picks up once it is onroad.</span>`}
        `}
      </div>
      <div class="plotStatusGrid">
        <p><strong>Onroad:</strong> ${live.isOnroad ? "Yes" : "No"}</p>
        <p><strong>Last sample:</strong> ${live.sampleAgeSeconds == null ? "none yet" : `${fmtNum(live.sampleAgeSeconds, 1)} s ago`}</p>
        <p><strong>Speed:</strong> ${state.latest ? `${fmtNum(state.latest.v, 1)} m/s` : "—"}</p>
        <p><strong>openpilot:</strong> ${!state.latest ? "—" : state.latest.enabled
          ? `engaged (steer ${state.latest.lat_active ? "on" : "off"}, long ${longStateName(state.latest.long_state)})` : "not engaged"}</p>
      </div>
      ${state.error ? html`<p class="plotError"><strong>Error:</strong> ${state.error}</p>` : ""}
      ${live.lastError ? html`<p class="plotError"><strong>Source error:</strong> ${live.lastError}</p>` : ""}
    </section>
  `
}

function LiveSection() {
  const live = state.live || {}
  const la = live.liveAnalysis
  return html`
    <section class="plotCard plotStatusCard">
      <div class="plotCardHeader">
        <h2>Live (last ${live.liveWindowSeconds || 30} s)</h2>
        <div class="plotActions plotActionsInline">
          <button class="plotButton" @click="${togglePaused}">${state.paused ? "Resume" : "Pause"}</button>
          <button class="plotButton" @click="${toggleAdvancedTerms}">${state.showAdvancedTerms ? "Hide controller terms" : "Show controller terms"}</button>
        </div>
      </div>
      ${live.stale && !state.loading ? html`<p class="plotMuted">No live data — openpilot is not running or the car is offroad.</p>` : ""}
      <div class="qualitySummaryGrid">
        ${la ? AnalysisBlock("Steering", "lateral", la.lateral, true) : ""}
        ${la ? AnalysisBlock("Speed control", "longitudinal", la.longitudinal, true) : ""}
      </div>
      <p class="qualityMethodNote">${HELP_TEXT}</p>
    </section>
    <div class="plotCharts">${state.liveCharts.map((c) => ChartCard(c))}</div>
  `
}

function SessionList() {
  return html`
    <section class="plotCard plotStatusCard">
      <h2>Saved drives</h2>
      ${state.sessions.length ? html`
        <div class="plotSessionList">
          ${state.sessions.map((s) => html`
            <button class="plotSessionRow ${s.id === state.selectedId ? "selected" : ""}" @click="${() => openSession(s.id)}">
              <span class="plotSessionHead">
                <strong>${fmtDate(s.started_at)}</strong>
                <span class="plotMuted">${s.duration_s == null ? "" : fmtDuration(s.duration_s)} · ${statusLabel(s.status)}${s.stop_reason ? ` (${s.stop_reason})` : ""}</span>
              </span>
              ${s.lateral_summary ? html`<span class="plotSessionSummary">${s.lateral_summary}</span>` : ""}
              ${s.longitudinal_summary ? html`<span class="plotSessionSummary">${s.longitudinal_summary}</span>` : ""}
              ${s.error ? html`<span class="plotError">${s.error}</span>` : ""}
            </button>
          `)}
        </div>
      ` : html`<p class="plotMuted">No saved drives yet.</p>`}
    </section>
  `
}

function SessionDetail() {
  if (!state.selectedId) return ""
  const d = state.detail
  const a = d?.analysis
  const m = d?.meta || {}
  const overview = a ? buildOverviewCharts(a.overview) : []
  return html`
    <section class="plotCard plotStatusCard plotDetail">
      <div class="plotCardHeader">
        <h2>Drive ${fmtDate(m.started_at)}</h2>
        <div class="plotActions plotActionsInline">
          ${m.status === "done" ? html`<a class="plotButton" href="${sessionUrl(state.selectedId, "/download")}">Download CSV</a>` : ""}
          ${m.status !== "recording" ? html`<button class="plotButton plotButtonStop" @click="${() => deleteSession(state.selectedId)}">Delete</button>` : ""}
          <button class="plotButton" @click="${closeSession}">Close</button>
        </div>
      </div>
      ${state.detailError ? html`<p class="plotError">${state.detailError}</p>` : ""}
      ${!d ? html`<p class="plotMuted">Loading...</p>` : ""}
      ${d && !a ? html`<p class="plotMuted">${m.status === "recording" ? "Still recording." : m.status === "error" ? `Analysis failed: ${m.error || ""}` : "Analyzing..."}</p>` : ""}
      ${a ? html`
        <div class="plotStatusGrid">
          <p><strong>Duration:</strong> ${fmtDuration(a.duration_s)}</p>
          <p><strong>Disengagements:</strong> ${a.disengagements}</p>
          <p><strong>Car:</strong> ${m.car || "—"}</p>
          <p><strong>Software:</strong> ${m.git_branch || "—"} ${m.git_commit ? `@ ${String(m.git_commit).slice(0, 8)}` : ""}</p>
        </div>
        <div class="qualitySummaryGrid">
          ${AnalysisBlock("Steering", "lateral", a.lateral)}
          ${AnalysisBlock("Speed control", "longitudinal", a.longitudinal)}
        </div>
        <p class="qualityMethodNote">${a.method}</p>
      ` : ""}
    </section>
    ${overview.length ? html`
      <p class="plotMuted">Tap a chart to zoom into ${2 * ZOOM_HALF_WINDOW_S} s at full resolution.</p>
      <div class="plotCharts">${overview.map((c) => ChartCard(c, zoomAt))}</div>
    ` : ""}
    ${state.zoom ? html`
      <section class="plotCard plotStatusCard">
        <div class="plotCardHeader">
          <h2>Zoom ${fmtDuration(state.zoom.start)} – ${fmtDuration(state.zoom.end)}</h2>
          <button class="plotButton" @click="${() => { state.zoom = null }}">Close zoom</button>
        </div>
      </section>
      <div class="plotCharts">${state.zoom.charts.map((c) => ChartCard(c))}</div>
    ` : ""}
  `
}

// arrow-core keeps a rendered template when its static strings match and only re-runs function slots, so a
// block whose shape changes (empty list -> rows, recording -> idle) would keep the stale DOM. A root whose markup
// carries a fresh revision forces the block to re-render, the same trick bluetooth.js uses.
let renderRevision = 0
function fresh(block) {
  renderRevision += 1
  return html([`<div class="plotBlock" data-rev="${renderRevision}">`, "</div>"], block)
}

async function initialize() {
  try {
    state.showAdvancedTerms = localStorage.getItem(ADVANCED_TERMS_KEY) === "1"
  } catch (error) {
    state.showAdvancedTerms = false
  }

  try {
    await Promise.all([fetchLiveData(), fetchSessions()])
  } catch (error) {
    state.error = error?.message || String(error)
    state.loading = false
  } finally {
    ensurePolling()
  }

  if (!visibilityListenerAttached) {
    visibilityListenerAttached = true
    document.addEventListener("visibilitychange", () => {
      if (document.visibilityState !== "visible") {
        stopPolling()
        return
      }
      if (isPlotsRouteActive() && !state.paused) ensurePolling()
    })
  }
}

export function LivePlots() {
  if (!initialized) {
    initialized = true
    initialize()
  }
  if (!state.paused) {
    ensurePolling()
  }

  return html`
    <div class="plotsPage">
      <h1>Plots</h1>
      ${() => fresh(RecordingCard())}
      ${() => fresh(SessionDetail())}
      ${() => fresh(LiveSection())}
      ${() => fresh(SessionList())}
      ${() => state.loading ? html`<p>Loading live data...</p>` : ""}
    </div>
  `
}
