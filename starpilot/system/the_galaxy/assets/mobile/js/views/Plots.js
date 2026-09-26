import { api, showSnackbar } from "../api.js"
import { usePolling } from "../composables.js"
import { GalaxyConfirm } from "../components/GalaxyModal.js"
import {
  HELP_TEXT, LIVE_POLL_MS, LiveBuffer, ZOOM_HALF_WINDOW_S,
  buildOverviewCharts, buildTrackingCharts, fmtDate, fmtDuration, fmtNum, keyNumbers,
  longStateName, sessionUrl, statusLabel, timeAtClick, toSeries,
} from "/assets/components/tools/drive_plots_shared.mjs"

const ADVANCED_TERMS_KEY = "plotsShowAdvancedTerms"
const SESSIONS_POLL_MS = 5000

// One chart: SVG geometry from drive_plots_shared.buildChart, drawn without a canvas so it scales with the card.
const PlotChart = {
  name: "PlotChart",
  props: { chart: { type: Object, required: true }, clickable: { type: Boolean, default: false } },
  emits: ["pick"],
  methods: {
    onClick(e) {
      if (!this.clickable) return
      const t = timeAtClick(e, this.chart.geo)
      if (t !== null) this.$emit("pick", t)
    },
  },
  template: `
    <section class="gx-card" style="overflow:hidden;">
      <div class="gx-section__header">
        <i class="bi bi-activity"></i>
        <span class="gx-section__title">{{ chart.title }}</span>
        <span v-if="chart.unit" style="margin-left:auto; color: var(--text-muted); font-size: var(--fs-xs, 0.8rem);">{{ chart.unit }}</span>
      </div>
      <div style="padding: var(--sp-2) var(--sp-3) var(--sp-3);">
        <div style="display:flex; gap:12px; flex-wrap:wrap; margin-bottom: var(--sp-2);">
          <span v-for="item in chart.legend" :key="item.label" style="display:inline-flex; align-items:center; gap:4px; font-size: var(--fs-xs, 0.8rem);">
            <i :style="{ width: '12px', height: '3px', borderRadius: '2px', display: 'inline-block', background: item.color }"></i>
            {{ item.label }}<template v-if="item.value">: {{ item.value }}</template>
          </span>
        </div>
        <div v-if="chart.geo.empty" class="gx-empty">Waiting for data...</div>
        <div v-else style="position:relative;">
          <svg :viewBox="'0 0 ' + chart.geo.width + ' ' + chart.geo.height" preserveAspectRatio="none"
               :style="{ width: '100%', height: '180px', display: 'block', cursor: clickable ? 'zoom-in' : 'default',
                         background: 'rgba(0,0,0,0.25)', borderRadius: '6px' }"
               role="img" :aria-label="chart.title" @click="onClick">
            <rect v-for="(s, i) in chart.geo.shade" :key="'s' + i" :x="s.x" y="0" :width="s.w" :height="chart.geo.height" fill="rgba(255,255,255,0.08)"></rect>
            <line v-for="(l, i) in chart.geo.grid" :key="'g' + i" x1="0" :y1="l.y" :x2="chart.geo.width" :y2="l.y"
                  :stroke="l.zero ? 'rgba(255,255,255,0.35)' : 'rgba(255,255,255,0.08)'" :stroke-dasharray="l.zero ? '6 6' : null"
                  vector-effect="non-scaling-stroke"></line>
            <line v-for="(x, i) in chart.geo.xTicks" :key="'x' + i" :x1="x.x" y1="0" :x2="x.x" :y2="chart.geo.height"
                  stroke="rgba(255,255,255,0.08)" vector-effect="non-scaling-stroke"></line>
            <path v-for="p in chart.geo.paths" :key="p.key" :d="p.d" fill="none" :stroke="p.color" stroke-width="2"
                  stroke-linejoin="round" vector-effect="non-scaling-stroke"></path>
          </svg>
          <span v-for="(l, i) in chart.geo.grid.filter((g, j) => j % 2 === 0)" :key="'yl' + i"
                :style="{ position: 'absolute', left: '4px', top: 'calc(' + l.pct + '% - 7px)', fontSize: '10px', color: 'var(--text-muted)' }">{{ l.label }}</span>
          <div style="position:relative; height:14px; margin-top:2px;">
            <span v-for="(x, i) in chart.geo.xTicks" :key="'xl' + i"
                  :style="{ position: 'absolute', left: x.pct + '%', transform: 'translateX(-50%)', fontSize: '10px', color: 'var(--text-muted)' }">{{ x.label }}</span>
          </div>
        </div>
      </div>
    </section>
  `,
}

export const Plots = {
  name: "Plots",
  components: { PlotChart },
  props: { embedded: { type: Boolean, default: false } },
  data() {
    return {
      loading: true,
      error: "",
      paused: false,
      showAdvancedTerms: false,
      busy: false,
      live: null,
      latest: null,
      liveCharts: [],
      sessions: [],
      selectedId: "",
      detail: null,
      detailError: "",
      zoom: null,
    }
  },
  created() {
    this.buffer = new LiveBuffer(60)
    this.lastSessionsFetch = 0
    this.helpText = HELP_TEXT
    this.zoomSeconds = 2 * ZOOM_HALF_WINDOW_S
    try {
      this.showAdvancedTerms = localStorage.getItem(ADVANCED_TERMS_KEY) === "1"
    } catch (e) { this.showAdvancedTerms = false }
    this.poll = usePolling(() => this.load(), { interval: LIVE_POLL_MS, enabled: () => !this.paused })
    this.poll.start()
    this.loadSessions().catch(() => {})
  },
  beforeUnmount() { this.poll?.destroy() },
  computed: {
    recording() { return this.live?.recording || null },
    statusRows() {
      const live = this.live || {}
      const s = this.latest
      return [
        { label: "Onroad", value: live.isOnroad ? "Yes" : "No" },
        { label: "Last sample", value: live.sampleAgeSeconds == null ? "none yet" : `${fmtNum(live.sampleAgeSeconds, 1)} s ago` },
        { label: "Speed", value: s ? `${fmtNum(s.v, 1)} m/s` : "—" },
        { label: "openpilot", value: !s ? "—" : s.enabled ? `engaged (steer ${s.lat_active ? "on" : "off"}, long ${longStateName(s.long_state)})` : "not engaged" },
      ]
    },
    liveBlocks() {
      const la = this.live?.liveAnalysis
      if (!la) return []
      return [
        { title: "Steering", m: la.lateral },
        { title: "Speed control", m: la.longitudinal },
      ]
    },
    detailMeta() { return this.detail?.meta || {} },
    analysis() { return this.detail?.analysis || null },
    detailBlocks() {
      const a = this.analysis
      if (!a) return []
      return [
        { title: "Steering", m: a.lateral, numbers: keyNumbers("lateral", a.lateral) },
        { title: "Speed control", m: a.longitudinal, numbers: keyNumbers("longitudinal", a.longitudinal) },
      ]
    },
    overviewCharts() { return this.analysis ? buildOverviewCharts(this.analysis.overview) : [] },
    downloadUrl() { return this.selectedId ? sessionUrl(this.selectedId, "/download") : "" },
  },
  methods: {
    fmtDate,
    fmtDuration,
    statusLabel,
    rebuildLive() {
      this.liveCharts = buildTrackingCharts(this.buffer.view(), { advanced: this.showAdvancedTerms })
      this.latest = this.buffer.latest()
    },
    async load() {
      try {
        const payload = await api.getPlotsLive(this.buffer.seq)
        const hadRecording = !!this.live?.recording
        this.live = payload && typeof payload === "object" ? payload : this.live
        this.error = ""
        this.loading = false
        if (this.buffer.ingest(payload)) this.rebuildLive()
        if (hadRecording !== !!payload?.recording || Date.now() - this.lastSessionsFetch > SESSIONS_POLL_MS) {
          this.loadSessions().catch(() => {})
        }
      } catch (e) {
        this.error = e?.message || "Failed to load live plot data"
        this.loading = false
        throw e
      }
    },
    async loadSessions() {
      this.lastSessionsFetch = Date.now()
      const payload = await api.getPlotsSessions()
      this.sessions = Array.isArray(payload?.sessions) ? payload.sessions : []
      const selected = this.sessions.find((s) => s.id === this.selectedId)
      if (selected && this.detail && this.detail.meta?.status !== selected.status) this.openSession(selected.id)
    },
    async startRecording() {
      this.busy = true
      try {
        await api.startPlotsRecording()
        showSnackbar("Recording started.")
        await this.load()
      } catch (e) {
        showSnackbar(e?.message || "Could not start recording", "error")
      } finally {
        this.busy = false
      }
    },
    async stopRecording() {
      this.busy = true
      try {
        const payload = await api.stopPlotsRecording()
        showSnackbar("Recording stopped. Analyzing the drive...")
        await this.load()
        await this.loadSessions()
        if (payload?.stopped?.id) this.openSession(payload.stopped.id)
      } catch (e) {
        showSnackbar(e?.message || "Could not stop recording", "error")
      } finally {
        this.busy = false
      }
    },
    async openSession(id) {
      this.selectedId = id
      this.detailError = ""
      this.zoom = null
      try {
        this.detail = await api.getPlotsSession(id)
      } catch (e) {
        this.detail = null
        this.detailError = e?.message || String(e)
      }
    },
    closeSession() {
      this.selectedId = ""
      this.detail = null
      this.zoom = null
    },
    async deleteSession(id) {
      const ok = await GalaxyConfirm({ title: "Delete saved drive?", message: "This cannot be undone.", confirmLabel: "Delete", danger: true })
      if (!ok) return
      try {
        await api.deletePlotsSession(id)
        if (this.selectedId === id) this.closeSession()
        await this.loadSessions()
        showSnackbar("Drive deleted.")
      } catch (e) {
        showSnackbar(e?.message || "Delete failed", "error")
      }
    },
    async zoomAt(t) {
      if (!this.selectedId) return
      const start = Math.max(0, t - ZOOM_HALF_WINDOW_S)
      const end = start + 2 * ZOOM_HALF_WINDOW_S
      try {
        const payload = await api.getPlotsSessionWindow(this.selectedId, start, end)
        const series = toSeries(payload.columns, payload.rows)
        this.zoom = { start, end, charts: buildTrackingCharts(series, { advanced: this.showAdvancedTerms, tMin: start, tMax: end }) }
      } catch (e) {
        this.detailError = e?.message || String(e)
      }
    },
    togglePaused() {
      this.paused = !this.paused
      if (this.paused) return
      this.load().catch((e) => {
        this.error = e?.message || "Failed to resume live data"
      })
      if (this.poll) this.poll.start()
    },
    toggleAdvancedTerms() {
      this.showAdvancedTerms = !this.showAdvancedTerms
      try {
        localStorage.setItem(ADVANCED_TERMS_KEY, this.showAdvancedTerms ? "1" : "0")
      } catch (e) { console.warn("Failed to persist plots advanced terms preference", e) }
      this.rebuildLive()
    },
  },
  template: `
    <div class="gx-view">
      <h2 v-if="!embedded" style="margin-top:0;">Plots</h2>

      <section class="gx-card">
        <div class="gx-section__header">
          <i class="bi bi-record-circle"></i>
          <span class="gx-section__title">Drive recording</span>
        </div>
        <div style="padding: var(--sp-3);">
          <p style="color: var(--text-muted); line-height:1.6; margin:0 0 var(--sp-3);">
            Press Start recording before a drive. What openpilot asks for and what the car does is saved at 20 Hz until
            you press Stop (or 30 s after the car goes offroad), then analyzed. Recordings stay on the device.
          </p>
          <div style="display:flex; gap:8px; flex-wrap:wrap; align-items:center;">
            <button v-if="recording" type="button" class="gx-btn gx-btn--danger" :disabled="busy" @click="stopRecording">
              <i class="bi bi-stop-fill"></i> Stop recording
            </button>
            <button v-else type="button" class="gx-btn" :disabled="busy" @click="startRecording">
              <i class="bi bi-record-fill"></i> Start recording
            </button>
            <span v-if="recording" style="display:inline-flex; align-items:center; gap:6px; font-weight: var(--fw-bold, 600);">
              <i class="bi bi-circle-fill" style="color:#e05577; font-size:10px;"></i>
              {{ fmtDuration(recording.elapsed_s) }} · {{ recording.rows }} samples
            </span>
            <span v-else-if="live && !live.isOnroad" style="color: var(--text-muted); font-size: var(--fs-xs, 0.8rem);">
              Offroad; recording picks up once the car is onroad.
            </span>
          </div>

          <div v-if="loading" class="gx-loading" style="margin-top: var(--sp-3);">Loading live data...</div>
          <div v-if="error" class="gx-alert gx-alert--warn" style="border:none; margin: var(--sp-3) 0 0;">
            <i class="bi bi-exclamation-triangle-fill gx-alert__icon"></i>
            <div class="gx-alert__body"><strong>Error:</strong> <span>{{ error }}</span></div>
          </div>
          <div v-if="live?.lastError" class="gx-alert gx-alert--warn" style="margin: var(--sp-3) 0 0;">
            <i class="bi bi-exclamation-triangle gx-alert__icon"></i>
            <div class="gx-alert__body"><strong>Source error:</strong> <span>{{ live.lastError }}</span></div>
          </div>

          <div style="margin-top: var(--sp-3);">
            <div v-for="(row, i) in statusRows" :key="row.label" class="gx-row" :style="i === 0 ? 'border-top:none;' : ''">
              <span class="gx-row__label">{{ row.label }}</span><span class="gx-row__value">{{ row.value }}</span>
            </div>
          </div>
        </div>
      </section>

      <section v-if="selectedId" class="gx-card" style="margin-top: var(--sp-3);">
        <div class="gx-section__header">
          <i class="bi bi-clipboard-data"></i>
          <span class="gx-section__title">Drive {{ fmtDate(detailMeta.started_at) }}</span>
        </div>
        <div style="padding: var(--sp-3);">
          <div style="display:flex; gap:8px; flex-wrap:wrap; margin-bottom: var(--sp-3);">
            <a v-if="detailMeta.status === 'done'" class="gx-btn gx-btn--tonal" :href="downloadUrl"><i class="bi bi-download"></i> CSV</a>
            <button v-if="detailMeta.status && detailMeta.status !== 'recording'" type="button" class="gx-btn gx-btn--tonal" @click="deleteSession(selectedId)">
              <i class="bi bi-trash"></i> Delete
            </button>
            <button type="button" class="gx-btn gx-btn--tonal" @click="closeSession"><i class="bi bi-x-lg"></i> Close</button>
          </div>
          <div v-if="detailError" class="gx-alert gx-alert--warn" style="margin:0 0 var(--sp-2);">
            <i class="bi bi-exclamation-triangle gx-alert__icon"></i>
            <div class="gx-alert__body"><span>{{ detailError }}</span></div>
          </div>
          <div v-if="!detail && !detailError" class="gx-loading">Loading...</div>
          <p v-else-if="detail && !analysis" style="color: var(--text-muted); margin:0;">
            {{ detailMeta.status === 'recording' ? 'Still recording.' : detailMeta.status === 'error' ? 'Analysis failed: ' + (detailMeta.error || '') : 'Analyzing...' }}
          </p>
          <template v-if="analysis">
            <div class="gx-row" style="border-top:none;"><span class="gx-row__label">Duration</span><span class="gx-row__value">{{ fmtDuration(analysis.duration_s) }}</span></div>
            <div class="gx-row"><span class="gx-row__label">Disengagements</span><span class="gx-row__value">{{ analysis.disengagements }}</span></div>
            <div class="gx-row"><span class="gx-row__label">Car</span><span class="gx-row__value">{{ detailMeta.car || '—' }}</span></div>
            <div v-for="b in detailBlocks" :key="b.title" style="margin-top: var(--sp-3);">
              <p style="margin:0; font-weight: var(--fw-bold, 600);">{{ b.title }}</p>
              <p style="margin:2px 0 0;">{{ b.m.summary }}</p>
              <ul v-if="b.m.notes && b.m.notes.length" style="margin: 4px 0 0; padding-left: 1.2em; color: var(--text-muted); font-size: var(--fs-xs, 0.8rem); line-height:1.5;">
                <li v-for="n in b.m.notes" :key="n">{{ n }}</li>
              </ul>
              <div v-for="k in b.numbers" :key="k.label" class="gx-row">
                <span class="gx-row__label">{{ k.label }}</span><span class="gx-row__value">{{ k.value }}</span>
              </div>
            </div>
            <p style="color: var(--text-muted); font-size: var(--fs-xs, 0.8rem); margin: var(--sp-3) 0 0; line-height:1.6;">{{ analysis.method }}</p>
          </template>
        </div>
      </section>

      <template v-if="selectedId && overviewCharts.length">
        <p style="color: var(--text-muted); font-size: var(--fs-xs, 0.8rem); margin: var(--sp-3) 0 0;">
          Tap a chart to zoom into {{ zoomSeconds }} s at full resolution.
        </p>
        <div style="display:grid; gap: var(--sp-3); margin-top: var(--sp-2);">
          <PlotChart v-for="c in overviewCharts" :key="'o' + c.id" :chart="c" clickable @pick="zoomAt"></PlotChart>
        </div>
      </template>

      <template v-if="zoom">
        <div style="display:flex; align-items:center; gap:8px; margin-top: var(--sp-3);">
          <strong>Zoom {{ fmtDuration(zoom.start) }} – {{ fmtDuration(zoom.end) }}</strong>
          <button type="button" class="gx-btn gx-btn--tonal" @click="zoom = null"><i class="bi bi-x-lg"></i> Close zoom</button>
        </div>
        <div style="display:grid; gap: var(--sp-3); margin-top: var(--sp-2);">
          <PlotChart v-for="c in zoom.charts" :key="'z' + c.id" :chart="c"></PlotChart>
        </div>
      </template>

      <section class="gx-card" style="margin-top: var(--sp-3);">
        <div class="gx-section__header">
          <i class="bi bi-graph-up-arrow"></i>
          <span class="gx-section__title">Live (last {{ live?.liveWindowSeconds || 30 }} s)</span>
        </div>
        <div style="padding: var(--sp-3);">
          <p v-if="live?.stale && !loading" style="color: var(--text-muted); margin:0 0 var(--sp-2);">
            No live data — openpilot is not running or the car is offroad.
          </p>
          <div v-for="b in liveBlocks" :key="b.title" style="margin-bottom: var(--sp-2);">
            <p style="margin:0; font-weight: var(--fw-bold, 600);">{{ b.title }}</p>
            <p style="margin:2px 0 0;">{{ b.m.summary }}</p>
            <ul v-if="b.m.notes && b.m.notes.length" style="margin: 4px 0 0; padding-left: 1.2em; color: var(--text-muted); font-size: var(--fs-xs, 0.8rem); line-height:1.5;">
              <li v-for="n in b.m.notes" :key="n">{{ n }}</li>
            </ul>
          </div>
          <p style="color: var(--text-muted); font-size: var(--fs-xs, 0.8rem); margin: var(--sp-2) 0 0; line-height:1.6;">{{ helpText }}</p>
          <div style="display:flex; gap:8px; flex-wrap:wrap; margin-top: var(--sp-3);">
            <button type="button" class="gx-btn gx-btn--tonal" @click="togglePaused">
              <i class="bi" :class="paused ? 'bi-play-fill' : 'bi-pause-fill'"></i> {{ paused ? 'Resume' : 'Pause' }}
            </button>
            <button type="button" class="gx-btn gx-btn--tonal" @click="toggleAdvancedTerms">
              <i class="bi" :class="showAdvancedTerms ? 'bi-eye-slash' : 'bi-sliders'"></i>
              {{ showAdvancedTerms ? 'Hide controller terms' : 'Show controller terms' }}
            </button>
          </div>
        </div>
      </section>

      <div style="display:grid; gap: var(--sp-3); margin-top: var(--sp-3);">
        <PlotChart v-for="c in liveCharts" :key="'l' + c.id" :chart="c"></PlotChart>
      </div>

      <section class="gx-card" style="margin-top: var(--sp-3);">
        <div class="gx-section__header">
          <i class="bi bi-collection"></i>
          <span class="gx-section__title">Saved drives</span>
        </div>
        <div v-if="!sessions.length" class="gx-empty">No saved drives yet.</div>
        <button v-for="s in sessions" :key="s.id" type="button" class="gx-row"
                :style="{ display: 'block', width: '100%', textAlign: 'left', background: s.id === selectedId ? 'rgba(122,162,247,0.12)' : 'transparent', border: 'none', color: 'inherit', cursor: 'pointer' }"
                @click="openSession(s.id)">
          <span style="display:flex; justify-content:space-between; gap:8px; flex-wrap:wrap;">
            <strong>{{ fmtDate(s.started_at) }}</strong>
            <span style="color: var(--text-muted); font-size: var(--fs-xs, 0.8rem);">
              {{ s.duration_s == null ? '' : fmtDuration(s.duration_s) + ' · ' }}{{ statusLabel(s.status) }}
            </span>
          </span>
          <span v-if="s.lateral_summary" style="display:block; font-size: var(--fs-xs, 0.8rem); margin-top:2px;">{{ s.lateral_summary }}</span>
          <span v-if="s.longitudinal_summary" style="display:block; font-size: var(--fs-xs, 0.8rem);">{{ s.longitudinal_summary }}</span>
          <span v-if="s.error" style="display:block; font-size: var(--fs-xs, 0.8rem); color:#e05577;">{{ s.error }}</span>
        </button>
      </section>
    </div>
  `,
}
