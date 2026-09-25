import { api, showSnackbar } from "../api.js"
import { usePolling } from "../composables.js"
import { GalaxyConfirm } from "./GalaxyModal.js"
import { GxNotice } from "./GxNotice.js"

// Mobile face of the classic /lat_tune page (STATUS 117): same /api/lat_tune/* endpoints, same rules.
const MAX_ROUTES = 8
const MAX_RENDERED_ROUTES = 250
const DONE_STATES = ["complete", "failed", "cancelled_onroad", "cancelled"]

export const NrdrLatTunePanel = {
  name: "NrdrLatTunePanel",
  components: { GxNotice },
  data() {
    return {
      loading: true,
      busy: false,
      error: "",
      loadingRoutes: false,
      routes: [],
      selectedRoutes: [],
      workspace: { trials: [], activeStack: [], currentSchedule: "", currentFingerprint: "", currentBands: [] },
      status: {},
      isOnroad: false,
      expanded: {},
      maxRoutes: MAX_ROUTES,
    }
  },
  created() {
    this.poll = usePolling(() => this.refreshStatus(), { interval: 3000 })
    this.refresh()
    this.loadRoutes()
    this.poll.start()
  },
  beforeUnmount() { this.poll?.destroy() },
  computed: {
    canAnalyze() {
      return !this.busy && this.selectedRoutes.length > 0 && !this.isOnroad && !this.status.running
    },
    stackTop() {
      const stack = this.workspace.activeStack || []
      return stack.length ? stack[stack.length - 1] : ""
    },
  },
  methods: {
    fmt(v, d = 2) { return typeof v === "number" && Number.isFinite(v) ? v.toFixed(d) : "–" },
    bandRange(b) { return b.highMph == null ? `${b.lowMph}+ mph` : `${b.lowMph}–${b.highMph} mph` },
    pif(g) { return g ? `${g.p} / ${g.i} / ${g.f}` : "–" },
    isBandTrial(t) { return t && t.schemaVersion === 2 },
    kv(obj) { return Object.entries(obj || {}).map(([k, v]) => `${k}=${v === "" ? "(unset)" : v}`).join(", ") },
    fmtList(values, d = 2) { return (values || []).map((v) => this.fmt(v, d)).join(" / ") },
    when(ts) { return ts ? new Date(ts * 1000).toLocaleString() : "" },
    fmtDate(value) {
      const n = Number(value)
      if (Number.isFinite(n) && n > 0) return new Date(n > 100000000000 ? n : n * 1000).toLocaleString()
      return value ? String(value) : ""
    },
    errMsg(e, fallback) { return (e && (e.data?.error || e.message)) || fallback },
    async refresh() {
      try {
        const ws = await api.getLatTuneWorkspace()
        this.workspace = {
          trials: Array.isArray(ws?.trials) ? ws.trials : [],
          activeStack: Array.isArray(ws?.activeStack) ? ws.activeStack : [],
          currentSchedule: ws?.currentSchedule || "",
          currentFingerprint: ws?.currentFingerprint || "",
          currentBands: Array.isArray(ws?.currentBands) ? ws.currentBands : [],
        }
        this.error = ""
      } catch (e) {
        this.error = this.errMsg(e, "Failed to load the lateral tune workspace.")
      } finally {
        this.loading = false
      }
      await this.refreshStatus()
    },
    async refreshStatus() {
      try {
        const st = await api.getLatTuneStatus()
        const prev = this.status.state
        this.isOnroad = !!st?.isOnroad
        this.status = (st?.status && typeof st.status === "object") ? st.status : {}
        if (prev && prev !== this.status.state && DONE_STATES.includes(this.status.state)) await this.refresh()
      } catch (e) {
        this.error = this.errMsg(e, "Failed to read analysis status.")
      }
    },
    async loadRoutes() {
      if (this.loadingRoutes) return
      this.loadingRoutes = true
      this.routes = []
      try {
        await api.getRoutesStream({
          onProgress: () => {},
          onRoutes: (list) => {
            for (const route of (list || [])) {
              if (!route || !route.name || this.routes.some((r) => r.name === route.name)) continue
              if (this.routes.length >= MAX_RENDERED_ROUTES) return
              this.routes.push(route)
            }
          },
        })
      } catch (e) {
        showSnackbar(this.errMsg(e, "Failed to load routes."), "error")
      } finally {
        this.loadingRoutes = false
      }
    },
    toggleRoute(name) {
      if (this.selectedRoutes.includes(name)) {
        this.selectedRoutes = this.selectedRoutes.filter((r) => r !== name)
        return
      }
      if (this.selectedRoutes.length >= MAX_ROUTES) {
        showSnackbar(`Pick at most ${MAX_ROUTES} routes.`, "error")
        return
      }
      this.selectedRoutes = [...this.selectedRoutes, name]
    },
    selectLatest() { this.selectedRoutes = this.routes.slice(0, MAX_ROUTES).map((r) => r.name) },
    clearSelection() { this.selectedRoutes = [] },
    async runWith(fn, okMessage) {
      if (this.busy) return null
      this.busy = true
      this.error = ""
      try {
        const payload = await fn()
        showSnackbar(payload && payload.message ? payload.message : okMessage)
        return payload
      } catch (e) {
        this.error = this.errMsg(e, "Action failed.")
        showSnackbar(this.error, "error")
        throw e
      } finally {
        this.busy = false
        await this.refresh()
      }
    },
    async analyze() {
      if (!this.canAnalyze) return
      const ok = await this.runWith(() => api.latTuneAnalyze(this.selectedRoutes), "Analysis started.").catch(() => null)
      if (ok) this.selectedRoutes = []
    },
    async stopAnalyze() {
      await this.runWith(() => api.latTuneStopAnalyze(), "Analysis stopped.").catch(() => {})
    },
    async applyTrial(trial) {
      const ok = await GalaxyConfirm({
        title: "Apply Trial",
        message: `Apply trial ${trial.trialId}? This writes the PID band P scales LatPScaleLowSpeed/Standard/Highway (I and F unchanged) and drops any P term from LatGainSchedule. You can revert it here.`,
        confirmLabel: "Apply",
      })
      if (!ok) return
      try {
        await this.runWith(() => api.latTuneApplyTrial(trial.trialId), "Trial applied.")
      } catch (e) {
        if (!/fingerprint/i.test(this.errMsg(e, ""))) return
        const force = await GalaxyConfirm({
          title: "Tuning Changed",
          message: "Your manual lateral tuning changed since these routes were driven. Apply anyway?",
          confirmLabel: "Apply Anyway",
          danger: true,
        })
        if (force) await this.runWith(() => api.latTuneApplyTrial(trial.trialId, true), "Trial applied.").catch(() => {})
      }
    },
    async revertTrial(trial) {
      const ok = await GalaxyConfirm({
        title: "Revert Trial",
        message: `Revert trial ${trial.trialId}? The previous P band scales and LatGainSchedule are restored.`,
        confirmLabel: "Revert",
      })
      if (ok) await this.runWith(() => api.latTuneRevertTrial(trial.trialId), "Trial reverted.").catch(() => {})
    },
    async deleteTrial(trial) {
      const ok = await GalaxyConfirm({ title: "Delete Trial", message: `Delete trial ${trial.trialId}?`, confirmLabel: "Delete", danger: true })
      if (ok) await this.runWith(() => api.latTuneDeleteTrial(trial.trialId), "Trial deleted.").catch(() => {})
    },
    async toggleDetails(trial) {
      const id = trial.trialId
      if (this.expanded[id]) {
        this.expanded = { ...this.expanded, [id]: null }
        return
      }
      try {
        this.expanded = { ...this.expanded, [id]: await api.getLatTuneTrial(id) }
      } catch (e) {
        showSnackbar(this.errMsg(e, "Failed to load trial."), "error")
      }
    },
  },
  template: `
    <div>
      <section class="gx-card">
        <div class="gx-section__header">
          <i class="bi bi-sliders"></i>
          <span class="gx-section__title">NRDR PID lateral tune</span>
        </div>
        <div style="padding: var(--sp-4);">
          <p style="color: var(--text-muted); line-height:1.6; margin:0 0 var(--sp-3);">
            Pick up to {{ maxRoutes }} routes. The device analyzes them while parked and proposes one P step per NRDR PID speed band
            (0–25, 25–50, 50+ mph; factor 0.85–1.15, written on the 5 % grid). Only P changes; I and F are shown and kept. Each run is a trial you can apply and revert.
            Unit-test/replay evidence only; nothing here is road-validated.
          </p>
          <GxNotice v-if="isOnroad" text="Analyze, apply and revert are offroad-only. Park and go offroad first." style="margin:0 0 var(--sp-3);" />
          <GxNotice v-if="status.state === 'failed' && status.error" tone="danger" :text="status.error" style="margin:0 0 var(--sp-3);" />
          <GxNotice v-if="error" tone="danger" :text="error" style="margin:0 0 var(--sp-3);" />

          <div style="display:flex; gap:8px; margin-bottom: var(--sp-3); flex-wrap:wrap;">
            <button type="button" class="gx-btn" :disabled="!canAnalyze" @click="analyze">
              <i class="bi bi-graph-up-arrow"></i> Analyze {{ selectedRoutes.length }} route(s)
            </button>
            <button type="button" class="gx-btn gx-btn--tonal" :disabled="busy || !status.running" @click="stopAnalyze">
              <i class="bi bi-stop-fill"></i> Stop
            </button>
            <button type="button" class="gx-icon-btn" title="Refresh" :disabled="busy || loading" @click="refresh"><i class="bi bi-arrow-clockwise"></i></button>
          </div>

          <div v-if="loading" class="gx-loading">Loading lateral tune workspace...</div>
          <div class="gx-row" style="border-top:none;"><span class="gx-row__label">State</span><span class="gx-row__value">{{ status.state || 'idle' }}</span></div>
          <div class="gx-row"><span class="gx-row__label">Onroad</span><span class="gx-row__value">{{ isOnroad ? 'Yes (parked only)' : 'No' }}</span></div>
          <div v-if="status.total" class="gx-row"><span class="gx-row__label">Progress</span><span class="gx-row__value">{{ status.progress || 0 }} / {{ status.total }}</span></div>
          <div v-if="status.currentSegment" class="gx-row"><span class="gx-row__label">Segment</span><span class="gx-row__value">{{ status.currentSegment }}</span></div>
          <div class="gx-row"><span class="gx-row__label">Applied stack</span><span class="gx-row__value">{{ (workspace.activeStack || []).join(' → ') || 'none' }}</span></div>
          <div style="border-top:1px solid var(--glass-border); padding: var(--sp-2) 0 0;">
            <div class="gx-row__label">Current P / I / F</div>
            <div v-for="b in workspace.currentBands" :key="'cb' + b.name" class="gx-row__desc">{{ b.name }} ({{ bandRange(b) }}): {{ pif(b) }}</div>
            <div class="gx-row__label" style="margin-top:6px;">Current LatGainSchedule</div>
            <div class="gx-row__desc" style="word-break:break-all; font-family:monospace;">{{ workspace.currentSchedule || '(none)' }}</div>
          </div>
        </div>
      </section>

      <section class="gx-card" style="margin-top: var(--sp-3);">
        <div class="gx-section__header">
          <i class="bi bi-journal-arrow-down"></i>
          <span class="gx-section__title">Local Routes</span>
        </div>
        <div style="padding: var(--sp-4);">
          <div v-if="loadingRoutes" class="gx-loading">Loading local routes...</div>
          <div v-else-if="!routes.length" class="gx-empty">No local routes found.</div>
          <div v-else>
            <span class="gx-chip">{{ selectedRoutes.length }}/{{ maxRoutes }} routes selected</span>
            <button type="button" class="gx-btn gx-btn--text" style="font-size:var(--fs-xs);" @click="selectLatest">Select latest {{ maxRoutes }}</button>
            <button type="button" class="gx-btn gx-btn--text" style="font-size:var(--fs-xs);" @click="clearSelection">Clear</button>
            <div v-for="route in routes" :key="route.name" style="border-top:1px solid var(--glass-border); padding: var(--sp-2) 0;">
              <label style="display:flex; gap:10px; align-items:flex-start; cursor:pointer;">
                <input type="checkbox" :checked="selectedRoutes.includes(route.name)" :disabled="!selectedRoutes.includes(route.name) && selectedRoutes.length >= maxRoutes" @change="toggleRoute(route.name)" style="margin-top:4px;" />
                <span style="min-width:0;">
                  <strong>{{ fmtDate(route.timestamp) || route.name }}</strong>
                  <div class="gx-row__desc" style="word-break:break-all;">{{ route.name }}</div>
                  <div class="gx-row__desc">{{ route.segmentCount || 0 }} segment(s)</div>
                </span>
              </label>
            </div>
          </div>
        </div>
      </section>

      <section class="gx-card" style="margin-top: var(--sp-3);">
        <div class="gx-section__header">
          <i class="bi bi-collection"></i>
          <span class="gx-section__title">Trials</span>
        </div>
        <div style="padding: var(--sp-4);">
          <div v-if="!workspace.trials.length" class="gx-empty">No trials yet. Analyze routes to create one.</div>
          <div v-for="t in workspace.trials" :key="t.trialId" style="border-top:1px solid var(--glass-border); padding: var(--sp-2) 0;">
            <strong>{{ t.trialId }}</strong>
            <span v-if="t.applied" class="gx-chip" style="background:var(--primary);color:var(--on-primary); margin-left:6px;">Applied</span>
            <div class="gx-row__desc">{{ when(t.createdAt) }}<span v-if="t.applied"> · applied {{ when(t.applied.at) }}</span></div>
            <div class="gx-row__desc" style="word-break:break-all;">{{ (t.routeNames || []).length }} route(s): {{ (t.routeNames || []).join(', ') }}</div>
            <template v-if="isBandTrial(t)">
              <div class="gx-row__desc">Ready: {{ (t.readyBands || []).join(', ') || 'none' }}</div>
              <div class="gx-row__desc">P low / std / hwy: {{ (t.currentP || []).join(' / ') }} → {{ (t.proposedP || []).join(' / ') }}</div>
            </template>
            <div v-else class="gx-row__desc" style="color:var(--error);">⚠ Pre-band trial (20/30/40/50 mph knots); re-analyze for NRDR PID band values.</div>
            <div v-for="w in (t.warnings || [])" :key="w" class="gx-row__desc" style="color:var(--error);">⚠ {{ w }}</div>
            <div style="display:flex; gap:6px; flex-wrap:wrap; margin-top:6px;">
              <button type="button" class="gx-btn gx-btn--text" @click="toggleDetails(t)">{{ expanded[t.trialId] ? 'Hide' : 'Details' }}</button>
              <button type="button" class="gx-btn gx-btn--tonal" :disabled="busy || isOnroad || !!t.applied || !isBandTrial(t) || !(t.readyBands || []).length" @click="applyTrial(t)">Apply</button>
              <button type="button" class="gx-btn gx-btn--tonal" :disabled="busy || isOnroad || !t.applied || stackTop !== t.trialId" @click="revertTrial(t)">Revert</button>
              <button type="button" class="gx-btn gx-btn--text" :disabled="busy || !!t.applied" style="color:var(--error);" @click="deleteTrial(t)">Delete</button>
            </div>
            <div v-if="expanded[t.trialId]" style="margin-top:8px; overflow-x:auto;">
              <div v-if="!Array.isArray(expanded[t.trialId].bands)" class="gx-row__desc" style="color:var(--error);">⚠ This trial predates the speed bands; re-analyze the routes.</div>
              <template v-else>
              <table style="width:100%; border-collapse:collapse; font-size:var(--fs-xs);">
                <thead><tr style="text-align:left; color:var(--text-muted);"><th>Band</th><th>Min</th><th>Ready</th><th>P/I/F now</th><th>P new</th></tr></thead>
                <tbody>
                  <tr v-for="b in expanded[t.trialId].bands" :key="b.name" style="border-top:1px solid var(--glass-border);">
                    <td>{{ b.name }}<br /><span style="color:var(--text-muted);">{{ bandRange(b) }}</span></td><td>{{ fmt(b.minutes, 1) }}</td>
                    <td :style="b.ready ? 'color:var(--primary);' : 'color:var(--text-muted);'">{{ b.ready ? 'yes' : 'need 3 min' }}</td>
                    <td>{{ pif(b.current) }}</td>
                    <td :style="b.proposed.p !== b.current.p ? 'color:var(--primary); font-weight:600;' : ''">{{ b.proposed.p }}</td>
                  </tr>
                </tbody>
              </table>
              <div v-for="b in expanded[t.trialId].bands" :key="'r' + b.name" class="gx-row__desc">
                sign {{ fmt(b.signRate) }}/s · curve {{ fmt(b.curveRatio, 3) }} · overrides {{ fmt(b.pressRate) }}/min · factor {{ fmt(b.factor) }} — {{ b.reason }}
              </div>
              <div class="gx-row__desc">Band values from the newest route's logs (fingerprint {{ (expanded[t.trialId].baseline || {}).fingerprint || '–' }}).</div>
              </template>
              <div v-if="expanded[t.trialId].applied" class="gx-row__desc" style="word-break:break-all; font-family:monospace;">
                written: {{ kv(expanded[t.trialId].applied.writtenParams) }}<br />prior: {{ kv(expanded[t.trialId].applied.priorParams) }}<br />LatGainSchedule prior: {{ expanded[t.trialId].applied.priorSchedule || '(none)' }}
              </div>
            </div>
          </div>
        </div>
      </section>
    </div>
  `,
}
