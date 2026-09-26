// Shared, DOM-free helpers for the Plots page (classic and mobile).
// The server (drive_plots.py) does all scoring; this file only buffers rows and turns them into SVG geometry.

export const LIVE_POLL_MS = 500
export const LIVE_VIEW_SECONDS = 30
export const ZOOM_HALF_WINDOW_S = 30
export const CHART_W = 1000
export const CHART_H = 220

export const LINE_COLORS = {
  desired: "#7aa2f7",
  actual: "#9ece6a",
  speed: "#e0af68",
  p: "#63b3ff",
  i: "#63d79d",
  d: "#f0b35e",
  f: "#d08bff",
}

const LONG_STATE_NAMES = ["off", "pid", "stopping", "starting"]

export function toNumber(value, fallback = 0) {
  const n = Number(value)
  return Number.isFinite(n) ? n : fallback
}

export function fmtNum(value, digits = 2) {
  if (value === null || value === undefined || !Number.isFinite(Number(value))) return "—"
  return Number(value).toFixed(digits)
}

export function fmtDuration(seconds) {
  const s = Math.max(0, Math.round(toNumber(seconds)))
  const h = Math.floor(s / 3600)
  const m = Math.floor((s % 3600) / 60)
  const sec = s % 60
  if (h) return `${h}h ${String(m).padStart(2, "0")}m`
  if (m) return `${m}m ${String(sec).padStart(2, "0")}s`
  return `${sec}s`
}

export function fmtDate(epochSeconds) {
  const v = toNumber(epochSeconds, 0)
  if (!v) return "—"
  return new Date(v * 1000).toLocaleString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" })
}

export function longStateName(value) {
  return LONG_STATE_NAMES[Math.round(toNumber(value))] || "?"
}

// Column-major view of a {columns, rows} payload.
export function toSeries(columns, rows) {
  const out = {}
  const cols = Array.isArray(columns) ? columns : []
  cols.forEach((name, idx) => { out[name] = (rows || []).map((r) => toNumber(r[idx])) })
  return out
}

// Incremental 20 Hz buffer fed by /api/plots/live?since=<seq>.
export class LiveBuffer {
  constructor(keepSeconds = 60) {
    this.keepSeconds = keepSeconds
    this.reset()
  }

  reset() {
    this.columns = null
    this.rows = []
    this.seq = 0
  }

  // Returns true when new rows arrived.
  ingest(payload) {
    if (!payload || !Array.isArray(payload.columns)) return false
    const seq = toNumber(payload.seq, 0)
    // The recorder restarted (new process): its sequence numbers start over.
    if (seq < this.seq) this.reset()
    this.columns = payload.columns
    const rows = Array.isArray(payload.rows) ? payload.rows : []
    if (!rows.length) return false
    const seqIdx = this.columns.indexOf("seq")
    const fresh = seqIdx < 0 ? rows : rows.filter((r) => toNumber(r[seqIdx]) > this.seq)
    if (!fresh.length) return false
    this.rows.push(...fresh)
    this.seq = Math.max(this.seq, seq)
    const tIdx = this.columns.indexOf("t")
    const lastT = toNumber(this.rows[this.rows.length - 1][tIdx])
    let drop = 0
    while (drop < this.rows.length && toNumber(this.rows[drop][tIdx]) < lastT - this.keepSeconds) drop++
    if (drop) this.rows.splice(0, drop)
    return true
  }

  // Last `seconds` of data, with t relative to the newest sample (so the x axis reads -30 s .. 0 s).
  view(seconds = LIVE_VIEW_SECONDS) {
    if (!this.columns || !this.rows.length) return null
    const s = toSeries(this.columns, this.rows)
    const lastT = s.t[s.t.length - 1]
    const start = s.t.findIndex((t) => t >= lastT - seconds)
    const out = {}
    for (const [k, arr] of Object.entries(s)) out[k] = arr.slice(start)
    out.t = out.t.map((t) => t - lastT)
    return out
  }

  latest() {
    if (!this.columns || !this.rows.length) return null
    const row = this.rows[this.rows.length - 1]
    const out = {}
    this.columns.forEach((name, idx) => { out[name] = toNumber(row[idx]) })
    return out
  }
}

function niceHalfSpan(maxAbs, floor) {
  const raw = Math.max(floor, maxAbs * 1.15)
  const steps = [0.1, 0.2, 0.25, 0.5, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10, 15, 20, 25, 30, 40, 50]
  return steps.find((s) => s >= raw) || Math.ceil(raw)
}

// Build SVG geometry for one chart.
//   t: x values (seconds). lines: [{key, values, color, cls, label}]. inactive: per-sample 0..1 (1 = in control)
//   used to shade where openpilot was not in control. opts: {floor, zeroBased, tMin, tMax, unit}
export function buildChart(t, lines, inactive, opts = {}) {
  const W = CHART_W
  const H = CHART_H
  const pad = 6
  const n = Array.isArray(t) ? t.length : 0
  if (n < 2) return { empty: true, width: W, height: H }
  const tMin = opts.tMin ?? t[0]
  const tMax = opts.tMax ?? t[n - 1]
  const tSpan = Math.max(1e-6, tMax - tMin)
  let maxAbs = 0
  let maxVal = 0
  for (const line of lines) {
    for (const v of line.values) {
      const x = toNumber(v)
      if (Math.abs(x) > maxAbs) maxAbs = Math.abs(x)
      if (x > maxVal) maxVal = x
    }
  }
  const floor = opts.floor ?? 1
  const zeroBased = !!opts.zeroBased
  const max = zeroBased ? niceHalfSpan(maxVal, floor) : niceHalfSpan(maxAbs, floor)
  const min = zeroBased ? 0 : -max
  const ySpan = max - min
  const xFor = (x) => ((x - tMin) / tSpan) * W
  const yFor = (v) => pad + ((max - Math.max(min, Math.min(max, toNumber(v)))) / ySpan) * (H - 2 * pad)

  // Down-sample to ~2 points per horizontal unit so long windows stay light.
  const stride = Math.max(1, Math.floor(n / (W * 2)))
  const paths = lines.map((line) => {
    let d = ""
    for (let i = 0; i < n; i += stride) {
      d += `${d ? "L" : "M"}${xFor(t[i]).toFixed(1)},${yFor(line.values[i]).toFixed(1)}`
    }
    return { key: line.key, cls: line.cls || line.key, color: line.color, d }
  })

  // Shade contiguous runs where openpilot was not in control (activity < 0.5), and gaps in the data.
  const shade = []
  if (Array.isArray(inactive)) {
    let runStart = null
    for (let i = 0; i <= n; i++) {
      const off = i < n && toNumber(inactive[i]) < 0.5
      if (off && runStart === null) runStart = i
      if (!off && runStart !== null) {
        const x0 = xFor(t[Math.max(0, runStart - 1)])
        const x1 = xFor(t[Math.min(n - 1, i)])
        shade.push({ x: x0.toFixed(1), w: Math.max(1, x1 - x0).toFixed(1) })
        runStart = null
      }
    }
  }

  const grid = (zeroBased ? [max, max / 2, 0] : [max, max / 2, 0, -max / 2, -max]).map((v) => ({
    y: yFor(v).toFixed(1),
    zero: v === 0,
    label: fmtNum(v, ySpan < 1 ? 2 : 1),
    pct: ((yFor(v) / H) * 100).toFixed(2),
  }))

  const xTicks = []
  const tickStep = [1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600].find((s) => tSpan / s <= 6) || 7200
  for (let x = Math.ceil(tMin / tickStep) * tickStep; x <= tMax; x += tickStep) {
    xTicks.push({ x: xFor(x).toFixed(1), pct: ((xFor(x) / W) * 100).toFixed(2), label: fmtTick(x, tSpan) })
  }
  // Single-path forms (fixed element count) for renderers that cannot loop inside an <svg>.
  const shadeD = shade.map((s) => `M${s.x},0h${s.w}v${H}h-${s.w}Z`).join("")
  const gridD = grid.filter((l) => !l.zero).map((l) => `M0,${l.y}H${W}`).join("") + xTicks.map((x) => `M${x.x},0V${H}`).join("")
  const zeroD = grid.filter((l) => l.zero).map((l) => `M0,${l.y}H${W}`).join("")
  const slots = [0, 1, 2, 3].map((i) => paths[i] || { key: `empty${i}`, cls: "", color: "none", d: "" })
  return { empty: false, width: W, height: H, paths, slots, shade, shadeD, grid, gridD, zeroD, xTicks, min, max, tMin, tMax,
           unit: opts.unit || "" }
}

function fmtTick(x, span) {
  if (x <= 0 && span <= 120) return `${Math.round(x)}s`
  if (span <= 180) return `${Math.round(x)}s`
  const m = Math.floor(Math.abs(x) / 60)
  const s = Math.round(Math.abs(x) % 60)
  return `${x < 0 ? "-" : ""}${m}:${String(s).padStart(2, "0")}`
}

// Map a click on a chart's SVG to a time value on its x axis.
export function timeAtClick(event, chart) {
  const el = event.currentTarget
  if (!el || !chart || chart.empty) return null
  const rect = el.getBoundingClientRect()
  if (!rect.width) return null
  const frac = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width))
  return chart.tMin + frac * (chart.tMax - chart.tMin)
}

const LAT_LINES = (s) => [
  { key: "lat_des", values: s.lat_des, color: LINE_COLORS.desired, cls: "desired", label: "Requested" },
  { key: "lat_act", values: s.lat_act, color: LINE_COLORS.actual, cls: "actual", label: "Measured" },
]
const LONG_LINES = (s) => [
  { key: "long_des", values: s.long_des, color: LINE_COLORS.desired, cls: "desired", label: "Requested" },
  { key: "long_act", values: s.long_act, color: LINE_COLORS.actual, cls: "actual", label: "Measured" },
]

// Charts for a column-major series (live view or a zoomed window of a saved drive).
export function buildTrackingCharts(s, { advanced = false, tMin, tMax } = {}) {
  if (!s || !s.t || s.t.length < 2) return []
  const charts = [
    { id: "lat", title: "Lateral acceleration", unit: "m/s²", lines: LAT_LINES(s), active: s.lat_active, floor: 0.5 },
    { id: "long", title: "Longitudinal acceleration", unit: "m/s²", lines: LONG_LINES(s), active: s.long_active, floor: 0.5 },
    { id: "v", title: "Speed", unit: "m/s", lines: [{ key: "v", values: s.v, color: LINE_COLORS.speed, cls: "speed", label: "Speed" }],
      active: s.enabled, floor: 5, zeroBased: true },
  ]
  if (advanced) {
    charts.push(
      { id: "latTerms", title: "Steering controller terms", unit: "", active: s.lat_active, floor: 0.1, lines: [
        { key: "lat_p", values: s.lat_p, color: LINE_COLORS.p, cls: "p", label: "P" },
        { key: "lat_i", values: s.lat_i, color: LINE_COLORS.i, cls: "i", label: "I" },
        { key: "lat_d", values: s.lat_d, color: LINE_COLORS.d, cls: "d", label: "D" },
        { key: "lat_f", values: s.lat_f, color: LINE_COLORS.f, cls: "f", label: "F" },
      ] },
      { id: "longTerms", title: "Longitudinal controller terms", unit: "m/s²", active: s.long_active, floor: 0.1, lines: [
        { key: "long_up", values: s.long_up, color: LINE_COLORS.p, cls: "p", label: "P" },
        { key: "long_ui", values: s.long_ui, color: LINE_COLORS.i, cls: "i", label: "I" },
        { key: "long_uf", values: s.long_uf, color: LINE_COLORS.f, cls: "f", label: "FF" },
      ] },
    )
  }
  return charts.map((c) => ({
    ...c,
    legend: c.lines.map((l) => ({ label: l.label, cls: l.cls, color: l.color, value: fmtNum(l.values[l.values.length - 1], 2) })),
    geo: buildChart(s.t, c.lines, c.active, { floor: c.floor, zeroBased: c.zeroBased, unit: c.unit, tMin, tMax }),
  }))
}

// Whole-drive overview charts from analysis.overview (bucket averages).
export function buildOverviewCharts(ov) {
  if (!ov || !Array.isArray(ov.t) || ov.t.length < 2) return []
  const charts = [
    { id: "lat", title: "Lateral acceleration (whole drive)", unit: "m/s²", lines: LAT_LINES(ov), active: ov.lat_active, floor: 0.5 },
    { id: "long", title: "Longitudinal acceleration (whole drive)", unit: "m/s²", lines: LONG_LINES(ov), active: ov.long_active, floor: 0.5 },
    { id: "v", title: "Speed", unit: "m/s", lines: [{ key: "v", values: ov.v, color: LINE_COLORS.speed, cls: "speed", label: "Speed" }],
      active: null, floor: 5, zeroBased: true },
  ]
  return charts.map((c) => ({
    ...c,
    legend: c.lines.map((l) => ({ label: l.label, cls: l.cls, color: l.color, value: "" })),
    geo: buildChart(ov.t, c.lines, c.active, { floor: c.floor, zeroBased: c.zeroBased, unit: c.unit }),
  }))
}

// Key numbers for one axis of an analysis result, in display order.
export function keyNumbers(axis, m) {
  if (!m) return []
  const rows = [["Engaged time", m.engaged_s == null ? "—" : fmtDuration(m.engaged_s)]]
  if (m.status === "ok") {
    rows.push(["Typical error (RMS)", `${fmtNum(m.rmse)} m/s²`])
    rows.push(["95% of samples within", `${fmtNum(m.p95_abs_error)} m/s²`])
    rows.push(["Response lag", `${fmtNum(m.lag_s)} s`])
  }
  if (axis === "lateral") {
    if (m.status === "ok") {
      rows.push(["Curve response", m.curve_gain == null ? "no sustained curves" : `${Math.round(m.curve_gain * 100)}%`])
      rows.push(["Oscillation ratio", fmtNum(m.wobble_ratio)])
      rows.push(["Straight-road bias", m.straight_bias == null ? "—" : `${fmtNum(m.straight_bias)} m/s²`])
    }
    rows.push(["Steering takeovers", String(m.steer_overrides ?? 0)])
  } else {
    if (m.status === "ok") {
      rows.push(["Overall response", m.gain == null ? "—" : `${Math.round(m.gain * 100)}%`])
      rows.push(["Braking bias", m.brake_bias == null ? "—" : `${fmtNum(m.brake_bias)} m/s²`])
      rows.push(["Acceleration bias", m.accel_bias == null ? "—" : `${fmtNum(m.accel_bias)} m/s²`])
      rows.push(["Jerk (95th pct)", m.jerk_p95 == null ? "—" : `${fmtNum(m.jerk_p95)} m/s³`])
    }
    rows.push(["Hard brakes", String(m.hard_brakes ?? 0)])
    rows.push(["Gas presses", String(m.gas_overrides ?? 0)])
  }
  return rows.map(([label, value]) => ({ label, value }))
}

export function statusLabel(status) {
  return { recording: "Recording", analyzing: "Analyzing…", done: "Saved", error: "Analysis failed" }[status] || status || "?"
}

export function sessionUrl(id, suffix = "") {
  return `/api/plots/sessions/${encodeURIComponent(id)}${suffix}`
}

export const HELP_TEXT = "'Requested' is what openpilot asked for, 'Measured' is what the car did. Shaded areas are where openpilot " +
  "was not in control (disengaged, you were steering or pressing the gas); they are left out of the analysis. " +
  "Lateral 'measured' is computed from the steering angle through the vehicle model, so a wrong steer ratio shows up as a gain error."
