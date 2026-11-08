import { GxNotice } from "./notice.js"
import { reportSave } from "./save-feedback.js"

// Presets in minutes; the browser converts every choice to an epoch second before sending it,
// so the phone's own local time zone (and clock) decides when the pause actually ends.
export const PRESET_MINUTES = [15, 30, 60, 120, 240, 480]

export function pausedLabel(pausedUntil) {
  if (!Number.isSafeInteger(pausedUntil)) return ""
  return new Date(pausedUntil * 1000).toLocaleString(undefined, { weekday: "short", hour: "numeric", minute: "2-digit" })
}

export class SentryPauseClient {
  constructor({ publish, unauthorized, fetcher = (...args) => fetch(...args) }) {
    this.publish = publish; this.unauthorized = unauthorized; this.fetcher = fetcher
    this.busy = false
  }
  async run(payload = null) {
    this.busy = true
    this.publish({ pauseBusy: true, pauseError: "" })
    try {
      const response = await this.fetcher("./api/sentry/pause", { credentials: "same-origin", cache: "no-store",
        ...(payload === null ? {} : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) }) })
      if (response.status === 401) { this.unauthorized(); return false }
      const value = await response.json()
      if (!response.ok) throw new Error(value?.error || "Sentry pause request failed.")
      if (!(value && (value.pausedUntil === null || Number.isSafeInteger(value.pausedUntil)))) throw new Error("Sentry pause status is unavailable.")
      this.publish({ pausedUntil: value.pausedUntil, pauseError: "" })
      return true
    } catch (error) {
      this.publish({ pauseError: error.message || "Sentry pause request failed." })
      return false
    } finally {
      this.busy = false
      this.publish({ pauseBusy: false })
    }
  }
}

export const SentryPause = {
  components: { GxNotice },
  name: "SentryPause",
  props: { mode: { type: String, required: true }, unauthorized: { type: Function, required: true } },
  data: () => ({ pausedUntil: null, pauseBusy: false, pauseError: "", customAmount: "30", customUnit: "minutes", customUntil: "" }),
  created() { this.client = new SentryPauseClient({ publish: update => Object.assign(this.$data, update), unauthorized: this.unauthorized }) },
  mounted() { if (this.mode === "local") this.client.run() },
  methods: {
    reportSave,
    pausedLabel,
    presetLabel(minutes) { return minutes < 60 ? `${minutes} min` : `${minutes / 60} h` },
    pauseFor(minutes) { return reportSave(() => this.client.run({ until: Math.floor(Date.now() / 1000) + minutes * 60 })) },
    pauseCustom() {
      const amount = Number(this.customAmount)
      if (!Number.isFinite(amount) || amount <= 0) { this.pauseError = "Enter a pause amount greater than zero."; return Promise.resolve(false) }
      const seconds = Math.round(amount * (this.customUnit === "hours" ? 3600 : 60))
      return reportSave(() => this.client.run({ until: Math.floor(Date.now() / 1000) + seconds }))
    },
    pauseUntil() {
      const until = this.customUntil ? Math.floor(new Date(this.customUntil).getTime() / 1000) : NaN
      if (!Number.isFinite(until)) { this.pauseError = "Choose a valid date and time."; return Promise.resolve(false) }
      return reportSave(() => this.client.run({ until }))
    },
    resumeNow() { return reportSave(() => this.client.run({ action: "clear" })) },
  },
  template: `
    <section class="gx-card" aria-label="Sentry pause" :inert="pauseBusy" :aria-busy="pauseBusy || undefined">
      <div class="gx-section__header"><i class="bi bi-pause-circle" aria-hidden="true"></i><span class="gx-section__title">Pause</span></div>
      <div class="gx-panel gx-stack">
        <p v-if="mode !== 'local'">Pausing is available on your connected device.</p>
        <template v-else>
          <GxNotice tone="danger" v-if="pauseError">{{ pauseError }}</GxNotice>
          <template v-if="pausedUntil">
            <p class="gx-note">Paused until {{ pausedLabel(pausedUntil) }}</p>
            <button type="button" class="gx-btn gx-btn--tonal" :disabled="pauseBusy" @click="resumeNow">Resume now</button>
          </template>
          <template v-else>
            <p class="gx-note">Pausing stops monitoring, arming and notifications; it resumes by itself and does not change the saved on/off setting.</p>
            <div class="gx-actions">
              <button v-for="minutes in [15, 30, 60, 120, 240, 480]" :key="minutes" type="button" class="gx-btn gx-btn--tonal" :disabled="pauseBusy"
                @click="pauseFor(minutes)">{{ presetLabel(minutes) }}</button>
            </div>
            <div class="gx-row gx-row--borderless">
              <label>Custom <input v-model="customAmount" class="gx-field" type="number" min="1" step="1" inputmode="numeric"></label>
              <select v-model="customUnit" class="gx-field"><option value="minutes">minutes</option><option value="hours">hours</option></select>
              <button type="button" class="gx-btn gx-btn--tonal" :disabled="pauseBusy" @click="pauseCustom">Pause</button>
            </div>
            <div class="gx-row gx-row--borderless">
              <label>Until <input v-model="customUntil" class="gx-field" type="datetime-local"></label>
              <button type="button" class="gx-btn gx-btn--tonal" :disabled="pauseBusy" @click="pauseUntil">Pause</button>
            </div>
          </template>
        </template>
      </div>
    </section>`,
}
