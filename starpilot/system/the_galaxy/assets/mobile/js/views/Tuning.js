import { LateralTuningPanel } from "../components/LateralTuningPanel.js"
import { NrdrLatTunePanel } from "../components/NrdrLatTunePanel.js"
import { GalaxyTabs } from "../components/GalaxyTabs.js"
import { Plots } from "./Plots.js"
import { TestingGround } from "./TestingGround.js"
import { useTabRouting } from "../composables.js"

const TABS = {
  lateral: "Lateral Tuning",
  nrdr: "NRDR PID lateral tune",
  plots: "Plots",
  testing: "Testing Ground",
}

export const Tuning = {
  name: "Tuning",
  components: { LateralTuningPanel, NrdrLatTunePanel, Plots, TestingGround, GalaxyTabs },
  setup() {
    return useTabRouting("/tuning", { lateral: "lateral", nrdr: "nrdr-pid", plots: "plots", testing: "testing" })
  },
  data() { return { TABS } },
  template: `
    <div class="gx-view">
      <h2 style="margin-top:0;">Tuning, Plots & Testing</h2>

      <GalaxyTabs :items="TABS" :active="tab" @select="selectTab" />

      <template v-if="tab === 'lateral'">
        <LateralTuningPanel />
      </template>

      <template v-else-if="tab === 'nrdr'">
        <NrdrLatTunePanel />
      </template>

      <template v-else-if="tab === 'plots'">
        <Plots :embedded="true" />
      </template>

      <template v-else>
        <TestingGround :embedded="true" />
      </template>
    </div>
  `,
}
