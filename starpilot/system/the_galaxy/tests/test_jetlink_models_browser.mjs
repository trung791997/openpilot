// The Jetlink panel in the Galaxy model manager, classic and mobile, against a
// synthetic /api/models/jetlink: lists jetlink's catalog, selects by ref and
// back to the default, and locks while onroad. Screenshots go to EVIDENCE.
const {chromium} = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
import {fileURLToPath} from 'node:url';
import {mkdtempSync, mkdirSync, readFileSync} from 'node:fs';
import {tmpdir} from 'node:os';
import assert from 'node:assert/strict';
const root = (process.env.REPO || fileURLToPath(new URL('../../../../', import.meta.url))) + '/starpilot/system/the_galaxy';
const out = process.env.EVIDENCE || mkdtempSync(tmpdir() + '/jetlink-models-browser-');
mkdirSync(out, {recursive: true});
const browser = await chromium.launch({executablePath: process.env.BROWSER_EXECUTABLE || undefined, args: ['--no-sandbox']});
const A = 'bf3e3631b3f91d92a1020a5e0dd4298b93ff4244', B = '2fb4ac4a00000000000000000000000000000000';
try {
  for (const surface of ['classic', 'mobile']) for (const onroad of [false, true]) {
    const mobile = surface === 'mobile';
    const context = await browser.newContext({viewport: {width: mobile ? 390 : 1200, height: 1400}});
    let picked = '';
    const writes = [];
    const jetlink = () => ({available: true, mode: 'usb', enabled: true, present: true, transport: 'USB', ready: picked === A, reason: null,
      progress: {stage: 'download', frac: 0.4, msg: '40%'}, model: 'Cinque Terre V3 Model (September 17, 2026)', defaultModel: 'Cinque Terre V3 Model',
      activeModel: picked === B ? 'ResAction Preview (October 05, 2026)' : 'Cinque Terre V3 Model (September 17, 2026)', isOnroad: onroad,
      models: [{ref: B, name: 'ResAction Preview (October 05, 2026)', state: picked === B ? 'downloaded' : null, selected: picked === B},
               {ref: A, name: 'Cinque Terre V3 Model (September 17, 2026)', state: 'ready', selected: picked === A}]});
    const status = {models: [], currentModel: '', activeBigModel: '', activeSmallModel: '', summary: {installed: 0, missing: 0, total: 0}, downloading: false, progress: '', isOnroad: onroad};
    const mount = mobile ? `import {createApp} from 'vue';import {ModelManager} from '/assets/mobile/js/views/ModelManager.js';createApp(ModelManager).mount('#app');`
      : `import {html} from '/assets/vendor/arrow-core.js';import {ModelManager} from '/assets/components/tools/model_manager.js';window.showSnackbar=()=>{};html\`\${()=>ModelManager()}\`(document.querySelector('#app'));`;
    const page = `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><link rel="stylesheet" href="/assets/vendor/bootstrap-icons/bootstrap-icons.min.css"><link rel="stylesheet" href="${mobile ? '/assets/mobile/css/material.css' : '/assets/components/tools/model_manager.css'}"><script type="importmap">{"imports":{"vue":"/assets/vendor/vue/vue.esm-browser.js"}}</script><style>:root{--text-color:#e9eaf2;--text-muted:#aab0c1;--card-bg:#1b2030;--secondary-bg:#262c3c;--sidebar-border-color:#353e54;--input-bg:#171c29;--success-bg:#81d4b1;--color-black:#10251c;--danger-bg:#883644;--sidebar-active-bg:#384560;--font-size-base:14px;--padding-base:16px;--margin-base:16px;--gap-xs:4px;--gap-sm:8px;--gap-md:12px;--gap-lg:20px;--border-radius-base:8px;--border-radius-lg:12px}body{background:#101420;color:#e9eaf2;font:14px system-ui;margin:16px}*{box-sizing:border-box}</style></head><body><main id="app"></main><script type="module">${mount}</script></body></html>`;
    await context.route('**/*', async route => {
      const req = route.request(), url = new URL(req.url());
      if (url.origin !== 'http://galaxy.invalid') return route.abort();
      if (url.pathname === '/api/models/jetlink') {
        if (req.method() === 'PUT') { writes.push(req.postDataJSON()); picked = writes.at(-1).ref; }
        return route.fulfill({json: jetlink()});
      }
      if (url.pathname.startsWith('/api/')) return route.fulfill({json: status});
      if (url.pathname.startsWith('/assets/')) {
        try { return route.fulfill({body: readFileSync(root + url.pathname), contentType: url.pathname.endsWith('.css') ? 'text/css' : 'text/javascript'}); } catch { return route.abort(); }
      }
      return route.fulfill({contentType: 'text/html', body: page});
    });
    const tab = await context.newPage();
    const errors = [];
    tab.on('pageerror', e => errors.push(String(e)));
    await tab.goto('http://galaxy.invalid/');
    const panel = tab.locator(mobile ? 'section.gx-card' : 'section.mm-jetlink', {hasText: 'Jetlink Big Models'});
    await panel.getByText('ResAction Preview (October 05, 2026)').waitFor();
    await panel.getByText('Running: Cinque Terre V3 Model (September 17, 2026)').waitFor();
    await panel.getByText('Built on host').waitFor();
    const selects = panel.getByRole('button', {name: 'Select'});
    assert.equal(await selects.count(), 2);
    if (onroad) {
      assert.equal(await selects.first().isDisabled(), true);
    } else {
      await selects.first().click();
      await panel.getByRole('button', {name: 'Use Default'}).waitFor();
      assert.deepEqual(writes, [{ref: B}]);
      // a response that changes what is running and downloaded shows without a reload
      await panel.getByText('Running: ResAction Preview (October 05, 2026)').waitFor();
      await panel.getByText(mobile ? /^2fb4ac4a00 · Downloaded$/ : /^Downloaded$/).waitFor();
      await panel.getByRole('button', {name: 'Use Default'}).click();
      await tab.waitForFunction(() => true);
      await panel.getByRole('button', {name: 'Select'}).nth(1).waitFor();
      assert.deepEqual(writes, [{ref: B}, {ref: ''}]);
    }
    await panel.screenshot({path: `${out}/jetlink-${surface}${onroad ? '-onroad' : ''}.png`});
    assert.deepEqual(errors, []);
    await context.close();
    console.log(`${surface}${onroad ? ' onroad' : ''}: ok`);
  }
} finally {
  await browser.close();
}
console.log(`screenshots: ${out}`);
