import { defineConfig } from '@playwright/test';

const pythonLauncher = process.platform === 'win32' ? 'py -3' : 'python3';

export default defineConfig({
  testDir: './tests',
  testMatch: /(?:visitor-site-browser|visitor-ide-browser|asimov-physical-browser|asimov-whole-body-browser|browser-smoke|microduck-policy-parity|webmcp|physics-slice-browser|physics-load-failure-browser|so101-physics-browser|so101-live-python-browser|so101-manipulation-browser|so101-ide-physical-browser|openarm-phase5a-browser|openarm-workcell-browser|openarm-general-scene-browser|lekiwi-phase5b-browser|lekiwi-conformance-browser|lekiwi-presentation-browser|microduck-phase5c-browser|microduck-interfaces-browser|microduck-presentation-browser|unitree-g1-conformance-browser|unitree-g1-phase5d-browser|unitree-g1-presentation-browser|webmcp-robot-controls)\.spec\.mjs/,
  workers: process.env.CI ? 1 : undefined,
  timeout: 120_000,
  expect: { timeout: 20_000 },
  use: {
    baseURL: 'http://127.0.0.1:4173',
    headless: true,
    viewport: { width: 1366, height: 768 },
    // Opt-in escape hatch for sandboxes that ship a pre-installed Chromium whose build
    // number differs from the one this pinned @playwright/test expects. Unset in CI, where
    // `npx playwright install` provides the matching build, so CI behaviour is unchanged.
    ...(process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE
      ? { launchOptions: { executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE } }
      : {}),
  },
  webServer: {
    command: `${pythonLauncher} -m http.server 4173 --bind 127.0.0.1`,
    url: 'http://127.0.0.1:4173/index.html',
    reuseExistingServer: false,
    timeout: 20_000,
  },
});
