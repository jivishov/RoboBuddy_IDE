import { defineConfig } from '@playwright/test';

const pythonLauncher = process.platform === 'win32' ? 'py -3' : 'python3';

export default defineConfig({
  testDir: './tests',
  testMatch: /(?:browser-smoke|microduck-policy-trace|webmcp)\.spec\.mjs/,
  timeout: 120_000,
  expect: { timeout: 20_000 },
  use: {
    baseURL: 'http://127.0.0.1:4173',
    headless: true,
    viewport: { width: 1366, height: 768 },
  },
  webServer: {
    command: `${pythonLauncher} -m http.server 4173 --bind 127.0.0.1`,
    url: 'http://127.0.0.1:4173/index.html',
    reuseExistingServer: false,
    timeout: 20_000,
  },
});
