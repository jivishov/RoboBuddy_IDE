import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: './tests',
  testMatch: /browser-smoke\.spec\.mjs/,
  timeout: 120_000,
  expect: { timeout: 20_000 },
  use: {
    baseURL: 'http://127.0.0.1:4173',
    headless: true,
    viewport: { width: 1366, height: 768 },
  },
  webServer: {
    command: 'python3 -m http.server 4173 --bind 127.0.0.1',
    url: 'http://127.0.0.1:4173/index.html',
    reuseExistingServer: false,
    timeout: 20_000,
  },
});
