import {defineConfig} from '@playwright/test';
const python = process.env.HARDWARE_TEST_PYTHON || (process.platform === 'win32' ? 'python' : 'python3');
export default defineConfig({
  testDir: './tests/hardware', testMatch: 'panel.spec.mjs', workers: 1, timeout: 45000,
  expect: {timeout: 10000}, use: {baseURL:'http://127.0.0.1:8766',headless:true,viewport:{width:1366,height:900},screenshot:'only-on-failure'},
  webServer:{command:python+' -m hardware_bridge.server --demo --port 8766',url:'http://127.0.0.1:8766/api/health',reuseExistingServer:false,timeout:20000}
});

