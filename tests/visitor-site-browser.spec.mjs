import { test, expect } from '@playwright/test';

test('visitor entry is lightweight, accurately scoped, and exposes both guides', async ({ page }, info) => {
  const requests=[],errors=[];page.on('request',r=>requests.push(r.url()));page.on('pageerror',e=>errors.push(String(e)));
  await page.goto('/');
  await expect(page).toHaveTitle('RoboBuddy IDE — Build, program, observe');
  await expect(page.getByRole('heading',{level:1})).toContainText('Build a workcell.');
  await expect(page.locator('.robot')).toHaveCount(6);
  expect(await page.locator('.preview img').evaluate(i=>i.complete&&i.naturalWidth>0)).toBe(true);
  await page.locator('.help-nav summary').click();
  await expect(page.locator('.help-links')).toBeVisible();
  await page.locator('.help-links a').filter({hasText:'OpenArm manual'}).click();
  await expect(page).toHaveURL(/\/guides\/openarm.html$/);
  expect(requests.some(u=>/\.wasm|pyodide|codemirror|app-v2\.js|modelContext/i.test(u))).toBe(false);
  expect(requests.every(u=>new URL(u).origin==='http://127.0.0.1:4173')).toBe(true);
  expect(errors).toEqual([]);
  await page.goto('/');await page.screenshot({path:info.outputPath('visitor-desktop.png'),fullPage:true});
});

test('manual content, copy, printing, navigation and files work', async ({ page, context }, info) => {
  await context.grantPermissions(['clipboard-read','clipboard-write']);
  await page.goto('/guides/webmcp.html');
  await expect(page.getByRole('heading',{level:1})).toHaveText('WebMCP manual');
  await page.locator('.copy').first().click();
  await expect(page.locator('.copy').first()).toHaveText('Copied');
  expect(JSON.parse(await page.evaluate(()=>navigator.clipboard.readText()))).toEqual({view:'schema'});
  await page.locator('.toc a[href="#example"]').click();
  await expect(page).toHaveURL(/#example$/);
  const [download] = await Promise.all([page.waitForEvent('download'),page.locator('#example a[download]').first().click()]);
  expect(download.suggestedFilename()).toBe('adapter-scene.json');
  await page.evaluate(()=>{window.print=()=>{window.printCalled=true;};});
  await page.locator('[data-print]').click();expect(await page.evaluate(()=>window.printCalled)).toBe(true);
  await page.emulateMedia({media:'print'});
  await expect(page.locator('.toc')).not.toBeVisible();await expect(page.locator('main')).toBeVisible();
  await page.emulateMedia({media:'screen'});
  await page.goto('/guides/openarm.html');await page.screenshot({path:info.outputPath('openarm-manual-desktop.png'),fullPage:true});
});

for (const path of ['/', '/guides/openarm.html', '/guides/webmcp.html']) test(`mobile layout and readable navigation: ${path}`, async ({page},info)=>{
  await page.setViewportSize({width:390,height:844});await page.goto(path);
  expect(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1)).toBe(true);
  await expect(page.getByRole('heading',{level:1})).toBeVisible();
  await page.locator('.help-nav summary').click();await expect(page.locator('.help-links')).toBeVisible();
  await page.keyboard.press('Escape');await expect(page.locator('.help-links')).not.toBeVisible();
  await page.screenshot({path:info.outputPath('mobile.png'),fullPage:true});
});

test('overview and manual are readable with JavaScript disabled', async ({browser})=>{
  const context=await browser.newContext({javaScriptEnabled:false});const page=await context.newPage();
  await page.goto('http://127.0.0.1:4173/');await expect(page.getByRole('heading',{level:1})).toBeVisible();
  await page.locator('.help-nav summary').click();await page.locator('.help-links a').filter({hasText:'WebMCP manual'}).click();
  await expect(page.getByRole('heading',{level:1})).toHaveText('WebMCP manual');await expect(page.locator('#example')).toContainText('TaskSpec JSON');
  await context.close();
});

test('legacy explicit URLs preserve their same-origin workspace destination', async ({page})=>{
  // Test routing only, not a mocked simulation. Block IDE runtime fetches for this navigation-only case.
  await page.route('**/src/app-v2.js',r=>r.fulfill({contentType:'text/javascript',body:''}));
  await page.route('https://**',r=>r.abort());
  for(const suffix of ['?ci=visitor-route','?view=ide','?robot=openarm','#ide']){
    await page.goto('/'+suffix,{waitUntil:'domcontentloaded'});
    await expect(page).toHaveURL(new RegExp('/ide\\.html'+(suffix==='#ide'?'$':suffix.replace(/[?]/g,'\\?')+'$')));
  }
  await page.goto('/?utm_source=example');await expect(page.getByRole('heading',{level:1})).toBeVisible();
});
