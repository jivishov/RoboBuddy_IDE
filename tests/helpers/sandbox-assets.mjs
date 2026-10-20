// Optional exact-byte asset transport for sandbox runners whose Chromium cannot
// use the environment's egress proxy. Does not alter app code or replace models.
import {execFile} from 'node:child_process';
import {promisify} from 'node:util';
import {createHash} from 'node:crypto';
import {mkdir,readFile,writeFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
const run=promisify(execFile), pending=new Map();
export async function sandboxAssets(page) {
  if(process.env.ROBOBUDDY_TEST_CURL_PROXY!=='1') return;
  const cache=join(tmpdir(),'robobuddy-test-assets');await mkdir(cache,{recursive:true});
  await page.context().route(/^https:\/\/(cdn\.jsdelivr\.net|storage\.googleapis\.com)\//,async route=>{
    const url=route.request().url();
    if(!pending.has(url)) pending.set(url,(async()=>{
      const path=join(cache,createHash('sha256').update(url).digest('hex'));
      try{return await readFile(path);}catch{}
      const {stdout}=await run('curl',['--fail','--location','--silent','--show-error','--max-time','45',url],{encoding:'buffer',maxBuffer:80*1024*1024});
      await writeFile(path,stdout);return stdout;
    })());
    try {
      const pathname=new URL(url).pathname;
      const type=/\.(m?js)$/.test(pathname)?'text/javascript':pathname.endsWith('.wasm')?'application/wasm':pathname.endsWith('.css')?'text/css':'application/octet-stream';
      await route.fulfill({status:200,body:await pending.get(url),contentType:type,headers:{'Access-Control-Allow-Origin':'*'}});
    } catch {pending.delete(url);await route.abort('failed');}
  });
}
