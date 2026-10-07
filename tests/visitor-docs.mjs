import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { compileGeneralScene } from '../src/physics/openarm-general-scene.js';
import { validateGeneralTask } from '../src/physics/openarm-general-task.js';
import { validateOpenArmProgram } from '../src/webmcp/openarm-workcell.js';
const root = fileURLToPath(new URL('../', import.meta.url));
const read = p => readFileSync(resolve(root, p), 'utf8');
const parse = p => JSON.parse(read(p));
const pages = ['index.html', 'ide.html', 'guides/openarm.html', 'guides/webmcp.html'];
const ids = new Map(pages.map(p => [p, new Set([...read(p).matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]))]));
for (const file of pages) {
  const content = read(file);
  const allIds = [...content.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
  assert.equal(new Set(allIds).size, allIds.length, `${file}: duplicate IDs`);
  assert.match(content, /<html lang="en"/);
  for (const [, value] of content.matchAll(/(?:href|src)="([^"]+)"/g)) {
    if (/^(https?:|data:|mailto:)/.test(value)) continue;
    const url = new URL(value, `https://test.invalid/RoboBuddy_IDE/${file}`);
    assert.ok(url.pathname.startsWith('/RoboBuddy_IDE/'), `${file}: escaped Pages subpath ${value}`);
    const target = decodeURIComponent(url.pathname.slice('/RoboBuddy_IDE/'.length));
    assert.ok(existsSync(resolve(root, target)), `${file}: missing ${value}`);
    if (url.hash && ids.has(target)) assert.ok(ids.get(target).has(decodeURIComponent(url.hash.slice(1))), `${file}: missing anchor ${value}`);
  }
  for (const tag of content.matchAll(/<a\b[^>]*target="_blank"[^>]*>/g)) assert.match(tag[0], /rel="[^"]*noopener/);
}
assert.doesNotMatch(read('index.html'), /app-v2\.js|pyodide|codemirror|type="importmap"/i, 'Visitor page must not initialize or load IDE runtime');
for (const doc of ['guides/openarm.html','guides/webmcp.html']) {
  assert.match(read(doc), /data-print/);
  assert.match(read(doc), /hardware/i);
  assert.match(read(doc), /not.*(?:reconstruction|photo)/i);
}
for (const guide of ['openarm','webmcp']) assert.match(read('ide.html'), new RegExp(`href="\\./guides/${guide}\\.html" target="_blank" rel="noopener"`));
const scene=parse('guides/examples/adapter-scene.json');
assert.deepEqual(scene, parse('tests/fixtures/general-scenes/novel-adapter.json'), 'Guide demo must match the actual physical fixture');
const compiled=compileGeneralScene(scene);
const task=parse('guides/examples/adapter-task.json');
const observation={openarm:{equipment:compiled.records},bodies:{}};
validateGeneralTask(task,observation);
const program=parse('guides/examples/adapter-program.json');
const workcell={authority:{sceneRevision:program.expected_scene_revision},taskEvaluation:{schemaVersion:'robobuddy.lab.task.v2'},geometryIds:compiled.geoms.map(g=>g.id)};
validateOpenArmProgram(program,workcell,observation);
assert.equal(program.segments.at(-1).wait_for.type,'authored_task_complete');
// Every JSON example on the manual is syntactically valid; markers denote values the visitor must replace.
const decode=s=>s.replace(/&quot;/g,'"').replace(/&#x27;/g,"'").replace(/&lt;/g,'<').replace(/&gt;/g,'>').replace(/&amp;/g,'&');
for (const [, content] of read('guides/webmcp.html').matchAll(/<code class="language-json">([\s\S]*?)<\/code>/g)) JSON.parse(decode(content));
console.log('Visitor documents, Pages-relative links, Help destinations and canonical scene/task/program contracts: OK');
