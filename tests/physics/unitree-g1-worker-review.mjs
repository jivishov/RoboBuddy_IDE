import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { UnitreeG1StandEvaluator } from '../../src/physics/unitree-g1-scene.js';
import { UNITREE_G1_FREEBASE_PACKAGE } from '../../src/physics/unitree-g1-model-package.js';

// Execute the shipped worker and bundled WASM unchanged. Only worker messaging and same-origin
// file delivery are adapted to Node; the Chromium conformance suite exercises browser transport.
const workerUrl = new URL(process.env.G1_REVIEW_WORKER || '../../src/physics/unitree-g1-mujoco-worker.js', import.meta.url);
globalThis.self = { location: workerUrl };
let reply;
globalThis.postMessage = (value) => { reply = value; };
let corruptMesh = false;
globalThis.fetch = async (input) => {
  const url = new URL(input);
  assert.equal(url.protocol, 'file:');
  let bytes = await readFile(fileURLToPath(url));
  if (corruptMesh && url.pathname.endsWith('/head_link.STL')) {
    // Change a binary STL header byte: still a valid mesh file, but not the pinned asset.
    bytes = Buffer.from(bytes); bytes[0] ^= 1;
  }
  return new Response(bytes, { status: 200 });
};
await import(workerUrl.href);
let id = 0;
async function rpc(op, payload) {
  reply = null;
  await self.onmessage({ data: { id: ++id, op, payload } });
  assert.equal(reply.id, id);
  if (!reply.ok) throw new Error(reply.error);
  return reply.payload;
}

await rpc('load', { modelPackage: UNITREE_G1_FREEBASE_PACKAGE });
for (const request of [
  { type: 'set_joint_targets', targetsRad: { left_knee_joint: NaN } },
  { type: 'set_lowlevel_targets', commands: { left_knee_joint: { kp: Infinity } } },
]) {
  await rpc('command', { type: 'engage_stand' });
  const before = await rpc('observe');
  await assert.rejects(rpc('command', request), /finite/);
  const after = await rpc('observe');
  console.log(JSON.stringify({ request: request.type, before: before.controller.id, after: after.controller.id }));
  assert.deepEqual(after.controller, before.controller, 'rejected commands must not disengage standing');
  assert.deepEqual(after.joints, before.joints, 'rejected commands must not change accepted targets or gains');
}
const evaluator = new UnitreeG1StandEvaluator();
const run = await rpc('stepSampled', { count: 4000, sampleEverySteps: 10 });
for (const sample of run.observations) evaluator.observe(sample);
const stand = evaluator.snapshot();
assert.equal(stand.standing, true, JSON.stringify(stand));
assert.ok(run.observations.at(-1).contactClasses.leftFootFloor.some((entry) => entry.normalForceN > 1));
console.log(JSON.stringify({ standing: stand.standing, measured: stand.measured, terminal: stand.terminal }));
await rpc('dispose');
if (!process.env.G1_SKIP_MESH_REVIEW) {
  corruptMesh = true;
  await assert.rejects(rpc('load', { modelPackage: UNITREE_G1_FREEBASE_PACKAGE }), /SHA-256.*head_link|head_link.*SHA-256/);
  await rpc('dispose');
}
console.log('Unitree G1 shipped WASM worker: command atomicity and asset integrity OK');
