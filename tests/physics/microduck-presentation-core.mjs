import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { MICRODUCK_MODEL_PACKAGES } from '../../src/physics/microduck-model-package.js';
const rig = JSON.parse(readFileSync(new URL('../../assets/microduck/generated/procedural-rig.json', import.meta.url)));
const names = rig.bodies.map(({ name }) => name);
assert.equal(names.length, 15);
for (const model of MICRODUCK_MODEL_PACKAGES) {
  assert.deepEqual(model.bodies.filter(({ id }) => id !== 'microduck_ball').map(({ id }) => id), names);
  const xml = readFileSync(new URL('../../' + model.asset, import.meta.url), 'utf8');
  assert.deepEqual([...xml.matchAll(/<body name="([^"]+)"/g)].map((m) => m[1]), model.bodies.map(({ id }) => id));
}
const simulator = readFileSync(new URL('../../src/physics/microduck-physical-simulator.js', import.meta.url), 'utf8');
const method = simulator.slice(simulator.indexOf('  #applyPresentation('), simulator.indexOf('  #disposePresentation('));
assert.match(method, /applyPhysicalBodyPoses\(observation\.bodies\)/);
assert.doesNotMatch(method, /rig\.applyState|observation\.joints/);
assert.match(simulator, /includeConfiguredRollers: false/);
console.log('MicroDuck presentation contracts: all 15 source bodies in each plant; no mixed joint FK; no cosmetic rollers');
