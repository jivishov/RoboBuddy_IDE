import { createHash } from 'node:crypto';
import { readFile, readdir } from 'node:fs/promises';
import { resolve, relative } from 'node:path';
import { decodeMicroDuckBakedVisual } from '../src/microduck/baked-visual.js';

const root = resolve('assets/microduck');
const manifest = JSON.parse(await readFile(resolve(root, 'manifest.json'), 'utf8'));
if (manifest.pins.microduck !== '590b986bd8c0d50ae02cb3ea2f59c463b6828168') throw new Error('wrong MicroDuck source pin');
if (manifest.pins.mujoco !== '3.11.0' || manifest.pins.onnxruntimeWeb !== '1.27.0') throw new Error('wrong browser runtime pins');
if (manifest.policyContract?.input?.join('x') !== '1x61' || manifest.policyContract?.output?.join('x') !== '1x14') throw new Error('wrong policy shape contract');
if (!/no RL, MuJoCo, dynamics, or hardware parity/.test(manifest.policyContract.claim)) throw new Error('missing bounded inference claim');

async function walk(directory) {
  const result = [];
  for (const item of await readdir(directory, { withFileTypes: true })) {
    const path = resolve(directory, item.name);
    if (item.isDirectory()) result.push(...await walk(path)); else result.push(path);
  }
  return result;
}
const actual = (await walk(root)).filter((path) => !path.endsWith('manifest.json')).map((path) => relative(root, path).replaceAll('\\', '/')).sort();
const recorded = manifest.entries.map((entry) => entry.path).sort();
if (JSON.stringify(actual) !== JSON.stringify(recorded)) throw new Error(`manifest inventory mismatch\nactual=${actual}\nrecorded=${recorded}`);
for (const entry of manifest.entries) {
  const bytes = await readFile(resolve(root, entry.path));
  const hash = createHash('sha256').update(bytes).digest('hex');
  if (bytes.length !== entry.bytes || hash !== entry.sha256) throw new Error(`${entry.path}: size/hash mismatch`);
  if (bytes.subarray(0, 64).toString('utf8').startsWith('version https://git-lfs.github.com/spec')) throw new Error(`${entry.path}: Git LFS pointer`);
  if (entry.path.endsWith('.wasm') && bytes.subarray(0, 4).toString('hex') !== '0061736d') throw new Error(`${entry.path}: wrong WebAssembly magic`);
  if (entry.path.endsWith('.mid') && bytes.subarray(0, 4).toString('ascii') !== 'MThd') throw new Error(`${entry.path}: wrong MIDI magic`);
  if (entry.path.endsWith('.bin') && bytes.subarray(0, 4).toString('ascii') !== 'DUCK') throw new Error(`${entry.path}: wrong MicroDuck baked-visual magic`);
  if (entry.path.endsWith('.onnx')) {
    const prefix = bytes.subarray(0, 64).toString('utf8').trimStart().toLowerCase();
    if (bytes.length < 10000 || prefix.startsWith('<!doctype') || prefix.startsWith('<html') || prefix.startsWith('{')) throw new Error(`${entry.path}: truncated or text response instead of ONNX protobuf`);
  }
  if (/runtime\/(mujoco|onnx)\/.*\.(js|mjs)$/.test(entry.path) && bytes.length < 10000) throw new Error(`${entry.path}: truncated runtime module`);
  for (const field of ['license','repository','revision','sourcePath','status','purpose']) if (!entry[field]) throw new Error(`${entry.path}: missing ${field}`);
}
const policies = manifest.entries.filter((entry) => entry.path.endsWith('.onnx'));
if (policies.length !== 9) throw new Error(`expected 9 ONNX policies, found ${policies.length}`);
const rig = JSON.parse(await readFile(resolve(root, 'generated/procedural-rig.json'), 'utf8'));
if (rig.joints.length !== 14) throw new Error('procedural rig does not contain 14 policy joints');
const expectedPolicyOrder = ['left_hip_yaw','left_hip_roll','left_hip_pitch','left_knee','left_ankle','neck_pitch','head_pitch','head_yaw','head_roll','right_hip_yaw','right_hip_roll','right_hip_pitch','right_knee','right_ankle'];
if (JSON.stringify(rig.joints.map((joint) => joint.name)) !== JSON.stringify(expectedPolicyOrder)) throw new Error('source XML joint order drifted from the policy action order');
if (JSON.stringify(rig.jointContract?.policyJointOrder) !== JSON.stringify(expectedPolicyOrder) || rig.jointContract?.mouthWireIndex !== 9) throw new Error('explicit policy/wire joint mapping is missing or wrong');
if (rig.jointContract?.angleUnit !== 'radians' || !/right-hand/.test(rig.jointContract?.positiveDirection || '')) throw new Error('joint unit/sign convention is missing');
if (rig.bodies.some((body) => !body.inertial)) throw new Error('procedural rig is missing extracted body inertials');
for (const frame of ['head_camera','tof','imu','head_imu','mouth_tip','left_foot','right_foot']) if (!rig.sites.some((site) => site.name === frame)) throw new Error(`missing named frame ${frame}`);
if (!rig.fidelity.configuredApproximation.includes('configured articulation of official lower-bill part') || !rig.fidelity.excludedEvidence.includes('microduck_rl MJCF')) throw new Error('rig fidelity boundary missing');
if (rig.primitives.length !== 0) throw new Error('obsolete procedural shell primitives remain in the rig contract');
if (JSON.stringify(rig.configuredAttachments?.mouth?.officialPartIndices) !== '[40]' || rig.configuredAttachments.mouth.closedRad !== 0 || rig.configuredAttachments.mouth.openRad > 0.1) throw new Error('official lower-bill articulation mapping is missing or visually unbounded');
const visualEntry = manifest.entries.find((entry) => entry.path === 'visual/duck.bin');
if (!visualEntry || visualEntry.license !== 'Apache-2.0' || visualEntry.repository !== 'pollen-robotics/microduck' || visualEntry.revision !== manifest.pins.microduck || visualEntry.sourcePath !== 'robotctl/assets/duck.bin') throw new Error('official visual provenance is incomplete');
const visualBytes = await readFile(resolve(root, 'visual/duck.bin'));
const visualBuffer = visualBytes.buffer.slice(visualBytes.byteOffset, visualBytes.byteOffset + visualBytes.byteLength);
const visual = decodeMicroDuckBakedVisual(visualBuffer);
if (visual.format !== 'DUCK v1' || visual.meshes.length !== 28 || visual.bodies.length !== 15 || visual.parts.length !== 58) throw new Error('official visual structure drifted');
if (manifest.visualContract?.meshes !== visual.meshes.length || manifest.visualContract?.bodies !== visual.bodies.length || manifest.visualContract?.parts !== visual.parts.length) throw new Error('official visual manifest contract drifted');
const close = (a, b) => Math.abs(Number(a) - Number(b)) <= 1e-6;
visual.bodies.forEach((body, index) => {
  const expectedParent = body.parentIndex < 0 ? null : rig.bodies[body.parentIndex]?.name;
  if (rig.bodies[index]?.parent !== expectedParent) throw new Error(`official visual body ${index} parent mismatch`);
  if (!body.positionM.every((value, axis) => close(value, rig.bodies[index].pos[axis])) || !body.quaternionWxyz.every((value, axis) => close(value, rig.bodies[index].quatWxyz[axis]))) throw new Error(`official visual body ${index} transform mismatch`);
  if (body.jointWireIndex >= 0 && !rig.joints.some((joint) => joint.name === rig.jointContract.wireJointOrder[body.jointWireIndex])) throw new Error(`official visual body ${index} joint mismatch`);
});
const fixture = JSON.parse(await readFile(resolve(root, 'fixtures/inference-parity.json'), 'utf8'));
if (fixture.input.length !== 61 || Object.values(fixture.policies).some((item) => item.output.length !== 14)) throw new Error('fixture shape mismatch');
if (fixture.command !== 'python scripts/verify_microduck_cpu_fixture.py') throw new Error('fixture does not record its exact CPU verification command');
const scores = manifest.entries.filter((entry) => entry.path.startsWith('scores/'));
if (JSON.stringify(scores.map((entry) => entry.path).sort()) !== JSON.stringify(['scores/duck_strut.mid','scores/wistful.duckscore'])) throw new Error('release-safe score inventory is incomplete');
if (!manifest.scoreContract?.excluded?.some((item) => item.includes('outer_wilds.mid'))) throw new Error('copyrighted upstream test score is not explicitly excluded');
console.log(`MicroDuck asset gate passed: ${manifest.entries.length} files, ${policies.length} policies, 14 joints, ${visual.meshes.length} official visual meshes/${visual.parts.length} parts, ${scores.length} release-safe scores, 2 fixed inference outputs.`);
