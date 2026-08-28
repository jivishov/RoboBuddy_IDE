import fs from 'node:fs';

const root = new URL('../', import.meta.url);
const rig = fs.readFileSync(new URL('src/canonical-rig.js', root), 'utf8');
const sim = fs.readFileSync(new URL('src/simulator.js', root), 'utf8');
const profiles = fs.readFileSync(new URL('src/profiles.js', root), 'utf8');

const revision = '66d18a029a0caeb6a6075e681dbd9ecd6b22affa';
for (const token of [
  revision,
  'robot-mesh-data-openarm-v2.js',
  'robot-mesh-data-so101.js',
  'robot-mesh-data-lekiwi.js',
  "robotId: 'openarm_v2_bimanual'",
  "robotId: 'so101_follower'",
  "robotId: 'lekiwi_sim'",
  'right_finger_inner_mesh',
  'right_finger_outer_mesh',
  'getUint16(index * 2, true)',
]) {
  if (!rig.includes(token)) throw new Error(`canonical rig loader missing ${token}`);
}

for (const token of [
  'offsetMm: [0, -168, 0]',
  '`${side}_joint_${index}.pos`',
  '`${side}_gripper.pos`',
  'state[`arm_${key}.pos`]',
  'state[`${key}.pos`]',
  'this.root.rotation.set(0, -(Number(basePose.yaw) || 0), 0)',
]) {
  if (!rig.includes(token)) throw new Error(`canonical command mapping missing ${token}`);
}

for (const token of [
  'CanonicalRobotRig.load(profileId)',
  "canonicalVisual:'unavailable'",
  "hardwareValidation:'pending'",
]) {
  if (!sim.includes(token)) throw new Error(`simulator missing fail-closed canonical behavior ${token}`);
}

for (const id of ['openarm_v2_bimanual','so101_follower','lekiwi_sim']) {
  if (!profiles.includes(id)) throw new Error(`profile does not declare canonical robot id ${id}`);
}

if (sim.includes('_armVisual(') || sim.includes('segmentMesh(')) {
  throw new Error('procedural placeholder robot geometry remains in simulator');
}

console.log('canonical RoboBuddy visual-source checks: OK');
