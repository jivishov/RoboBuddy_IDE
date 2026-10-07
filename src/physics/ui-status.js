import { capabilityLabel, physicsCapabilityFor } from './capabilities.js';

const PHYSICAL_MODE_CHIP = Object.freeze({
  asimov: 'ASIMOV 1 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  openarm: 'OPENARM V2 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  lekiwi: 'LEKIWI V1 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  so101: 'SO-101 PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
  microduck: 'MICRODUCK ALPHA PHYSICAL WORKSPACE · MUJOCO AUTHORITY',
});

const PHYSICAL_SIM_BADGE = Object.freeze({
  asimov: 'MUJOCO · 23 SOURCE JOINTS · ESTIMATED PD',
  openarm: 'MUJOCO V2 BIMANUAL DRY-STACK · SHARED FREE BODIES · NOT HARDWARE CALIBRATION',
  lekiwi: 'MUJOCO HOLONOMIC BASE · PASSIVE OMNI ROLLERS · FREE PAYLOAD · NOT HARDWARE CALIBRATION',
  so101: 'MUJOCO RIGID-BODY BENCHMARK · ACTUAL CONTACT/GRAVITY · NOT HARDWARE CALIBRATION',
  microduck: 'MUJOCO CONTACT LOCOMOTION · DEPLOYED ONNX · IDENTIFIED XL330 SERVO · NOT HARDWARE CALIBRATION',
});

const PHYSICAL_SIDE_SUMMARY = Object.freeze({
  asimov: 'Asimov 1. Browser MuJoCo owns the 23-joint source model and measured body poses. Free-base WebMCP can execute bounded agent-generated whole-body keyframes, including stepping/walking attempts, with optional explicit simulator-only ankle stabilization. No trained gait or hardware-calibration claim.',
  openarm: 'OpenArm V2. Browser MuJoCo is the single physical authority for both arms and the free task objects; the canonical mesh is presentation-only and hardware validation remains pending.',
  lekiwi: 'LeKiwi V1. Browser MuJoCo is the single physical authority for the free base, driven wheels, mounted arm and free beaker; the canonical mesh follows observed state only and hardware validation remains pending.',
  so101: 'SO-101. Browser MuJoCo is the single physical authority for the arm and free benchmark object; the canonical mesh is presentation-only and hardware validation remains pending.',
  microduck: 'MicroDuck alpha. Browser MuJoCo is the single physical authority for the free trunk, fourteen actuated joints and the free ball; the official runtime visual follows observed state only, the root is never driven, and hardware validation remains pending.',
});

export function applyPhysicsPreviewStatus(profileId, { physical = true } = {}) {
  const capability = physicsCapabilityFor(profileId, { physical });
  const backendBadge = document.getElementById('physicsBackendBadge');
  const modeChip = document.getElementById('modeChip');
  const simBadge = document.getElementById('simBadge');
  const fidelityText = document.getElementById('fidelityText');
  const sideRobotSummary = document.getElementById('sideRobotSummary');
  if (backendBadge) {
    backendBadge.textContent = capabilityLabel(capability);
    backendBadge.title = `${capability.capability} ${capability.limitations.join(' ')}`;
  }
  if (modeChip && capability.backend === 'browser-mujoco' && PHYSICAL_MODE_CHIP[profileId]) {
    modeChip.textContent = PHYSICAL_MODE_CHIP[profileId];
  }
  if (simBadge && capability.backend === 'browser-mujoco' && PHYSICAL_SIM_BADGE[profileId]) {
    simBadge.textContent = PHYSICAL_SIM_BADGE[profileId];
  }
  if (sideRobotSummary && capability.backend === 'browser-mujoco' && PHYSICAL_SIDE_SUMMARY[profileId]) {
    sideRobotSummary.textContent = PHYSICAL_SIDE_SUMMARY[profileId];
  }
  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'asimov') {
    fidelityText.textContent = 'Asimov 1 uses one authoritative browser MuJoCo PhysicsSession. WebMCP may generate finite full-body joint-target trajectories in free-base scenes; movement, support changes and falls come from bounded joint actuation, gravity and contact. The optional 20 Hz ground-truth ankle-target stabilizer is explicit simulator feedback and never writes root pose, root velocity or external force. A completed trajectory is not evidence of a trained or hardware-valid gait; inspect measured displacement, contacts, support transitions and final stability.';
  }
  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'microduck') {
    fidelityText.textContent = 'MicroDuck alpha runs on one authoritative browser MuJoCo PhysicsSession. The deployed ONNX policies choose fourteen joint targets, an identified XL330 position-servo model turns those targets into torque, and locomotion emerges from foot-floor contact: the root is never written, no base velocity is injected and no kick impulse is synthesised. Contact parameters and the servo identification are simulator estimates, the trained BAM voltage actuator and its action delay are not reproduced, and hardware validation remains pending.';
  }
  if (fidelityText && capability.backend === 'browser-mujoco' && profileId === 'openarm') {
    fidelityText.textContent = 'OpenArm V2 uses one authoritative browser MuJoCo PhysicsSession. Both arms, the free flask and beaker, fixtures, live Python, WebMCP and task evaluation consume that state. Only the source mechanical finger couplings remain constrained; task objects are never attached, welded, snapped, teleported or moved by renderer state. Collision proxies and dry workcell parameters are simulator estimates; hardware validation remains pending.';
  }
  return capability;
}
