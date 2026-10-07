import { EXECUTION_BACKENDS, MODEL_EVIDENCE, capabilityRecord } from './model-registry.js';

// Per-capability status for the Unitree G1 profile.
//
// The whole point of this table is that the profile does NOT get one undifferentiated badge. Each
// row names the backend that produces it, the evidence class that supports it, and what it is not.
// A row may only claim `physical / numerically-verified` when a gated trial establishes it.
const row = ({ id, label, backend, capability, evidence, workspace, limitations }) => Object.freeze({
  id, label, workspace, ...capabilityRecord({ backend, capability, evidence, limitations }),
});

export const UNITREE_G1_CAPABILITY_AUDIT = Object.freeze([
  row({
    id: 'kinematic_pose_inspection',
    label: 'Kinematic pose inspection',
    workspace: 'unitree-g1-kinematic-pose-inspection',
    backend: EXECUTION_BACKENDS.LEGACY,
    capability: 'kinematic / verified',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'Deliberately has no contact plant, gravity, balance, gait or hardware-control claim.',
      'It is a source-pose viewer and is never counted as standing, dynamics or contact evidence.',
    ],
  }),
  row({
    id: 'mounted_joint_dynamics',
    label: 'Mounted low-level joint dynamics',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'physical / verified',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'The pelvis is welded to the world by a declared, visible mount. A gravity-loaded joint settles visibly short of its target at the source hold gains, and that is the correct behaviour, not a defect.',
      'No result from the mounted fixture is balance, standing or locomotion evidence.',
    ],
  }),
  row({
    id: 'free_base_dynamics',
    label: 'Free-base dynamics, gravity and contact',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'physical / verified',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'Rigid-body dynamics with the source collision geometry and a declared flat floor. Surface material, compliance, deformation and terrain are not modelled.',
    ],
  }),
  row({
    id: 'fall_and_contact',
    label: 'Fall and named contact identity',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'physical / verified',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'The robot can genuinely fall and the fallen state persists. Reset establishes a new initial condition; it is never a recovery.',
      'Contacts are identified by named geometry pair. Support is never inferred from a contact count.',
    ],
  }),
  row({
    id: 'standing',
    label: 'Free-base standing (posture hold)',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'physical / verified',
    evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
    limitations: [
      'A bounded joint-space posture hold on a free root with real gravity, real foot contact and no support of any kind. It is not dynamic balance, not perturbation recovery and not locomotion-ready.',
      'The controller keeps Unitree FixStand\'s structure, standing posture, waist and arm gains; its four ankle gains are repository-authored and are not Unitree hardware settings.',
      'Unitree FixStand at its exact source gains is measured NOT to hold this posture free-base, and ships as evidence rather than as a standing capability.',
    ],
  }),
  row({
    id: 'perturbation_recovery',
    label: 'Perturbation recovery',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'unsupported',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'Recovery is not defined, not implemented and not exposed. The standing controller was characterised against declared initial root velocities and holds its envelope up to 0.15 m/s while failing at 0.20 m/s; surviving a disturbance inside a tested range is not a recovery capability.',
    ],
  }),
  row({
    id: 'walking',
    label: 'Walking / locomotion',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'unsupported',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'No walk operation, gait, velocity command or locomotion policy exists in this workspace, and none is exposed through live Python or WebMCP.',
      'Unitree publishes a G1 velocity locomotion policy; it was audited and deliberately not ported, because it is trained against a different actuation contract and would need its own observation-ordering, action-scaling, controller-frequency, stopping and traction-disabled gates before anything could be advertised.',
    ],
  }),
  row({
    id: 'dexterous_hands',
    label: 'Dexterous hand control',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'unsupported',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'The selected variant has fixed rubber hands. They are visual-only in the pinned source and carry no collision geometry, no finger joint and no actuator here.',
    ],
  }),
  row({
    id: 'hardware_calibration',
    label: 'Hardware calibration',
    workspace: 'unitree-g1-physical-dynamics',
    backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
    capability: 'unsupported',
    evidence: MODEL_EVIDENCE.MODEL_DERIVED,
    limitations: [
      'No measurement of an assembled Unitree G1 was used or is claimed anywhere in this work package.',
    ],
  }),
]);

export const UNITREE_G1_PHYSICAL_CAPABILITY = capabilityRecord({
  backend: EXECUTION_BACKENDS.BROWSER_MUJOCO,
  capability: 'single-authority Unitree G1 29-DoF free-base dynamics workspace with bounded low-level joint actuation, named contact identity, a verified standing posture controller, live async Python and bounded WebMCP',
  evidence: MODEL_EVIDENCE.NUMERICALLY_VERIFIED,
  limitations: [
    'Model identity, kinematics, inertials, joint limits, actuator effort limits, collision geometry and foot contact primitives are source-derived from the pinned Unitree G1 29-DoF fixed-rubber-hand description; joint armature, damping and friction loss come from Unitree\'s own low-level MuJoCo simulator at the same model.',
    'Standing is a bounded joint-space posture hold, verified unsupported and free-base. Perturbation recovery and walking are unsupported and are not exposed.',
    'The fixed rubber hands are passive geometry. No dexterous hand, grasp, force-sensor or tactile capability is modelled or claimed.',
    'No hardware comparison exists. Motor bandwidth, gearbox friction, compliance, backlash, contact material, sensor latency and control-network timing are unmeasured.',
  ],
});

export const UNITREE_G1_KINEMATIC_CAPABILITY = capabilityRecord({
  backend: EXECUTION_BACKENDS.LEGACY,
  capability: 'kinematic joint-pose inspection only',
  evidence: MODEL_EVIDENCE.MODEL_DERIVED,
  limitations: [
    'No dynamic balance, standing or walking controller is active.',
    'No physical contact plant is active.',
    'No hardware-validation claim is made.',
  ],
});

export function unitreeG1CapabilityById(id) {
  return UNITREE_G1_CAPABILITY_AUDIT.find((item) => item.id === id) || null;
}

export function assertCapabilityAudit() {
  const required = ['kinematic_pose_inspection', 'mounted_joint_dynamics', 'free_base_dynamics', 'fall_and_contact', 'standing', 'perturbation_recovery', 'walking', 'dexterous_hands', 'hardware_calibration'];
  const seen = new Set(UNITREE_G1_CAPABILITY_AUDIT.map((item) => item.id));
  const missing = required.filter((id) => !seen.has(id));
  if (missing.length) throw new Error(`Unitree G1 capability audit is missing: ${missing.join(', ')}`);
  for (const id of ['walking', 'perturbation_recovery', 'dexterous_hands', 'hardware_calibration']) {
    if (unitreeG1CapabilityById(id).capability !== 'unsupported') throw new Error(`Unitree G1 capability ${id} must remain unsupported until it has its own evidence`);
  }
  return UNITREE_G1_CAPABILITY_AUDIT.length;
}
