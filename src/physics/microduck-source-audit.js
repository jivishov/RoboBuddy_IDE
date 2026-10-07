import { PARAMETER_EVIDENCE } from './model-registry.js';

// Phase 5C source pins.
//
// P5C is a matched-environment integration, so it needs two pinned sources, not one.
// Neither is authority for everything:
//
//   * MICRODUCK_RUNTIME_SOURCE is the deployed controller/runtime authority. It supplies the
//     exact policy bytes, the 61-value observation contract, the joint order, the home pose,
//     the command encoding, the skill priority chain, the action scaling, the filters and the
//     50 Hz control cadence.
//   * MICRODUCK_RL_SOURCE is the physical/RL environment authority. It supplies the
//     task-specific collision plants, the BAM training-plant configuration, the source XML
//     fallback actuator, the physics timestep, reset pose and ball prop - none of which exist
//     in the deployed runtime source.
//
// The reconciliation between them is not assumed from names or joint counts. It is
// established below and asserted by tests/physics/microduck-phase5c-core.mjs.
export const MICRODUCK_RUNTIME_SOURCE = Object.freeze({
  repository: 'pollen-robotics/microduck',
  revision: '590b986bd8c0d50ae02cb3ea2f59c463b6828168',
  variant: 'alpha - the only shipped MicroDuck revision; every deployed policy is alpha_*',
  paths: Object.freeze([
    'kinematics/assets/alpha/robot_walk.xml',
    'duck-control/src/obs.rs',
    'duck-control/src/model.rs',
    'duck-control/src/policy.rs',
    'robotd/src/control.rs',
    'policies/README.md',
    'robotctl/assets/duck.bin',
  ]),
  license: 'Apache-2.0',
});

export const MICRODUCK_RL_SOURCE = Object.freeze({
  repository: 'pollen-robotics/microduck_rl',
  revision: '519142b1f5bf59fdfd44d06c205119e7fff8e3cb',
  revisionSubject: 'kick (2026-07-27)',
  paths: Object.freeze([
    'src/mjlab_microduck/robot/microduck/robot_walk.xml',
    'src/mjlab_microduck/robot/microduck/robot_allcollisions.xml',
    'src/mjlab_microduck/robot/microduck/config_mjcf_walk.json',
    'src/mjlab_microduck/robot/microduck/joints_properties.xml',
    'src/mjlab_microduck/robot/microduck/scene.xml',
    'src/mjlab_microduck/robot/microduck/ball.xml',
    'src/mjlab_microduck/robot/microduck_constants.py',
    'src/mjlab_microduck/tasks/microduck_velstand_env_cfg.py',
    'src/mjlab_microduck/tasks/microduck_velocity_env_cfg.py',
    'src/mjlab_microduck/tasks/microduck_ball_kick_env_cfg.py',
    'scripts/infer_policy.py',
  ]),
  license: 'Upstream currently declares project software Apache-2.0 and 3D model files Creative Commons BY-SA-NC (CC version not specified). Exact pinned STL collision bytes are redistributed here under that separate upstream 3D-model scope.',
  selectionReason:
    'It is the newest revision whose robot_walk.xml is still the deployed model AND which already carries the source ball prop. '
    + 'The deployed-model window on this branch runs from 6594292 (2026-07-01, "update model (head_pitch in right direction)") to '
    + 'd649ce8 (2026-08-06); 1ee2e3c (2026-07-28, "update robot model with more accurate weights") re-weights the model and is '
    + 'therefore outside it. Current develop is outside it too, which is why this package does not track upstream head.',
});

// The historical training-run repository the deployed policy bytes were copied from.
// policies/README.md at the deployed pin names it and the exact commit for each file.
// It is recorded for provenance completeness; no parameter in this package comes from it,
// because the deployed repository already carries the exact bytes.
export const MICRODUCK_POLICY_ORIGIN_SOURCE = Object.freeze({
  repository: 'apirrone/microduck_runtime',
  revision: '5f3b314',
  rouladeRevision: '7e4ab6d',
  role: 'origin of the nine deployed ONNX policy files, vendored verbatim into the deployed runtime repository',
  status: 'not-inspected',
  reason: 'The deployed runtime repository at the pin carries the exact policy bytes this repository already serves, verified by SHA-256. Nothing further is needed from the origin repository, and it was not used as a parameter source.',
});

// The nine deployed policies and the role each one fills, from policies/README.md at the
// deployed pin. `net` is the label robotd's scheduler uses.
export const MICRODUCK_DEPLOYED_POLICIES = Object.freeze([
  Object.freeze({ id: 'walking', file: 'alpha_walking.onnx', net: 'walk', upstreamRun: 'BEST_alpha_walking_rough.onnx', role: 'walking / velstand', trainingTask: 'VelStand (velocity2 recipe + gated fall-recovery layer)' }),
  Object.freeze({ id: 'stand', file: 'alpha_stand.onnx', net: 'stand', upstreamRun: 'BEST_alpha_stand_body_control.onnx', role: 'standing + body-pose', trainingTask: 'StandUp / body-pose control' }),
  Object.freeze({ id: 'sitstand', file: 'alpha_sitstand.onnx', net: 'sit_stand', upstreamRun: 'BEST_alpha_sitstand.onnx', role: 'sit <-> stand (posture flag in the twist vx slot)', trainingTask: 'SitStand' }),
  Object.freeze({ id: 'ground_pick', file: 'alpha_ground_pick.onnx', net: 'ground_pick', upstreamRun: 'alpha_ground_pick.onnx', role: 'ground pick (phase command in the twist slots)', trainingTask: 'GroundPick' }),
  Object.freeze({ id: 'kick_left', file: 'ball_kick_left.onnx', net: 'kick_left', upstreamRun: 'ball_kick_left.onnx', role: 'left-leg kick', trainingTask: 'BallKick' }),
  Object.freeze({ id: 'kick_right', file: 'ball_kick_right.onnx', net: 'kick_right', upstreamRun: 'ball_kick_right.onnx', role: 'right-leg kick', trainingTask: 'BallKick' }),
  Object.freeze({ id: 'roulade', file: 'roulade.onnx', net: 'roulade', upstreamRun: 'roulade.onnx', role: 'forward roll', trainingTask: 'Mjlab-Roulade-MicroDuck' }),
  Object.freeze({ id: 'roller', file: 'roller.onnx', net: 'roller', upstreamRun: 'BEST_roller.onnx', role: 'roller-mode locomotion', trainingTask: 'VelocityRollers (roller-skate plant)' }),
  Object.freeze({ id: 'roller_crouch', file: 'roller_crouch.onnx', net: 'roller_crouch', upstreamRun: 'BEST_roller_crounch.onnx', role: 'roller-mode crouch', trainingTask: 'RollerCrouch (roller-skate plant)' }),
]);

// Exact SHA-256 of the nine deployed policy files. These are the bytes this repository
// serves from assets/microduck/policies/ and the bytes committed at the deployed pin;
// they were compared directly and are identical.
export const MICRODUCK_POLICY_SHA256 = Object.freeze({
  walking: 'e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c',
  stand: '1569268713e40deea795dd2922dba50d3621e15a872855408b6b1b125b1c094b',
  sitstand: 'c6c40e35e726eabd803d633e090d112994f469921152448367953fbaf9799bc8',
  ground_pick: 'ffbf5109982ff999b0ba53afe86b9ae731bbec679d67fb7f8ab4c52152c88872',
  kick_left: 'd6928284dccd3dd61e08bf2f760effa74309fbefd97b2b31afb2a60f526d196a',
  kick_right: '147a32c388c6b19111b3ac3b550a9a6dc8b8bf267118af4d8c3712522eedb5af',
  roulade: '3d60da08fc13f29c1b57f41977aa898132c0d60042100149d8e775affcbca32b',
  roller: 'cf05651d2708a2f9364212e86b866c97a70ace8131c492500105e8f28bf99afd',
  roller_crouch: 'a1a084be240469c76ac9d3fa44d4792f16d4b1da60398b3ecd3cfc5e2244d990',
});

// The compatibility identity a physical policy is checked against before it may run.
// A mismatch in any field is a rejection, not a fidelity footnote.
export const MICRODUCK_COMPATIBILITY_SCHEMA_VERSION = 'microduck-p5c-1';

const row = (parameter, value, evidence, source, note) => Object.freeze({ parameter, value, evidence, source, note });

// Reconciliation table. Every physical and controller parameter the Phase 5C packages use
// appears here with its evidence class, its pinned source, and - where the two sources could
// have disagreed - the evidence that they do not.
export const MICRODUCK_RECONCILIATION = Object.freeze([
  row('robot variant', 'alpha', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/model.rs',
    'One variant. Every shipped policy is alpha_*; v1/v1.5/v1.6 are history in both sources.'),
  row('articulated hierarchy', '37 named bodies/joints/sites, 14 hinges, one free trunk', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck kinematics/assets/alpha/robot_walk.xml',
    'Compared entity-by-entity against microduck_rl robot_walk.xml at the selected revision: all 37 named entities present in both, '
    + 'with identical pos, quat, axis and range, and all 14 inertials identical. This is the evidence that the deployed runtime and the '
    + 'selected RL revision describe the same robot.'),
  row('total modelled mass', 0.744361, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'sum of the source inertials',
    'Compiled from the source inertials. Not a weighed robot; the CAD/inertial values are the source figures.'),
  row('home pose', '[0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0, 0, (mouth 0), 0, 0.0873, 0.4579, 0.0049, -0.4530]',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/model.rs DEFAULT_POSITION',
    'Identical to microduck_rl HOME_FRAME and to the STAND keyframe of the source scene. Both sources document it the same way: '
    + 'the STAND2 pose with the trunk ~5 mm forward so the CoM sits over the ankle axis. Observations are relative to it, so a '
    + 'discrepancy here would be a constant offset on fourteen observation slots.'),
  row('joint wire order', '15 slots, mouth at index 9', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-ipc-proto JOINT_NAMES, duck-control MOUTH_INDEX',
    'The physical model carries no mouth joint at all, matching the RL model; the wire order is preserved so the fourteen policy '
    + 'slots keep their published identity and the mouth slot cannot be filled by an off-by-one.'),
  row('observation layout', '61 = 3 gyro + 3 projected gravity + 14 (q - home) + 14 qdot + 14 previous raw action + 13 command',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs', 'Single layout across every alpha policy; the runtime rejects any other width at load.'),
  row('command block', '[vx, vy, vyaw, neck_pitch, head_pitch, head_yaw, head_roll, 0, 0, body_z, body_roll, body_pitch, 0]',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs',
    'Body x, y and yaw are unbound in training and are hardcoded zero. The body block is ordered z, roll, pitch.'),
  row('previous action semantics', 'raw policy output, before action scaling, shared across nets', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Controller::last_action', 'The policy was trained observing its own output, not the actuator command derived from it.'),
  row('observation history', 'none beyond the single previous action', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/obs.rs',
    'The 61-value contract carries exactly one previous action. No temporal stack exists in either source, so none is added.'),
  row('controller cadence', '50 Hz (20 ms)', PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/main.rs, scripts/infer_policy.py',
    'The deployed loop runs at 50 Hz; the RL reference runner uses decimation 4 over a 5 ms physics step, which is the same 20 ms interval.'),
  row('physics timestep', 0.005, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'scripts/infer_policy.py (model.opt.timestep = 0.005)',
    'The RL reference runner sets it explicitly; four steps per controller tick gives the deployed 50 Hz.'),
  row('integrator / solver', 'Euler, 100 solver iterations, 50 line-search iterations, pyramidal cone', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene.xml compiled defaults', 'The source scene overrides none of them, so these are the values the reference runner actually executes.'),
  row('action scaling (walk)', 0.9, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs Tuning::action_scale',
    'Audited divergence: the training environment uses JointPositionAction scale 1.0, and the RL reference runner defaults to 1.0. '
    + 'The deployed daemon de-rates walking to 0.9. This package reproduces the deployed value because this repository pins the '
    + 'deployed runtime, and records 1.0 as the training value. Measured effect in the native reference: about 7% less distance over 6 s.'),
  row('action scaling (stand/kick/rise)', 1.0, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs standing_action_scale',
    'Standing tuning also applies to a kick window and to the sitstand rise, because both present an all-zero effective command.'),
  row('action scaling (ground pick, roulade)', 1.0, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'robotd/src/control.rs SkillTuning', null),
  row('target low-pass filters', 'head joints alpha 0.5, other joints alpha 0.7', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Tuning::head_lowpass / legs_lowpass',
    'Audited divergence: the deployed source states the alpha policies are trained with these values, while the RL reference runner '
    + 'applies no filter and the training tree reverted its low-pass experiment. This package reproduces the deployed filters and '
    + 'reports both. Measured effect in the native reference: under 4% of walking distance either way.'),
  row('standing threshold', 0.05, PARAMETER_EVIDENCE.SOURCE_DERIVED, 'duck-control/src/policy.rs DEFAULT_STANDING_THRESHOLD',
    'Velocity-command magnitude below which the standing network takes over.'),
  row('skill priority chain', 'roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Controller::step', null),
  row('firmware gain schedule', '200 running; 160 standing / stand-tuned skills', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'robotd/src/control.rs Tuning::gain / standing_gain_ratio',
    'The physical worker passes the scheduled firmware gain into the BAM voltage-domain torque law. It does not emulate the register by mutating a MuJoCo position-actuator stiffness.'),
  row('BAM actuator model', 'Rhoban/bam v1.0.1 XL330/M6 voltage-domain torque motor', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'Rhoban/bam@ab81512c44f1f709b99ef332addb5e51568cd51c + microduck_rl actuator configuration',
    'Browser equations are independently cross-checked against better-actuator-models==1.0.1. Motor torque depends on target error, firmware gain, available voltage and back-EMF; MuJoCo ctrl is motor torque, not a position target.'),
  row('BAM armature and friction', 'identified armature 0.0018077432831600838 kg m^2 plus BAM directional/Stribeck/load/viscous friction', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'Rhoban/bam v1.0.1 M6 parameters',
    'BAM replaces source joint damping/frictionloss for the physical plant and supplies its own dynamic friction and identified armature. Training-reference friction scaling applies to the velocity-independent BAM friction budget; viscous friction remains nominal.'),
  row('source XML fallback actuator', 'position kp 0.55 N m/rad, kv 0, force range +/-0.96 N m, control range +/-10 rad', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl joints_properties.xml class chosen_actuator',
    'Retained only as source-XML/fallback evidence and pre-conversion validation. It is not the browser or native Phase 5C physical actuator authority.'),
  row('collision plant routing', 'walk/lowTraction -> robot_walk.xml; stand/recovery/sit_stand/ground_pick/roulade -> robot_allcollisions.xml; kick -> robot_allcollisions.xml + ball.xml', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl robot_walk.xml, robot_allcollisions.xml and pinned task configs',
    'Walking deliberately uses the reduced source collision plant. Body-on-ground skills and kick use the broad all-collision source plant. The pinned revision does not contain the roulade task configuration, so roulade uses the broad plant conservatively and remains physical/experimental.'),
  row('collision mesh bytes', 'exact pinned upstream STL bytes, no fitted primitive replacement', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl@519142b1f5bf59fdfd44d06c205119e7fff8e3cb assets + models/microduck/source/GIT_BLOB_SHA1SUMS',
    'The committed STL files are byte-compared against the pinned checkout and their Git blob identities are recorded. Generated task wrappers preserve source mesh geometry.'),
  row('contact parameters', 'task-specific source masks/priority/friction with explicit low-traction negative fixture', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl config_mjcf_walk.json / robot XML / ball.xml',
    'The normal task plants preserve source contact semantics. The low-traction package is the separately declared adverse fixture; it is not presented as a measured material.'),
  row('floor', 'flat plane at z = 0', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl scene/reference environment', 'A declared flat indoor-floor reference, not a measurement of a particular physical surface.'),
  row('reduced-traction surface', 0.02, PARAMETER_EVIDENCE.ESTIMATED, 'repository-authored adverse-condition fixture',
    'A deliberately degraded surface for the traction gate, matching the value the LeKiwi package already uses for the same purpose. Not a modelled real material.'),
  row('reset state', 'trunk at z = 0.12, identity orientation, joints at the home pose, actuator targets at the home pose',
    PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl scene.xml STAND keyframe',
    'The source keyframe spawns the robot 2.8 mm clear of the floor and lets it settle; this package keeps that exactly.'),
  row('ball radius / mass / inertia', [0.035, 0.015, 1.225e-5], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl ball.xml',
    '70 mm diameter, 15 g hollow plastic ball; the inertia is the thin-hollow-sphere value the source states.'),
  row('ball friction', [0.5, 0.005, 0.0001], PARAMETER_EVIDENCE.SOURCE_DERIVED, 'microduck_rl ball.xml',
    'Low rolling resistance so a kicked ball rolls. There is no separate rolling-resistance rule anywhere in the physical path.'),
  row('ball placement', 'x 0.09, y -0.042 in the robot yaw frame', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl microduck_ball_kick_env_cfg.py BALL_OFFSET', 'The nominal kick-task placement, in front of the right foot. Training added +/-15 mm of placement noise, which this package does not apply by default.'),
  row('gyro source', 'MuJoCo gyro sensor at the source-named imu site', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl sensors.xml, scripts/infer_policy.py', 'The imu site carries identity rotation relative to the trunk, so this is the trunk-frame angular velocity the deployed controller reads.'),
  row('projected gravity source', 'world -Z rotated into the trunk body frame', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'scripts/infer_policy.py get_projected_gravity', 'Taken from the trunk body quaternion, not from a simulated accelerometer.'),
  row('deployment IMU preprocessing', 'normalised projected gravity plus per-axis median-of-three on gyro and projected gravity', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'pollen-robotics/microduck@590b986 duck-control/src/imu.rs',
    'The deployment-reference workspace reproduces the deployed median-of-three pipeline; its history resets to two zero gyro samples and two upright gravity samples.'),
  row('deployment target delay', 'no synthetic target delay injected', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'deployed runtime control path',
    'The deterministic deployment-reference keeps the deployed controller path separate from training-only domain randomisation.'),
  row('training-reference randomisation', 'Vin 6.5..8.2 V; sag 0..0.2 V/Nm; 3..6 physics-step target delay; friction/armature 0.9..1.1; mass/inertia 0.95..1.05; encoder bias +/-0.015 rad; IMU mount +/-6 deg', PARAMETER_EVIDENCE.SOURCE_DERIVED,
    'microduck_rl training configuration and BAM integration',
    'These values are retained as a separate training-reference profile. They are not silently injected into the interactive deployment-reference workspace.'),
  row('deployment electrical reference', '7.4 V nominal, 0.1 V/Nm sag coefficient, 6.0 V minimum', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED,
    'source CPU regression/reference condition',
    'A deterministic source-backed rehearsal condition, not a battery/internal-resistance measurement from a particular assembled MicroDuck.'),
  row('hardware alignment', 'none', PARAMETER_EVIDENCE.CALIBRATION_REQUIRED, '-',
    'No measurement of an assembled MicroDuck was used anywhere in Phase 5C. Walking speed, traction, stability margin, kick distance, '
    + 'recovery probability and actuator dynamics are all unvalidated against hardware.'),
]);

export function reconciliationByEvidence(evidence) {
  return MICRODUCK_RECONCILIATION.filter((item) => item.evidence === evidence);
}

export function assertReconciliationCoverage() {
  const seen = new Set(MICRODUCK_RECONCILIATION.map((item) => item.parameter));
  const required = [
    'robot variant', 'articulated hierarchy', 'home pose', 'joint wire order', 'observation layout',
    'command block', 'previous action semantics', 'observation history', 'controller cadence',
    'physics timestep', 'action scaling (walk)', 'target low-pass filters', 'firmware gain schedule',
    'BAM actuator model', 'BAM armature and friction', 'source XML fallback actuator',
    'collision plant routing', 'collision mesh bytes', 'contact parameters', 'reset state',
    'deployment IMU preprocessing', 'training-reference randomisation', 'deployment electrical reference', 'hardware alignment',
  ];
  const missing = required.filter((name) => !seen.has(name));
  if (missing.length) throw new Error(`MicroDuck reconciliation table is missing: ${missing.join(', ')}`);
  for (const item of MICRODUCK_RECONCILIATION) {
    if (!Object.values(PARAMETER_EVIDENCE).includes(item.evidence)) {
      throw new Error(`MicroDuck reconciliation row ${item.parameter} has unknown evidence ${item.evidence}`);
    }
  }
  return MICRODUCK_RECONCILIATION.length;
}
