import loadMujoco from '../../assets/microduck/runtime/mujoco/mujoco.js';
import { HOME_POLICY_POSITION, PHYSICS_TIMESTEP_SECONDS } from './contract.js';
import { MICRODUCK_BALL_BOUNDARY_RADIUS_M, MICRODUCK_DUCK_BOUNDARY_RADIUS_M, MICRODUCK_FIELD_HALF_EXTENT_M, stopPlanarBodyAtFieldEdge } from './field-bounds.js';
import { applyConfiguredKick, applyRollingResistance, BALL_KICK_DELAY_SECONDS } from './ball-motion.js';

const ROOT_QPOS = 0;
const JOINT_QPOS = 7;
const BALL_QPOS = 21;
const ROOT_QVEL = 0;
const JOINT_QVEL = 6;
const BALL_QVEL = 20;

function approximateModelXml(joints) {
  const jointBodies = joints.map((joint, index) => {
    const side = index < 5 ? 1 : index > 8 ? -1 : 0;
    const z = index < 5 || index > 8 ? -0.055 - (index % 5) * 0.008 : 0.02 + (index - 5) * 0.012;
    const range = (joint.rangeRad || [-Math.PI, Math.PI]).join(' ');
    return `<body name="approx_${joint.name}" pos="0 ${side * 0.028} ${z}"><joint name="${joint.name}" axis="${joint.axis.join(' ')}" range="${range}" damping="0.2" armature="0.01" frictionloss="0.02"/><geom type="capsule" size="0.008 0.014" mass="0.012" contype="0" conaffinity="0"/></body>`;
  }).join('');
  const actuators = joints.map((joint) => `<motor name="act_${joint.name}" joint="${joint.name}" ctrllimited="true" ctrlrange="-2 2"/>`).join('');
  return `<mujoco model="robobuddy_microduck_approx"><compiler angle="radian"/><option timestep="${PHYSICS_TIMESTEP_SECONDS}" gravity="0 0 -9.81" integrator="RK4"/><default><joint limited="true"/></default><worldbody><geom name="arena" type="plane" size="${MICRODUCK_FIELD_HALF_EXTENT_M} ${MICRODUCK_FIELD_HALF_EXTENT_M} 0.05" friction="0.9 0.04 0.01" rgba="0.4 0.45 0.47 1"/><body name="microduck_root" pos="0 0 0.13"><freejoint name="microduck_free"/><geom name="approx_trunk" type="box" size="0.045 0.038 0.052" mass="0.38" friction="0.85 0.04 0.01"/>${jointBodies}</body><body name="ball" pos="0.32 0 0.035"><freejoint name="ball_free"/><geom name="ball_geom" type="sphere" size="0.035" mass="0.045" friction="0.7 0.02 0.01" rgba="0.9 0.3 0.12 1"/></body></worldbody><actuator>${actuators}</actuator></mujoco>`;
}

export class MicroDuckMujocoDynamics {
  static async create(rigData) {
    const base = new URL('../../assets/microduck/runtime/mujoco/', import.meta.url);
    const module = await loadMujoco({ locateFile: (path) => new URL(path, base).href });
    return new MicroDuckMujocoDynamics(module, rigData);
  }

  constructor(module, rigData) {
    this.module = module;
    this.joints = rigData.joints;
    this.model = module.from_xml_string(approximateModelXml(this.joints));
    this.data = new module.MjData(this.model);
    this.pendingKick = null;
    this.lastKickContact = false;
    this.reset();
  }

  reset({ ball = [0.32, 0, 0.035] } = {}) {
    this.module.mj_resetData(this.model, this.data);
    for (let index = 0; index < 14; index += 1) this.data.qpos[JOINT_QPOS + index] = HOME_POLICY_POSITION[index];
    this.data.qpos[BALL_QPOS] = ball[0]; this.data.qpos[BALL_QPOS + 1] = ball[1]; this.data.qpos[BALL_QPOS + 2] = ball[2];
    this.data.qpos[BALL_QPOS + 3] = 1;
    this.module.mj_forward(this.model, this.data);
    this.previousBallVelocity = [0, 0, 0];
    this.pendingKick = null;
    this.lastKickContact = false;
  }

  step(targets, movement, { actuationEnabled = true, movementEnabled = true } = {}) {
    for (let index = 0; index < 14; index += 1) {
      const position = Number(this.data.qpos[JOINT_QPOS + index]);
      const velocity = Number(this.data.qvel[JOINT_QVEL + index]);
      const force = 18 * (Number(targets[index]) - position) - 0.35 * velocity;
      this.data.ctrl[index] = actuationEnabled ? Math.max(-2, Math.min(2, force)) : 0;
    }
    if (movementEnabled) {
      this.data.qvel[ROOT_QVEL] = Number(movement[0]) || 0;
      this.data.qvel[ROOT_QVEL + 1] = Number(movement[1]) || 0;
      this.data.qvel[ROOT_QVEL + 5] = Number(movement[2]) || 0;
    }
    this.module.mj_step(this.model, this.data);
    this.lastKickContact = this.applyPendingKick();
    this.applyBallRollingResistance();
    if (this.stopAtFieldEdge()) this.module.mj_forward(this.model, this.data);
  }

  queueKick(skill) {
    if (skill !== 'kick_left' && skill !== 'kick_right') return false;
    this.pendingKick = { skill, releaseAt: Number(this.data.time) + BALL_KICK_DELAY_SECONDS };
    return true;
  }

  applyPendingKick() {
    const pending = this.pendingKick;
    if (!pending || Number(this.data.time) < pending.releaseAt) return false;
    this.pendingKick = null;
    const result = applyConfiguredKick({
      skill: pending.skill,
      rootPosition: this.data.qpos.slice(ROOT_QPOS, ROOT_QPOS + 3),
      rootQuaternion: this.data.qpos.slice(ROOT_QPOS + 3, ROOT_QPOS + 7),
      ballPosition: this.data.qpos.slice(BALL_QPOS, BALL_QPOS + 3),
      ballVelocity: this.data.qvel.slice(BALL_QVEL, BALL_QVEL + 3),
    });
    for (let index = 0; index < 3; index += 1) this.data.qvel[BALL_QVEL + index] = result.velocity[index];
    return result.contacted;
  }

  applyBallRollingResistance() {
    const velocity = applyRollingResistance(this.data.qvel.slice(BALL_QVEL, BALL_QVEL + 3), PHYSICS_TIMESTEP_SECONDS);
    for (let index = 0; index < 3; index += 1) this.data.qvel[BALL_QVEL + index] = velocity[index];
  }

  stopAtFieldEdge() {
    const duck = stopPlanarBodyAtFieldEdge(this.data.qpos.slice(ROOT_QPOS, ROOT_QPOS + 3), this.data.qvel.slice(ROOT_QVEL, ROOT_QVEL + 3), MICRODUCK_DUCK_BOUNDARY_RADIUS_M);
    const ball = stopPlanarBodyAtFieldEdge(this.data.qpos.slice(BALL_QPOS, BALL_QPOS + 3), this.data.qvel.slice(BALL_QVEL, BALL_QVEL + 3), MICRODUCK_BALL_BOUNDARY_RADIUS_M);
    for (let index = 0; index < 3; index += 1) {
      this.data.qpos[ROOT_QPOS + index] = duck.position[index]; this.data.qvel[ROOT_QVEL + index] = duck.velocity[index];
      this.data.qpos[BALL_QPOS + index] = ball.position[index]; this.data.qvel[BALL_QVEL + index] = ball.velocity[index];
    }
    return duck.reachedBoundary || ball.reachedBoundary;
  }

  spawnBall(position = [0.28, 0, 0.035]) {
    for (let index = 0; index < 3; index += 1) { this.data.qpos[BALL_QPOS + index] = Number(position[index]) || 0; this.data.qvel[BALL_QVEL + index] = 0; }
    this.data.qpos[BALL_QPOS + 3] = 1; this.data.qpos[BALL_QPOS + 4] = 0; this.data.qpos[BALL_QPOS + 5] = 0; this.data.qpos[BALL_QPOS + 6] = 0;
    this.module.mj_forward(this.model, this.data);
  }

  perturb(orientation = 'face_down') {
    const half = Math.SQRT1_2;
    const quaternion = orientation === 'face_up' ? [0, half, 0, half] : [0, half, 0, -half];
    for (let index = 0; index < 4; index += 1) this.data.qpos[ROOT_QPOS + 3 + index] = quaternion[index];
    this.data.qpos[ROOT_QPOS + 2] = 0.075;
    this.module.mj_forward(this.model, this.data);
  }

  setUpright() {
    this.data.qpos[ROOT_QPOS + 2] = 0.13;
    this.data.qpos[ROOT_QPOS + 3] = 1;
    this.data.qpos[ROOT_QPOS + 4] = 0; this.data.qpos[ROOT_QPOS + 5] = 0; this.data.qpos[ROOT_QPOS + 6] = 0;
    this.module.mj_forward(this.model, this.data);
  }

  snapshot() {
    const quaternion = Array.from(this.data.qpos.slice(ROOT_QPOS + 3, ROOT_QPOS + 7));
    const projectedGravity = projectGravity(quaternion);
    const ballVelocity = Array.from(this.data.qvel.slice(BALL_QVEL, BALL_QVEL + 3));
    const ballMovedByContact = this.lastKickContact || (Number(this.data.ncon) > 0 && Math.hypot(...ballVelocity) > Math.hypot(...this.previousBallVelocity) + 1e-8);
    this.previousBallVelocity = ballVelocity;
    return {
      time: Number(this.data.time),
      joints: Array.from(this.data.qpos.slice(JOINT_QPOS, JOINT_QPOS + 14)),
      jointVelocities: Array.from(this.data.qvel.slice(JOINT_QVEL, JOINT_QVEL + 14)),
      position: Array.from(this.data.qpos.slice(ROOT_QPOS, ROOT_QPOS + 3)),
      quaternion,
      gyro: Array.from(this.data.qvel.slice(ROOT_QVEL + 3, ROOT_QVEL + 6)),
      projectedGravity,
      contacts: { count: Number(this.data.ncon) || 0, ballContact: ballMovedByContact },
      ball: { position: Array.from(this.data.qpos.slice(BALL_QPOS, BALL_QPOS + 3)), velocity: ballVelocity },
    };
  }

  dispose() { this.data?.delete?.(); this.model?.delete?.(); this.data = null; this.model = null; }
}

export function projectGravity([w, x, y, z]) {
  return [2 * (x * z - w * y), 2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))];
}
