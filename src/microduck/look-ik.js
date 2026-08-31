import { HEAD_LIMITS, clamp, deepFreeze } from './contract.js';

const BODY_TRANSFORMS = Object.freeze([
  { position: [0.026, 0.0145011, 0.0324424], quaternion: [0, 0, -0.707107, 0.707107] },
  { position: [0, -0.05, 0], quaternion: [0, 1, 0, 0] },
  { position: [0, 0.0186931, -0.0145], quaternion: [0, 0, -0.707107, -0.707107] },
  { position: [-0.0179, 0, 0.0145], quaternion: [0.707107, 0, -0.707107, 0] },
]);
const CAMERA_TRANSFORM = Object.freeze({
  position: [0.01175, 0, -0.0735],
  quaternion: [0.707107, 0, 0.707107, 0],
});
const SITE_TO_CV2 = Object.freeze([0.5, -0.5, 0.5, -0.5]);
const LIMITS = Object.freeze([HEAD_LIMITS.neckPitch, HEAD_LIMITS.headPitch, HEAD_LIMITS.headYaw, HEAD_LIMITS.headRoll]);

function multiply(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

function conjugate(q) { return [q[0], -q[1], -q[2], -q[3]]; }

function rotate(q, v) {
  const result = multiply(multiply(q, [0, ...v]), conjugate(q));
  return result.slice(1);
}

function compose(parent, child) {
  const offset = rotate(parent.quaternion, child.position);
  return {
    position: parent.position.map((value, index) => value + offset[index]),
    quaternion: multiply(parent.quaternion, child.quaternion),
  };
}

function zRotation(angle) { return [Math.cos(angle / 2), 0, 0, Math.sin(angle / 2)]; }

function cameraPose(joints) {
  let pose = { position: [0, 0, 0], quaternion: [1, 0, 0, 0] };
  for (let index = 0; index < BODY_TRANSFORMS.length; index += 1) {
    pose = compose(pose, BODY_TRANSFORMS[index]);
    pose.quaternion = multiply(pose.quaternion, zRotation(joints[index]));
  }
  pose = compose(pose, CAMERA_TRANSFORM);
  pose.quaternion = multiply(pose.quaternion, SITE_TO_CV2);
  return pose;
}

function gazeError(target, joints) {
  const camera = cameraPose(joints);
  const relative = target.map((value, index) => value - camera.position[index]);
  const cameraVector = rotate(conjugate(camera.quaternion), relative);
  const flat = Math.hypot(cameraVector[0], cameraVector[2]);
  return [Math.atan2(cameraVector[0], cameraVector[2]), Math.atan2(cameraVector[1], flat)];
}

// Port of pinned MicroDuck HeadFk::look_at: real head-chain FK, finite-difference
// Jacobian, damped Gauss-Newton, bounded step, and per-iteration joint clamps.
export function solveLookAt(target, neckPitch = 0) {
  const tolerance = 1e-4;
  const stepH = 1e-5;
  const lambda = 1e-3;
  const maxStep = 0.7;
  const joints = [clamp(neckPitch, LIMITS[0]), 0, 0, 0];
  let residual = Number.POSITIVE_INFINITY;
  for (let iteration = 0; iteration < 30; iteration += 1) {
    const error = gazeError(target, joints);
    residual = Math.max(Math.abs(error[0]), Math.abs(error[1]));
    if (residual < tolerance) break;
    const jacobian = [[0, 0], [0, 0]];
    for (let column = 0; column < 2; column += 1) {
      const probe = [...joints];
      probe[column + 1] += stepH;
      const nextError = gazeError(target, probe);
      jacobian[0][column] = (nextError[0] - error[0]) / stepH;
      jacobian[1][column] = (nextError[1] - error[1]) / stepH;
    }
    const a = jacobian[0][0] ** 2 + jacobian[1][0] ** 2 + lambda;
    const b = jacobian[0][0] * jacobian[0][1] + jacobian[1][0] * jacobian[1][1];
    const d = jacobian[0][1] ** 2 + jacobian[1][1] ** 2 + lambda;
    const gradient = [
      jacobian[0][0] * error[0] + jacobian[1][0] * error[1],
      jacobian[0][1] * error[0] + jacobian[1][1] * error[1],
    ];
    const determinant = a * d - b * b;
    if (Math.abs(determinant) < 1e-12) break;
    const step = [
      (-d * gradient[0] + b * gradient[1]) / determinant,
      (b * gradient[0] - a * gradient[1]) / determinant,
    ];
    const scale = Math.min(1, maxStep / Math.hypot(...step));
    joints[1] = clamp(joints[1] + scale * step[0], LIMITS[1]);
    joints[2] = clamp(joints[2] + scale * step[1], LIMITS[2]);
  }
  return deepFreeze({
    joints: { neckPitch: joints[0], headPitch: joints[1], headYaw: joints[2], headRoll: joints[3] },
    clamped: residual >= tolerance,
    residual,
  });
}
