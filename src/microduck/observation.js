import { HOME_POLICY_POSITION, wireToPolicy } from './contract.js';

export const OBSERVATION_LAYOUT = Object.freeze({
  gyro: Object.freeze([0, 3]),
  projectedGravity: Object.freeze([3, 6]),
  jointPosition: Object.freeze([6, 20]),
  jointVelocity: Object.freeze([20, 34]),
  lastAction: Object.freeze([34, 48]),
  command: Object.freeze([48, 61]),
});

function fourteen(values, label) {
  if (!values || values.length !== 14) throw new TypeError(`${label} must contain fourteen values.`);
  return Array.from(values, Number);
}

export function buildObservation({
  gyro = [0, 0, 0],
  projectedGravity = [0, 0, -1],
  wireJointPosition,
  policyJointPosition,
  wireJointVelocity,
  policyJointVelocity,
  lastAction = new Array(14).fill(0),
  command = {},
} = {}) {
  const position = fourteen(policyJointPosition || wireToPolicy(wireJointPosition), 'Joint position');
  const velocity = fourteen(policyJointVelocity || wireToPolicy(wireJointVelocity), 'Joint velocity');
  const twist = Array.from(command.twist || [0, 0, 0], Number);
  const head = Array.from(command.head || [0, 0, 0, 0], Number);
  const body = command.body || {};
  const values = [
    ...gyro, ...projectedGravity,
    ...position.map((value, index) => value - HOME_POLICY_POSITION[index]),
    ...velocity,
    ...fourteen(lastAction, 'Last action'),
    ...twist, ...head,
    0, 0, Number(body.z || 0), Number(body.roll || 0), Number(body.pitch || 0), 0,
  ];
  if (values.length !== 61 || values.some((value) => !Number.isFinite(Number(value)))) throw new TypeError('Observation must be exactly sixty-one finite values.');
  return Float32Array.from(values);
}
