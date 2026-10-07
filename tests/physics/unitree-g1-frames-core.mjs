import assert from 'node:assert/strict';
import { angularVelocityWorld } from '../../src/physics/unitree-g1-frames.js';
const near = (actual, expected) => actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-12));
near(angularVelocityWorld([1, 0, 0, 0], [1, 2, 3]), [1, 2, 3]);
// A 90-degree yaw maps a local X spin to world Y, not world X.
near(angularVelocityWorld([Math.SQRT1_2, 0, 0, Math.SQRT1_2], [1, 0, 0]), [0, 1, 0]);
near(angularVelocityWorld([0, 1, 0, 0], [1, 2, 3]), [1, -2, -3]);
assert.throws(() => angularVelocityWorld([1, 0, 0, 0], [NaN, 0, 0]), /finite/);
console.log('G1 free-joint local-to-world angular velocity frames OK');
