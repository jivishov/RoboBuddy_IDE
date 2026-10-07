// MuJoCo free-joint translation velocity is world-frame, but its rotational qvel is
// body-local. Observations labelled mujoco_world must rotate that angular vector.
// Observation-only conversion: no simulation state is changed.
export function angularVelocityWorld(quaternionWxyz, angularVelocityLocalRadS) {
  if (!Array.isArray(quaternionWxyz) || quaternionWxyz.length !== 4 || !quaternionWxyz.every(Number.isFinite)
      || !Array.isArray(angularVelocityLocalRadS) || angularVelocityLocalRadS.length !== 3 || !angularVelocityLocalRadS.every(Number.isFinite)) {
    throw new TypeError('Angular velocity conversion requires finite quaternion and vector');
  }
  const [w, x, y, z] = quaternionWxyz;
  const [a, b, c] = angularVelocityLocalRadS;
  return [
    (1 - 2 * (y*y + z*z))*a + 2*(x*y - w*z)*b + 2*(x*z + w*y)*c,
    2*(x*y + w*z)*a + (1 - 2*(x*x + z*z))*b + 2*(y*z - w*x)*c,
    2*(x*z - w*y)*a + 2*(y*z + w*x)*b + (1 - 2*(x*x + y*y))*c,
  ];
}
