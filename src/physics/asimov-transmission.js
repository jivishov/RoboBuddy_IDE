/** Small-angle parallel ankle mathematics, NOT an activated hardware profile.
 * Menlo convention: theta_A=kp*p-kr*r, theta_B=-kp*p-kr*r.
 * The numeric ratios and per-motor inertias must be supplied with provenance.
 * No default ratios: the current full-body source does not establish them.
 */
export function parallelAnkle({ pitchRatio, rollRatio, motorInertiasKgM2 } = {}) {
  for (const v of [pitchRatio, rollRatio]) if (typeof v !== 'number' || !Number.isFinite(v) || v <= 0) throw new TypeError('Positive explicit ankle linkage ratios required');
  if (!Array.isArray(motorInertiasKgM2) || motorInertiasKgM2.length !== 2 || motorInertiasKgM2.some(v => !Number.isFinite(v) || v <= 0)) throw new TypeError('Two explicit per-motor output inertias required; not doubled joint-space values');
  const kp = pitchRatio, kr = rollRatio, [ia, ib] = motorInertiasKgM2;
  const pair = values => { if (!Array.isArray(values) || values.length !== 2 || values.some(v => typeof v !== 'number' || !Number.isFinite(v))) throw new TypeError('Finite two-vector required'); return values; };
  const toMotor = values => { const [p,r] = pair(values); return [kp*p-kr*r, -kp*p-kr*r]; };
  const toJoint = values => { const [a,b] = pair(values); return [(a-b)/(2*kp), -(a+b)/(2*kr)]; };
  const jointToMotorTorque = values => { const [p,r] = pair(values); return [p/(2*kp)-r/(2*kr), -p/(2*kp)-r/(2*kr)]; };
  const motorToJointTorque = values => { const [a,b] = pair(values); return [kp*(a-b), -kr*(a+b)]; };
  return Object.freeze({ toMotor, toJoint, jointToMotorTorque, motorToJointTorque,
    // L^T diag(I_A,I_B) L. It generally has off-diagonal terms, so cannot be
    // added to both independent joint armatures without a model transformation.
    reflectedInertiaKgM2: [[kp*kp*(ia+ib),kp*kr*(ib-ia)],[kp*kr*(ib-ia),kr*kr*(ia+ib)]],
    limitJointTorque(values, motorLimitsNm) {
      const limits = pair(motorLimitsNm); if (limits.some(v => v <= 0)) throw new RangeError('Positive motor torque limits required');
      const requested = jointToMotorTorque(values), accepted = requested.map((v,i)=>Math.max(-limits[i],Math.min(limits[i],v)));
      return { requestedMotorNm:requested, acceptedMotorNm:accepted, acceptedJointNm:motorToJointTorque(accepted), saturated:accepted.some((v,i)=>v!==requested[i]) };
    },
  });
}
