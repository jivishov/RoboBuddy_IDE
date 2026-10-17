export const JOINTS = Object.freeze(['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']);
export function visualPose(state, mapping = {}) {
  if (!state?.fresh || !state.calibrated) return null;
  const pose = {};
  for (const joint of JOINTS) {
    const observed = state.joints?.[joint]?.position;
    if (typeof observed !== 'number' || !Number.isFinite(observed)) return null;
    if (joint === 'gripper') { pose[joint + '.pos'] = observed; continue; }
    const entry = mapping[joint] || {sign: 1, offset: 0};
    if (![1, -1].includes(entry.sign) || !Number.isFinite(entry.offset) || Math.abs(entry.offset) > 360) throw new Error('Invalid 3D alignment.');
    pose[joint + '.pos'] = observed * entry.sign + entry.offset;
  }
  return pose;
}

