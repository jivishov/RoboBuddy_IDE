// Camera coordinates are relative input, never a Cartesian robot pose or depth measurement.
export const GESTURE_MAX_AGE_MS = 300;
export const GESTURE_JOINTS = Object.freeze({
  shoulder_pan: 'Shoulder pan', shoulder_lift: 'Shoulder lift', elbow_flex: 'Elbow',
  wrist_flex: 'Wrist flex', wrist_roll: 'Wrist roll', gripper: 'Gripper',
});
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const clamp = (x, a, b) => Math.min(b, Math.max(a, x));

export function analyzeHand(result) {
  if (result?.landmarks?.length !== 1) return {valid: false, reason: result?.landmarks?.length > 1 ? 'Show only one hand.' : 'Show one open hand.'};
  const points = result.landmarks[0];
  if (!Array.isArray(points) || points.length !== 21 || points.some(p => !p || !Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(p.z) || p.x < .02 || p.x > .98 || p.y < .02 || p.y > .98)) return {valid: false, reason: 'Keep your whole hand inside the camera view.'};
  const width = distance(points[5], points[17]);
  if (width < .055 || width > .48) return {valid: false, reason: 'Move your hand to a comfortable camera distance.'};
  const extended = [8, 12, 16, 20].map(tip => {
    const pip = points[tip - 2], base = points[tip - 3], end = points[tip];
    const ax = base.x - pip.x, ay = base.y - pip.y, bx = end.x - pip.x, by = end.y - pip.y;
    const cosine = (ax * bx + ay * by) / (Math.hypot(ax, ay) * Math.hypot(bx, by) || 1);
    return cosine < -.65 && distance(end, points[0]) > distance(pip, points[0]) * 1.1;
  });
  const palmIds = [0, 5, 9, 13, 17];
  return {
    valid: true, points, hand: result.handedness?.[0]?.[0]?.categoryName || 'hand',
    x: 1 - palmIds.reduce((sum, i) => sum + points[i].x, 0) / palmIds.length,
    pinch: distance(points[4], points[8]) / width,
    open: extended.every(Boolean), gripReady: extended.slice(1).every(Boolean),
    fist: extended.every(value => !value),
  };
}

export function gestureDirection(hand, neutral, joint, previous = 0) {
  if (!Object.hasOwn(GESTURE_JOINTS, joint) || !hand?.valid || !neutral || hand.hand !== neutral.hand) return 0;
  if (joint === 'gripper' ? !hand.gripReady : !hand.open) return 0;
  const delta = joint === 'gripper' ? hand.pinch - neutral.pinch : hand.x - neutral.x;
  const deadband = joint === 'gripper' ? .10 : .045;
  const span = joint === 'gripper' ? .45 : .20;
  // Center has an immediate zero; smoothing may never prolong a released command.
  if (Math.abs(delta) <= deadband) return 0;
  const requested = Math.sign(delta) * clamp((Math.abs(delta) - deadband) / span, 0, 1);
  return clamp(previous * .45 + requested * .55, -1, 1);
}
