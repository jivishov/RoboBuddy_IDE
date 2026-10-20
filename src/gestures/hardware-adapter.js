export function hardwareGestureAdapter(client, getState, isSupported) {
  let gestureId = null;
  const availability = () => {
    const state = getState();
    if (!state?.capabilities?.webcamGestures) return 'Update and restart the local hardware helper to use gesture control.';
    if (!state.fresh || !state.calibrated || !state.operatorEnabled || !['armed','holding'].includes(state.state)) return 'Connect, calibrate and explicitly enable motors before enabling gestures.';
    if (!isSupported()) return 'Confirm that the arm is supported, mounted and clear.';
    return '';
  };
  return {
    label: 'SO-101 · Local arm',
    context: () => getState()?.mode === 'demo' ? 'SO-101 · Demo motors' : 'SO-101 · Local arm',
    availability,
    readback(joint) {
      const state = getState(), position = state?.joints?.[joint]?.position;
      return {label: state?.mode === 'demo' ? 'Demo encoder' : 'Encoder', value: state?.fresh && Number.isFinite(position) ? position.toFixed(1) + (joint === 'gripper' ? '% open' : '°') : 'Unavailable'};
    },
    async begin(joint) {
      const reason = availability();
      if (reason) throw new Error(reason);
      const result = await client.command('gesture_start', {joint, supported: true});
      gestureId = result.gesture?.id;
      if (!gestureId) throw new Error('The helper did not accept gesture control.');
    },
    async send(joint, direction) {
      if (!isSupported()) throw new Error('Support acknowledgement removed. Gesture control paused.');
      const state = getState();
      if (!gestureId || !state?.fresh || state.gesture?.id !== gestureId || state.state === 'fault') throw new Error('Hardware gesture session expired. Set neutral and enable again.');
      // Never queue camera commands behind another operation. The helper expires the
      // gesture lease independently of the browser's ordinary operator heartbeat.
      if (client.busy) return;
      await client.command('gesture_move', {gestureId, joint, direction});
    },
    async end() { gestureId = null; if (client.session) await client.stop(); },
  };
}
