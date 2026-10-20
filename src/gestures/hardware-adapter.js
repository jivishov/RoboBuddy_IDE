export function hardwareGestureAdapter(client, getState, isSupported) {
  let gestureId = null;
  return {
    label: 'LOCAL ARM · BOUNDED JOINT JOGS',
    async begin(joint) {
      const state = getState();
      if (!state?.capabilities?.webcamGestures) throw new Error('Update and restart the local hardware helper to use gesture control.');
      if (!state.fresh || !state.calibrated || !state.operatorEnabled || !['armed','holding'].includes(state.state)) throw new Error('Connect, calibrate and explicitly enable motors before enabling gestures.');
      if (!isSupported()) throw new Error('Confirm that the arm is supported, mounted and clear.');
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
