import {SO101_MANIPULATION_MODEL_PACKAGE} from '../physics/model-packages.js';
export function simulatorGestureAdapter(app) {
  let authority = null;
  let pending = null;
  let selectedJoint = null, centerTarget = null, wasMoving = false;
  const matches = () => {
    const current = app.sim.getPhysicalAuthorityToken();
    return authority && current && authority.sessionId === current.sessionId && authority.epoch === current.epoch && app.profileId === 'so101';
  };
  const availability = () => {
    if (app.profileId !== 'so101' || app.workspaceStatus !== 'ready' || !app.sim.isReady()) return 'Wait for the SO-101 simulator to load, or reset it.';
    if (app.executionState !== 'idle' || app.getAgentAccess() !== 'off') return 'Stop the program and turn Agent Assist off before enabling gestures.';
    return '';
  };
  return {
    label: 'SO-101 · Simulation',
    availability,
    readback(joint) {
      const position = app.sim.getState()?.observation?.joints?.[joint]?.positionRad;
      return {label: 'Simulated angle', value: Number.isFinite(position) ? (position * 180 / Math.PI).toFixed(1) + '°' : 'Unavailable'};
    },
    changed() { app.updateExecutionControls(); },
    async begin(joint) {
      const reason = availability();
      if (reason) throw new Error(reason);
      authority = app.sim.getPhysicalAuthorityToken();
      if (!authority) throw new Error('No SO-101 physics authority is available.');
      const measured = app.sim.getState()?.observation?.joints;
      const hold = {};
      for (const modelJoint of SO101_MANIPULATION_MODEL_PACKAGE.joints) {
        const actuator = SO101_MANIPULATION_MODEL_PACKAGE.actuators.find(a => a.jointId === modelJoint.id);
        const position = measured?.[modelJoint.id]?.positionRad;
        const lo = Math.max(modelJoint.rangeRad[0], actuator.controlRangeRad[0]);
        const hi = Math.min(modelJoint.rangeRad[1], actuator.controlRangeRad[1]);
        if (!Number.isFinite(position) || position < lo || position > hi) throw new Error('A measured joint is outside its control range. Reset the simulator.');
        hold[modelJoint.id] = position;
      }
      if (!Object.hasOwn(hold, joint)) throw new Error('Choose one SO-101 joint.');
      selectedJoint = joint;
      centerTarget = hold[joint];
      wasMoving = false;
      await app.sim.resume();
      // Transfer control through actuator goals at the observed pose. Otherwise
      // advancing this joint can resume another joint's old program target.
      pending = app.sim.applyPhysicalTargets(hold, {maxSteps: 100, advanceSeconds: 0});
      await pending;
      app.updateExecutionControls();
    },
    async send(joint, direction) {
      if (!matches() || app.executionState !== 'idle' || app.getAgentAccess() !== 'off') throw new Error('Simulator control changed. Set neutral and enable again.');
      if (joint !== selectedJoint || !Number.isFinite(direction) || Math.abs(direction) > 1) throw new Error('Invalid gesture input. Set neutral and enable again.');
      const observation = app.sim.getState()?.observation;
      const position = observation?.joints?.[joint]?.positionRad;
      const modelJoint = SO101_MANIPULATION_MODEL_PACKAGE.joints.find(j => j.id === joint);
      const actuator = SO101_MANIPULATION_MODEL_PACKAGE.actuators.find(j => j.jointId === joint);
      if (!Number.isFinite(position) || !modelJoint || !actuator) throw new Error('No valid measured joint state. Reset the simulator.');
      const lo = Math.max(modelJoint.rangeRad[0], actuator.controlRangeRad[0]);
      const hi = Math.min(modelJoint.rangeRad[1], actuator.controlRangeRad[1]);
      if (position < lo || position > hi) throw new Error('The selected joint left its control range. Reset the simulator.');
      const step = joint === 'gripper' ? .035 : Math.PI / 360;
      if (direction === 0 && wasMoving) centerTarget = position;
      wasMoving = direction !== 0;
      // Neutral retains one measured hold target. Replacing it with each new
      // observation would follow gravity/load deflection instead of holding.
      const target = direction === 0 ? centerTarget : Math.min(hi, Math.max(lo, position + direction * step));
      pending = app.sim.applyPhysicalTargets({[joint]: target}, {maxSteps: 100, advanceSeconds: .1});
      await pending;
    },
    async end() {
      // Let the one bounded in-flight step finish before changing backend pause
      // state. Overlapping pause with accept/advance can invalidate its timing audit.
      await pending?.catch(() => {});
      if (matches()) {
        // Re-enabling explicitly resumes the same session. No transform writes.
        await app.sim.pause();
      }
      authority = null;
      pending = null;
      selectedJoint = null;
      app.updateExecutionControls();
    },
  };
}
