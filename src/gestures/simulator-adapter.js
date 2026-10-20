import {SO101_MANIPULATION_MODEL_PACKAGE} from '../physics/model-packages.js';
export function simulatorGestureAdapter(app) {
  let authority = null;
  let pending = null;
  const matches = () => {
    const current = app.sim.getPhysicalAuthorityToken();
    return authority && current && authority.sessionId === current.sessionId && authority.epoch === current.epoch && app.profileId === 'so101';
  };
  return {
    label: 'SIMULATION · BROWSER MUJOCO',
    changed() { app.updateExecutionControls(); },
    async begin() {
      if (app.profileId !== 'so101' || app.workspaceStatus !== 'ready' || !app.sim.isReady()) throw new Error('Wait for the SO-101 simulator to load, or reset it.');
      if (app.executionState !== 'idle' || app.getAgentAccess() !== 'off') throw new Error('Stop the program and turn Agent Assist off before enabling gestures.');
      authority = app.sim.getPhysicalAuthorityToken();
      if (!authority) throw new Error('No SO-101 physics authority is available.');
      await app.sim.resume();
      app.updateExecutionControls();
    },
    async send(joint, direction) {
      if (!matches() || app.executionState !== 'idle' || app.getAgentAccess() !== 'off') throw new Error('Simulator control changed. Set neutral and enable again.');
      const observation = app.sim.getState()?.observation;
      const position = observation?.joints?.[joint]?.positionRad;
      const modelJoint = SO101_MANIPULATION_MODEL_PACKAGE.joints.find(j => j.id === joint);
      const actuator = SO101_MANIPULATION_MODEL_PACKAGE.actuators.find(j => j.jointId === joint);
      if (!Number.isFinite(position) || !modelJoint || !actuator) throw new Error('No valid measured joint state. Reset the simulator.');
      const lo = Math.max(modelJoint.rangeRad[0], actuator.controlRangeRad[0]) + .005;
      const hi = Math.min(modelJoint.rangeRad[1], actuator.controlRangeRad[1]) - .005;
      const step = joint === 'gripper' ? .035 : Math.PI / 360;
      const target = Math.min(hi, Math.max(lo, position + direction * step));
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
      app.updateExecutionControls();
    },
  };
}
