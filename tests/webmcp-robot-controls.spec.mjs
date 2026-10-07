import { expect, test } from '@playwright/test';
import {
  createProfileControlSchema,
  executeProfileControl,
  getProfileControlDefinition,
  WEBMCP_DIRECT_CONTROL_PROFILES,
  WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
} from '../src/webmcp/robot-controls.js';
import { getUnitreeG1PhysicalControlDefinition } from '../src/webmcp/unitree-g1-physical-control.js';

function makeFacade(profileId, { simulationMode = '' } = {}) {
  const calls = [];
  const mode = simulationMode || (profileId === 'so101' ? 'physical_mujoco' : profileId === 'unitree' ? 'kinematic_pose' : 'source_plant');
  const state = {
    epoch: 7,
    workspaceGeneration: 3,
    simulatorEpoch: 11,
    executionState: 'idle',
    resetCount: 0,
    status: '',
    telemetry: {},
    bumpAuthorityOnPhysicalCall: false,
    authority: {
      sessionId: 'physical-session-1', epoch: 5,
      sceneRevision: 'p4-so101-benchmark-transfer-v3', robotId: 'so101_follower', simulationTimeSeconds: 0,
    },
  };
  const app = {
    getExecutionState: () => state.executionState,
    sim: {
      getPhysicalAuthorityToken: () => profileId === 'so101' ? { ...state.authority } : null,
      applyPhysicalTargets: async (targetsRad, options = {}) => {
        calls.push({ kind: 'physical', targetsRad: { ...targetsRad }, options: { ...options } });
        if (state.bumpAuthorityOnPhysicalCall) state.authority.epoch += 1;
        const first = Object.keys(targetsRad)[0];
        return {
          status: 'accepted', commandId: 'physical-command-1', acceptedTargetsRad: { ...targetsRad },
          observation: {
            simulationTimeSeconds: state.authority.simulationTimeSeconds + Number(options.advanceSeconds || 0),
            joints: { [first]: { positionRad: Number(targetsRad[first]) / 8 } },
            bodies: { benchmark_block: { positionM: [0.39416, -0.00169, 0.234] } },
            contactCount: 0,
          },
          taskEvaluation: { success: false },
        };
      },
      applyAction: async (action, options = {}) => {
        if (options.beforeTick) await options.beforeTick();
        calls.push({ kind: 'action', action: { ...action } });
        state.telemetry = { ...state.telemetry, ...action };
        return true;
      },
      advanceTime: async (seconds, options = {}) => {
        if (options.beforeTick) await options.beforeTick();
        calls.push({ kind: 'advance', seconds });
        return true;
      },
    },
    resetSimulation: async () => {
      state.resetCount += 1;
      state.telemetry = {};
      if (profileId === 'so101') state.authority.epoch += 1;
      return true;
    },
    setStatus: (message) => { state.status = message; },
    renderPanels: () => {},
    getAgentSnapshot: () => ({
      workspaceStatus: 'ready', workspaceGeneration: state.workspaceGeneration, profileId,
      taskId: profileId === 'so101' ? 'so101-physical-block-transfer' : 'mock-task', simulatorEpoch: state.simulatorEpoch,
      simulationMode: mode,
      simulation: {
        executionState: state.executionState, status: state.status, telemetry: { ...state.telemetry },
        contacts: {}, problems: [], preparedActionCount: 0,
      },
    }),
  };
  const facade = {
    app,
    activeControlId: null,
    controlSequence: 0,
    assertActive: (expectedEpoch) => { if (expectedEpoch !== state.epoch) throw new Error('stale epoch'); },
    getRegistrationContext: () => ({
      workspaceStatus: 'ready', simulationReady: true, profileId,
      simulationMode: mode,
      workspaceGeneration: state.workspaceGeneration, simulatorEpoch: state.simulatorEpoch,
    }),
    inspectSimulation: (snapshot) => ({
      executionState: snapshot.simulation.executionState,
      status: snapshot.simulation.status,
      telemetry: snapshot.simulation.telemetry,
      contacts: snapshot.simulation.contacts,
    }),
  };
  return { facade, state, calls };
}

test('direct WebMCP control is limited to SO-101, LeKiwi, and Unitree G1', () => {
  expect(WEBMCP_DIRECT_CONTROL_PROFILES).toEqual(['so101', 'lekiwi', 'unitree']);
  expect(getProfileControlDefinition(makeFacade('openarm').facade)).toBeNull();
  expect(getProfileControlDefinition(makeFacade('microduck').facade)).toBeNull();
  expect(getProfileControlDefinition(makeFacade('so101').facade)?.name).toBe('control_so101_simulation');
  expect(getProfileControlDefinition(makeFacade('lekiwi').facade)?.name).toBe('control_lekiwi_simulation');
  expect(getProfileControlDefinition(makeFacade('unitree').facade)?.name).toBe('control_unitree_g1_simulation');
});

test('a physical workspace suppresses the same profile\'s non-physical direct-control tool', () => {
  // A physical workspace has its own separately named, versioned tool. The source-plant and
  // kinematic pose tools write straight into their own non-physical plants, so neither may be
  // offered while a physical badge is displayed, and neither may share a name with a physical tool.
  expect(getProfileControlDefinition(makeFacade('lekiwi', { simulationMode: 'physical_mujoco' }).facade)).toBeNull();
  expect(getProfileControlDefinition(makeFacade('unitree', { simulationMode: 'physical_mujoco' }).facade)).toBeNull();
  expect(getUnitreeG1PhysicalControlDefinition(makeFacade('unitree', { simulationMode: 'physical_mujoco' }).facade)?.name).toBe('control_unitree_g1_physical_simulation');
  expect(getUnitreeG1PhysicalControlDefinition(makeFacade('unitree').facade)).toBeNull();
  expect(getProfileControlDefinition(makeFacade('unitree').facade)?.name).toBe('control_unitree_g1_simulation');
});

test('SO-101 schema is versioned, radian-only, and bounded by the executed joint/actuator intersection', () => {
  const schema = createProfileControlSchema('so101');
  const set = schema.oneOf.find((branch) => branch.properties.command.const === 'set_joint_targets');
  expect(set.properties.schema_version.const).toBe(WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION);
  const targets = set.properties.targets_rad;
  expect(Object.keys(targets.properties).sort()).toEqual(['elbow_flex','gripper','shoulder_lift','shoulder_pan','wrist_flex','wrist_roll']);
  expect(Object.keys(targets.properties).some((key) => key.endsWith('.pos'))).toBe(false);
  expect(targets.properties.shoulder_pan).toMatchObject({ minimum: -1.91986, maximum: 1.91986 });
  expect(targets.properties.gripper.minimum).toBe(-0.17453);
  expect(targets.properties.gripper.maximum).toBe(1.7453292);
  expect(targets.properties.wrist_roll.maximum).toBe(2.7438473);
  expect(set.properties.max_steps).toMatchObject({ minimum: 1, maximum: 5000 });
  expect(set.properties.advance_seconds).toMatchObject({ minimum: 0, maximum: 2 });
});

test('SO-101 WebMCP returns accepted radians separately from actual observed MuJoCo state', async () => {
  const { facade, calls, state } = makeFacade('so101');
  const before = { ...state.authority };
  const result = await executeProfileControl(facade, 'so101', {
    schema_version: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
    command: 'set_joint_targets',
    targets_rad: { shoulder_pan: 0.2 },
    advance_seconds: 0.02,
    max_steps: 20,
  }, new AbortController().signal, 7);
  expect(calls).toEqual([{ kind: 'physical', targetsRad: { shoulder_pan: 0.2 }, options: { maxSteps: 20, advanceSeconds: 0.02 } }]);
  expect(result).toMatchObject({
    ok: true, profileId: 'so101', command: 'set_joint_targets', hardwareValidated: false,
    schemaVersion: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
    acceptedTargetsRad: { shoulder_pan: 0.2 },
    observedState: { jointsRad: { shoulder_pan: 0.025 } },
  });
  expect(result.observedState.jointsRad.shoulder_pan).not.toBe(result.acceptedTargetsRad.shoulder_pan);
  expect(result.physicalAuthority).toMatchObject(before);

  await expect(executeProfileControl(facade, 'so101', {
    command: 'set_action', action: { 'shoulder_pan.pos': 12 },
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'INVALID_ARGUMENT' });
  await expect(executeProfileControl(facade, 'so101', {
    schema_version: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
    command: 'set_joint_targets', targets_rad: { gripper: -0.174532 },
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'INVALID_ARGUMENT' });
});

test('SO-101 reset may create a newer physical epoch while stale in-flight authority changes fail closed', async () => {
  const { facade, state } = makeFacade('so101');
  const beforeEpoch = state.authority.epoch;
  const reset = await executeProfileControl(facade, 'so101', {
    schema_version: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION, command: 'reset',
  }, new AbortController().signal, 7);
  expect(reset.reset).toBe(true);
  expect(reset.physicalAuthority.epoch).toBe(beforeEpoch + 1);

  state.bumpAuthorityOnPhysicalCall = true;
  await expect(executeProfileControl(facade, 'so101', {
    schema_version: WEBMCP_SO101_PHYSICAL_SCHEMA_VERSION,
    command: 'set_joint_targets', targets_rad: { shoulder_pan: 0.1 }, max_steps: 10,
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'OPERATION_CANCELLED' });
});

test('LeKiwi requires a duration for nonzero base velocity and auto-stops the base', async () => {
  const { facade, calls } = makeFacade('lekiwi');
  await expect(executeProfileControl(facade, 'lekiwi', {
    command: 'set_action', action: { 'x.vel': 0.2 },
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'INVALID_ARGUMENT' });

  const result = await executeProfileControl(facade, 'lekiwi', {
    command: 'set_action', action: { 'x.vel': 0.2, 'theta.vel': 15 }, duration_ms: 40,
  }, new AbortController().signal, 7);
  expect(calls).toEqual([
    { kind: 'action', action: { 'x.vel': 0.2, 'theta.vel': 15 } },
    { kind: 'advance', seconds: 0.02 },
    { kind: 'action', action: { 'x.vel': 0, 'y.vel': 0, 'theta.vel': 0 } },
  ]);
  expect(result).toMatchObject({ ok: true, profileId: 'lekiwi', durationMs: 40, baseAutoStopped: true });
});

test('Unitree G1 accepts a bounded partial pose and reset remains browser-only', async () => {
  const { facade, state, calls } = makeFacade('unitree');
  const pose = await executeProfileControl(facade, 'unitree', {
    command: 'set_action', action: { waist_pitch_joint: 8, left_elbow_joint: 45 },
  }, new AbortController().signal, 7);
  expect(calls[0]).toEqual({ kind: 'action', action: { waist_pitch_joint: 8, left_elbow_joint: 45 } });
  expect(pose.simulation.telemetry).toMatchObject({ waist_pitch_joint: 8, left_elbow_joint: 45 });

  const reset = await executeProfileControl(facade, 'unitree', { command: 'reset' }, new AbortController().signal, 7);
  expect(state.resetCount).toBe(1);
  expect(reset).toMatchObject({ ok: true, profileId: 'unitree', command: 'reset', reset: true, hardwareValidated: false });
});
