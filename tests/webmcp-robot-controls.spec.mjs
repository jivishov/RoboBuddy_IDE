import { expect, test } from '@playwright/test';
import {
  createProfileControlSchema,
  executeProfileControl,
  getProfileControlDefinition,
  WEBMCP_DIRECT_CONTROL_PROFILES,
} from '../src/webmcp/robot-controls.js';

function makeFacade(profileId) {
  const calls = [];
  const state = {
    epoch: 7,
    workspaceGeneration: 3,
    simulatorEpoch: 11,
    executionState: 'idle',
    resetCount: 0,
    status: '',
    telemetry: {},
  };
  const app = {
    getExecutionState: () => state.executionState,
    sim: {
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
      return true;
    },
    setStatus: (message) => { state.status = message; },
    renderPanels: () => {},
    getAgentSnapshot: () => ({
      workspaceStatus: 'ready',
      workspaceGeneration: state.workspaceGeneration,
      profileId,
      taskId: 'mock-task',
      simulatorEpoch: state.simulatorEpoch,
      simulationMode: profileId === 'unitree' ? 'kinematic_pose' : 'source_plant',
      simulation: {
        executionState: state.executionState,
        status: state.status,
        telemetry: { ...state.telemetry },
        contacts: {},
        problems: [],
        preparedActionCount: 0,
      },
    }),
  };
  const facade = {
    app,
    activeControlId: null,
    controlSequence: 0,
    assertActive: (expectedEpoch) => {
      if (expectedEpoch !== state.epoch) throw new Error('stale epoch');
    },
    getRegistrationContext: () => ({
      workspaceStatus: 'ready',
      simulationReady: true,
      profileId,
      workspaceGeneration: state.workspaceGeneration,
      simulatorEpoch: state.simulatorEpoch,
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

test('generated schemas expose only configured profile action fields and bounded reset/set_action commands', () => {
  const so101 = createProfileControlSchema('so101');
  const soAction = so101.oneOf.find((branch) => branch.properties.command.const === 'set_action').properties.action;
  expect(Object.keys(soAction.properties).sort()).toEqual([
    'elbow_flex.pos', 'gripper.pos', 'shoulder_lift.pos', 'shoulder_pan.pos', 'wrist_flex.pos', 'wrist_roll.pos',
  ]);
  expect(soAction.properties['shoulder_pan.pos']).toMatchObject({ minimum: -110, maximum: 110 });
  expect(soAction.additionalProperties).toBe(false);

  const lekiwi = createProfileControlSchema('lekiwi');
  const lekiwiSet = lekiwi.oneOf.find((branch) => branch.properties.command.const === 'set_action');
  expect(lekiwiSet.properties.duration_ms).toMatchObject({ minimum: 20, maximum: 3000 });
  expect(lekiwiSet.properties.action.properties['x.vel']).toMatchObject({ minimum: -0.6, maximum: 0.6 });

  const unitree = createProfileControlSchema('unitree');
  const unitreeAction = unitree.oneOf.find((branch) => branch.properties.command.const === 'set_action').properties.action;
  expect(Object.keys(unitreeAction.properties)).toHaveLength(29);
  expect(unitreeAction.properties.waist_pitch_joint).toMatchObject({ minimum: -29.7938, maximum: 29.7938 });
});

test('SO-101 applies only profile-validated partial actions', async () => {
  const { facade, calls } = makeFacade('so101');
  const result = await executeProfileControl(facade, 'so101', {
    command: 'set_action',
    action: { 'shoulder_pan.pos': 12, 'gripper.pos': 30 },
  }, new AbortController().signal, 7);
  expect(calls).toEqual([{ kind: 'action', action: { 'shoulder_pan.pos': 12, 'gripper.pos': 30 } }]);
  expect(result).toMatchObject({
    ok: true,
    profileId: 'so101',
    command: 'set_action',
    hardwareValidated: false,
    appliedAction: { 'shoulder_pan.pos': 12, 'gripper.pos': 30 },
  });

  await expect(executeProfileControl(facade, 'so101', {
    command: 'set_action',
    action: { 'shoulder_pan.pos': 500 },
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'INVALID_ARGUMENT' });
});

test('LeKiwi requires a duration for nonzero base velocity and auto-stops the base', async () => {
  const { facade, calls } = makeFacade('lekiwi');
  await expect(executeProfileControl(facade, 'lekiwi', {
    command: 'set_action',
    action: { 'x.vel': 0.2 },
  }, new AbortController().signal, 7)).rejects.toMatchObject({ code: 'INVALID_ARGUMENT' });

  const result = await executeProfileControl(facade, 'lekiwi', {
    command: 'set_action',
    action: { 'x.vel': 0.2, 'theta.vel': 15 },
    duration_ms: 40,
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
    command: 'set_action',
    action: { waist_pitch_joint: 8, left_elbow_joint: 45 },
  }, new AbortController().signal, 7);
  expect(calls[0]).toEqual({ kind: 'action', action: { waist_pitch_joint: 8, left_elbow_joint: 45 } });
  expect(pose.simulation.telemetry).toMatchObject({ waist_pitch_joint: 8, left_elbow_joint: 45 });

  const reset = await executeProfileControl(facade, 'unitree', { command: 'reset' }, new AbortController().signal, 7);
  expect(state.resetCount).toBe(1);
  expect(reset).toMatchObject({ ok: true, profileId: 'unitree', command: 'reset', reset: true, hardwareValidated: false });
});
