import { cancelledResult, domainErrorResult } from './agent-facade.js';
import { createMicroDuckControlSchema } from './microduck-control.js';
import { createMicroduckVisualCueSchema } from './microduck-visual-cues.js';
import { executeProfileControl, getProfileControlDefinition } from './robot-controls.js';

const READ_ONLY_ANNOTATIONS = Object.freeze({
  readOnlyHint: true,
  untrustedContentHint: true,
});

const UI_ONLY_ANNOTATIONS = Object.freeze({
  readOnlyHint: false,
  untrustedContentHint: true,
});

const RUN_ANNOTATIONS = Object.freeze({
  readOnlyHint: false,
  untrustedContentHint: true,
});

function wasAborted(signal) {
  return Boolean(signal?.aborted);
}

function safeHandler(operation) {
  return async (input = {}, execution = {}) => {
    try {
      if (wasAborted(execution.signal)) return cancelledResult();
      const result = await operation(input, execution.signal);
      return wasAborted(execution.signal) ? cancelledResult() : result;
    } catch (error) {
      return domainErrorResult(error);
    }
  };
}

function withReadyWorkspace(facade, epoch, operation) {
  return safeHandler(async (input, signal) => {
    const snapshot = facade.captureReadySnapshot(epoch);
    const result = await operation(snapshot, input, signal);
    if (wasAborted(signal)) return cancelledResult();
    facade.assertSnapshotCurrent(snapshot, epoch);
    return result;
  });
}

function withReadyWorkspaceMutation(facade, epoch, operation) {
  return safeHandler((input, signal) => {
    const snapshot = facade.captureReadySnapshot(epoch);
    if (wasAborted(signal)) return cancelledResult();
    // This synchronous operation intentionally increments workspaceGeneration.
    // Its expected-source comparison is the stale-write guard, so do not apply
    // the read-only post-operation snapshot check here.
    return operation(snapshot, input, signal);
  });
}

function createTools(facade, epoch) {
  const tools = [
    {
      name: 'describe_robobuddy_task',
      title: 'Describe RoboBuddy task',
      description: 'Describe the active RoboBuddy task, its available workspace files, and its explicit simulation fidelity boundaries.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: READ_ONLY_ANNOTATIONS,
      execute: withReadyWorkspace(facade, epoch, (snapshot, input) => facade.describeTask(snapshot, input)),
    },
    {
      name: 'read_robobuddy_workspace',
      title: 'Read RoboBuddy workspace',
      description: 'Read one bounded, line-numbered page from an explicitly selected active RoboBuddy Python workspace file.',
      inputSchema: {
        type: 'object',
        properties: {
          file: { type: 'string', enum: ['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py'] },
          start_line: { type: 'integer', minimum: 1 },
        },
        required: ['file'],
        additionalProperties: false,
      },
      annotations: READ_ONLY_ANNOTATIONS,
      execute: withReadyWorkspace(facade, epoch, (snapshot, input) => facade.readWorkspace(snapshot, input)),
    },
    {
      name: 'inspect_robobuddy_simulation',
      title: 'Inspect RoboBuddy simulation',
      description: 'Read the current visible RoboBuddy simulation status, compact modeled telemetry and contacts, and recent diagnostics.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: READ_ONLY_ANNOTATIONS,
      execute: withReadyWorkspace(facade, epoch, (snapshot, input) => facade.inspectSimulation(snapshot, input)),
    },
    {
      name: 'focus_robobuddy_workspace',
      title: 'Focus RoboBuddy workspace location',
      description: 'Focus a selected source line in the visible RoboBuddy editor for shared human-agent review. This never changes source.',
      inputSchema: {
        type: 'object',
        properties: {
          file: { type: 'string', enum: ['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py'] },
          line: { type: 'integer', minimum: 1 },
        },
        required: ['file', 'line'],
        additionalProperties: false,
      },
      annotations: UI_ONLY_ANNOTATIONS,
      execute: withReadyWorkspace(facade, epoch, (snapshot, input) => facade.focusWorkspace(snapshot, input)),
    },
    {
      name: 'run_robobuddy_program',
      title: 'Run RoboBuddy program',
      description: 'Reset and run the current visible RoboBuddy Python draft through its modeled simulation, then return a compact result. This never writes, saves, exports, or publishes source.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: RUN_ANNOTATIONS,
      execute: withReadyWorkspace(facade, epoch, (snapshot, input, signal) => facade.runProgram(snapshot, input, signal)),
    },
    {
      name: 'draft_robobuddy_cooperative_edit',
      title: 'Draft temporary cooperative editor fix',
      description: 'Temporarily replace one small, exact-match Python selection in the visible editor. RoboBuddy comments out the selected original code, adds the working replacement and explanation, does not save it, and reloads the workspace on refresh. replacement_code may use relative indentation or preserve the selected indentation. This is an in-memory collaboration draft only: it cannot save, export, publish, or edit outside the active four-file workspace.',
      inputSchema: {
        type: 'object',
        properties: {
          file: { type: 'string', enum: ['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py'] },
          start_line: { type: 'integer', minimum: 1 },
          end_line: { type: 'integer', minimum: 1 },
          expected_source: { type: 'string', minLength: 1, maxLength: 900 },
          replacement_code: { type: 'string', minLength: 1, maxLength: 1200 },
          explanation: { type: 'string', minLength: 1, maxLength: 280 },
        },
        required: ['file', 'start_line', 'end_line', 'expected_source', 'replacement_code', 'explanation'],
        additionalProperties: false,
      },
      annotations: UI_ONLY_ANNOTATIONS,
      execute: withReadyWorkspaceMutation(facade, epoch, (snapshot, input) => facade.draftWorkspaceEdit(snapshot, input)),
    },
  ];

  const directControl = getProfileControlDefinition(facade);
  if (directControl) {
    const { profileId, ...toolDefinition } = directControl;
    tools.push({
      ...toolDefinition,
      annotations: RUN_ANNOTATIONS,
      execute: safeHandler((input, signal) => executeProfileControl(facade, profileId, input, signal, epoch)),
    });
  }

  if (facade.shouldRegisterMicroduckControl()) {
    tools.push({
      name: 'control_microduck_simulation',
      title: 'Control MicroDuck browser simulation',
      description: 'Apply one catalog-bounded command to the active ready MicroDuck policy simulation. This controls approximate browser dynamics only and exposes no source write, hardware, network, media transport, BLE, multiplayer, device administration, shutdown, save, export, publish, or hidden reference-data surface.',
      inputSchema: createMicroDuckControlSchema(),
      annotations: RUN_ANNOTATIONS,
      execute: safeHandler((input, signal) => facade.controlMicroduck(input, signal, epoch)),
    });
    tools.push({
      name: 'manage_microduck_visual_cues',
      title: 'Manage MicroDuck visual cues',
      description: 'Create, update, remove, clear, or inspect a small bounded set of visible scene cues. Supported declarative cue primitives are labels, markers, lines, and rulers in configured world metres or attached to the modeled duck or ball. This changes only the current browser view; it never executes caller code, edits source, saves, exports, publishes, controls hardware, or changes simulation state.',
      inputSchema: createMicroduckVisualCueSchema(),
      annotations: UI_ONLY_ANNOTATIONS,
      execute: safeHandler((input, signal) => facade.manageMicroduckVisualCues(input, signal, epoch)),
    });
  }
  return tools;
}

function webMcpAvailable() {
  return typeof document?.modelContext?.registerTool === 'function';
}

export function createWebMcpRegistration(facade, { onRegistrationChange = () => {} } = {}) {
  let epoch = 0;
  let registrationController = null;
  let currentAccess = 'off';

  const publish = (registered = false, error = false, pending = false) => {
    onRegistrationChange({
      available: webMcpAvailable(),
      registered,
      error,
      pending,
    });
  };

  async function setAccess(access) {
    currentAccess = access;
    epoch += 1;
    const currentEpoch = epoch;
    facade.setRegistrationEpoch(currentEpoch);
    registrationController?.abort();
    registrationController = null;

    if (access !== 'assist' || !webMcpAvailable()) {
      publish(false, false, false);
      return;
    }

    const controller = new AbortController();
    registrationController = controller;
    publish(false, false, true);

    try {
      for (const tool of createTools(facade, currentEpoch)) {
        // Keep this direct top-level registration visible to WebMCP tooling and
        // to the competition's repository review requirements.
        await document.modelContext.registerTool(tool, { signal: controller.signal });
        if (controller.signal.aborted || currentEpoch !== epoch) return;
      }
      if (registrationController === controller && !controller.signal.aborted) {
        publish(true, false, false);
      }
    } catch {
      controller.abort();
      if (registrationController === controller && currentEpoch === epoch) {
        registrationController = null;
        publish(false, true, false);
      }
    }
  }

  publish(false, false, false);
  return Object.freeze({
    setAccess,
    reconcile() { return setAccess(currentAccess); },
    dispose() {
      epoch += 1;
      facade.setRegistrationEpoch(epoch);
      registrationController?.abort();
      registrationController = null;
      publish(false, false, false);
    },
    getEpoch: () => epoch,
  });
}
