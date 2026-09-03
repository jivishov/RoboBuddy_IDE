import { MICRODUCK_COMMANDS } from '../microduck/command-catalog.js';
import { boundedMicroDuckResult, isRetainedMicroDuckCommand, parseMicroDuckControlInput } from './microduck-control.js';

const READABLE_FILES = Object.freeze(['main.py', 'trajectories.py', 'robot_config.py', 'workcell.py']);
const MAX_SOURCE_CHARACTERS = 1_200;
const MAX_SOURCE_LINES = 32;
const MAX_VISIBLE_ENTRIES = 12;
const MAX_PROBLEMS = 4;
const MAX_EDIT_SOURCE_CHARACTERS = 900;
const MAX_EDIT_SOURCE_LINES = 12;
const MAX_REPLACEMENT_CHARACTERS = 1_200;
const MAX_REPLACEMENT_LINES = 24;
const MAX_EDIT_EXPLANATION_CHARACTERS = 280;
const CONTROL_ERROR_CODES = new Set([
  'INVALID_ARGUMENT', 'PROFILE_MISMATCH', 'SIMULATION_NOT_READY', 'SIMULATION_BUSY',
  'COMMAND_CONFLICT', 'AUDIO_LOCKED', 'OPERATION_CANCELLED', 'ASSET_UNAVAILABLE', 'POLICY_TIMEOUT',
]);

export class WebMcpDomainError extends Error {
  constructor(code, message, { retryable = false, details = {} } = {}) {
    super(message);
    this.name = 'WebMcpDomainError';
    this.code = code;
    this.retryable = retryable;
    this.details = details;
  }
}

function invalidInput(message) {
  throw new WebMcpDomainError('INVALID_ARGUMENT', message);
}

function assertPlainObject(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    invalidInput('Tool input must be an object.');
  }
}

function assertOnlyKeys(value, allowed) {
  for (const key of Object.keys(value)) {
    if (!allowed.includes(key)) invalidInput(`Unexpected input field: ${key}.`);
  }
}

function boundedText(value, maximum) {
  const text = String(value ?? '');
  return text.length <= maximum ? text : `${text.slice(0, Math.max(0, maximum - 1))}…`;
}

function requireBoundedString(input, key, maximum, label = key) {
  if (typeof input[key] !== 'string' || !input[key].trim()) {
    invalidInput(`${label} must be a non-empty string.`);
  }
  if (input[key].length > maximum) invalidInput(`${label} must not exceed ${maximum} characters.`);
  if (input[key].includes('\u0000')) invalidInput(`${label} cannot contain a null character.`);
  return input[key];
}

function sourceLineIndent(lines) {
  const firstCodeLine = lines.find((line) => line.trim());
  return firstCodeLine?.match(/^[\t ]*/)?.[0] || '';
}

function commentDisabledPython(lines) {
  return lines.map((line) => {
    const indent = line.match(/^[\t ]*/)?.[0] || '';
    return `${indent}# Agent-disabled: ${line.slice(indent.length)}`;
  });
}

function formatTemporaryCooperativeEdit({ lines, startLine, endLine, replacementCode, explanation }) {
  const indent = sourceLineIndent(lines);
  const reason = explanation.replace(/\s+/g, ' ').trim();
  const replacementLines = replacementCode.split('\n');
  const firstReplacementLine = replacementLines.find((line) => line.trim());
  const preservesSelectedIndent = !indent || firstReplacementLine.startsWith(indent);
  const replacement = replacementLines.map((line) => (
    !line || preservesSelectedIndent ? line : `${indent}${line}`
  ));
  return {
    source: [
      `${indent}# Agent cooperative edit: temporary draft; no save was performed.`,
      `${indent}# Explanation: ${reason}`,
      `${indent}# Original code disabled by Agent Assist:`,
      ...commentDisabledPython(lines),
      `${indent}# Working replacement:`,
      ...replacement,
    ].join('\n'),
    workingStartLine: startLine + lines.length + 4,
    replacedLineCount: endLine - startLine + 1,
    addedLineCount: replacement.length,
  };
}

function compactValue(value) {
  if (value === null || typeof value === 'boolean') return value;
  if (typeof value === 'number') return Number.isFinite(value) ? value : null;
  if (typeof value === 'string') return boundedText(value, 180);
  try {
    return boundedText(JSON.stringify(value), 240);
  } catch {
    return '[unavailable]';
  }
}

function compactEntries(value, maximum = MAX_VISIBLE_ENTRIES) {
  const entries = Object.entries(value || {});
  return {
    values: Object.fromEntries(entries.slice(0, maximum).map(([key, item]) => [key, compactValue(item)])),
    truncated: entries.length > maximum,
  };
}

export function cancelledResult() {
  return {
    ok: false,
    error: {
      code: 'OPERATION_CANCELLED',
      message: 'The WebMCP tool call was cancelled.',
      retryable: true,
    },
  };
}

export function domainErrorResult(error) {
  if (error instanceof WebMcpDomainError) {
    return {
      ok: false,
      error: {
        code: error.code,
        message: error.message,
        retryable: error.retryable,
        ...error.details,
      },
    };
  }

  if (CONTROL_ERROR_CODES.has(error?.code)) {
    return {
      ok: false,
      error: {
        code: error.code,
        message: boundedText(error.message, 240),
        retryable: error.code !== 'INVALID_ARGUMENT' && error.code !== 'PROFILE_MISMATCH' && error.code !== 'AUDIO_LOCKED',
      },
    };
  }

  return {
    ok: false,
    error: {
      code: 'INTERNAL_ERROR',
      message: 'RoboBuddy could not complete this WebMCP tool call.',
      retryable: true,
    },
  };
}

export class AgentFacade {
  constructor(app) {
    this.app = app;
    this.registrationEpoch = 0;
    this.controlSequence = 0;
    this.activeControlId = null;
  }

  setRegistrationEpoch(epoch) {
    this.registrationEpoch = Number(epoch);
  }

  assertActive(expectedEpoch) {
    if (expectedEpoch !== this.registrationEpoch) {
      throw new WebMcpDomainError(
        'STALE_REGISTRATION',
        'This WebMCP registration is no longer active.',
        { retryable: true },
      );
    }
    if (this.app.getAgentAccess() !== 'assist') {
      throw new WebMcpDomainError(
        'AGENT_ACCESS_DISABLED',
        'A person must enable Agent Assist in RoboBuddy before using these tools.',
        { retryable: true },
      );
    }
  }

  captureReadySnapshot(expectedEpoch) {
    this.assertActive(expectedEpoch);
    const snapshot = this.app.getAgentSnapshot();
    if (snapshot.workspaceStatus !== 'ready') {
      throw new WebMcpDomainError(
        'WORKSPACE_NOT_READY',
        'The RoboBuddy workspace is not ready for agent-assisted inspection.',
        { retryable: true, details: { workspaceStatus: snapshot.workspaceStatus } },
      );
    }
    return snapshot;
  }

  assertSnapshotCurrent(snapshot, expectedEpoch) {
    this.assertActive(expectedEpoch);
    const current = this.app.getAgentSnapshot();
    if (current.workspaceStatus !== 'ready') {
      throw new WebMcpDomainError(
        'WORKSPACE_NOT_READY',
        'The RoboBuddy workspace is no longer ready.',
        { retryable: true, details: { workspaceStatus: current.workspaceStatus } },
      );
    }
    if (current.workspaceGeneration !== snapshot.workspaceGeneration) {
      throw new WebMcpDomainError(
        'WORKSPACE_CHANGED',
        'The workspace changed while this WebMCP tool was running. Read the current state and try again.',
        { retryable: true },
      );
    }
    return current;
  }

  getRegistrationContext() {
    return this.app.getAgentRegistrationContext();
  }

  shouldRegisterMicroduckControl() {
    const context = this.getRegistrationContext();
    return context.workspaceStatus === 'ready'
      && context.profileId === 'microduck'
      && context.simulationMode === 'policy_sim'
      && context.simulationReady;
  }

  describeTask(snapshot, input = {}) {
    assertPlainObject(input);
    assertOnlyKeys(input, []);
    const sourcePlant = snapshot.simulationMode === 'source_plant';
    const policySimulation = snapshot.simulationMode === 'policy_sim';
    return {
      taskId: snapshot.taskId,
      title: boundedText(snapshot.title, 180),
      robot: boundedText(snapshot.robot, 120),
      brief: boundedText(snapshot.brief, 420),
      files: READABLE_FILES.filter((file) => Object.hasOwn(snapshot.files, file)),
      workspaceGeneration: snapshot.workspaceGeneration,
      simulationMode: snapshot.simulationMode,
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: sourcePlant,
      policySimulationAvailable: policySimulation,
      hardwareValidated: false,
      fidelityBoundaries: policySimulation
        ? [
          'Exact evidence is limited to pinned ONNX 1x61 -> 1x14 inference, the Apache-covered fourteen-joint hierarchy and named frames, and the pinned Apache-2.0 compact robotctl monitor visual.',
          'Configured movement of the exact official lower-bill part, rollers, visual floor alignment, contacts, camera/IMU/ToF/audio, and browser dynamics are modeled approximations; there is no RL-environment, locomotion, contact, or hardware parity claim.',
          'Python and WebMCP control only the bounded browser policy simulation; they expose no physical transport or hardware action.',
        ]
        : sourcePlant
        ? [
          'Contact, collision, and support values are modeled by a source-pinned browser plant; they are not hardware measurements.',
          'Python is replayed as open-loop public API events, not closed-loop hardware control.',
        ]
        : [
          'The Unitree workspace is a browser-held kinematic pose view with no collision, contact, balance, gait, or hardware-control claim.',
          'Python is replayed as open-loop public API events, not closed-loop hardware control.',
        ],
      agentBoundary: 'Tools can inspect, focus, run, and make one bounded temporary cooperative editor draft. The edit comments out exact matched source and never saves, exports, or publishes it.',
    };
  }

  readWorkspace(snapshot, input = {}) {
    assertPlainObject(input);
    assertOnlyKeys(input, ['file', 'start_line']);
    if (!READABLE_FILES.includes(input.file)) {
      invalidInput(`file must be one of: ${READABLE_FILES.join(', ')}.`);
    }
    if (!Object.hasOwn(snapshot.files, input.file)) {
      invalidInput(`${input.file} is not available in the active workspace.`);
    }

    const startLine = input.start_line ?? 1;
    if (!Number.isInteger(startLine) || startLine < 1) {
      invalidInput('start_line must be an integer greater than or equal to 1.');
    }

    const source = String(snapshot.files[input.file]);
    const lines = source.split('\n');
    if (startLine > lines.length) {
      invalidInput(`start_line exceeds the file length of ${lines.length} lines.`);
    }

    let characters = 0;
    let index = startLine - 1;
    let clippedLine = false;
    const content = [];
    while (index < lines.length && content.length < MAX_SOURCE_LINES) {
      const prefix = `${index + 1}: `;
      const remaining = MAX_SOURCE_CHARACTERS - characters - (content.length ? 1 : 0);
      if (remaining <= prefix.length) break;
      let line = `${prefix}${lines[index]}`;
      if (line.length > remaining) {
        line = `${line.slice(0, Math.max(prefix.length, remaining - 1))}…`;
        clippedLine = true;
      }
      content.push(line);
      characters += line.length + (content.length > 1 ? 1 : 0);
      index += 1;
      if (clippedLine) break;
    }

    return {
      file: input.file,
      startLine,
      endLine: index,
      nextLine: index < lines.length ? index + 1 : null,
      lineCount: lines.length,
      workspaceGeneration: snapshot.workspaceGeneration,
      contentClassification: 'untrusted_user_authored_source',
      content: content.join('\n'),
      truncated: clippedLine || index < lines.length,
    };
  }

  inspectSimulation(snapshot, input = {}) {
    assertPlainObject(input);
    assertOnlyKeys(input, []);
    const telemetry = compactEntries(snapshot.simulation.telemetry);
    const contacts = compactEntries(snapshot.simulation.contacts);
    const problems = (snapshot.simulation.problems || []).slice(-MAX_PROBLEMS).map((problem) => ({
      level: boundedText(problem.level, 24),
      code: boundedText(problem.code, 64),
      message: boundedText(problem.message, 220),
    }));
    return {
      workspaceGeneration: snapshot.workspaceGeneration,
      executionState: snapshot.simulation.executionState,
      status: boundedText(snapshot.simulation.status, 240),
      simulationMode: snapshot.simulationMode,
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: snapshot.simulationMode === 'source_plant',
      policySimulationAvailable: snapshot.simulationMode === 'policy_sim',
      hardwareValidated: false,
      telemetry: telemetry.values,
      telemetryTruncated: telemetry.truncated,
      contacts: contacts.values,
      contactsTruncated: contacts.truncated,
      preparedActionCount: snapshot.simulation.preparedActionCount,
      currentFile: snapshot.currentFile,
      recentProblems: problems,
      untrustedContent: true,
    };
  }

  focusWorkspace(snapshot, input = {}) {
    assertPlainObject(input);
    assertOnlyKeys(input, ['file', 'line']);
    if (!READABLE_FILES.includes(input.file) || !Object.hasOwn(snapshot.files, input.file)) {
      invalidInput(`file must be one of the active workspace files: ${READABLE_FILES.join(', ')}.`);
    }
    if (!Number.isInteger(input.line) || input.line < 1) {
      invalidInput('line must be an integer greater than or equal to 1.');
    }
    const lineCount = String(snapshot.files[input.file]).split('\n').length;
    if (input.line > lineCount) invalidInput(`line exceeds the file length of ${lineCount} lines.`);
    this.app.focusAgentWorkspaceLine(input.file, input.line);
    return {
      focused: true,
      file: input.file,
      line: input.line,
      workspaceGeneration: snapshot.workspaceGeneration,
      sourceChanged: false,
    };
  }

  draftWorkspaceEdit(snapshot, input = {}) {
    assertPlainObject(input);
    assertOnlyKeys(input, ['file', 'start_line', 'end_line', 'expected_source', 'replacement_code', 'explanation']);
    if (!READABLE_FILES.includes(input.file) || !Object.hasOwn(snapshot.files, input.file)) {
      invalidInput(`file must be one of the active workspace files: ${READABLE_FILES.join(', ')}.`);
    }
    if (!Number.isInteger(input.start_line) || input.start_line < 1) {
      invalidInput('start_line must be an integer greater than or equal to 1.');
    }
    if (!Number.isInteger(input.end_line) || input.end_line < input.start_line) {
      invalidInput('end_line must be an integer greater than or equal to start_line.');
    }
    if (input.end_line - input.start_line + 1 > MAX_EDIT_SOURCE_LINES) {
      invalidInput(`A cooperative edit may replace at most ${MAX_EDIT_SOURCE_LINES} lines.`);
    }
    const expectedSource = requireBoundedString(input, 'expected_source', MAX_EDIT_SOURCE_CHARACTERS, 'expected_source');
    const replacementCode = requireBoundedString(input, 'replacement_code', MAX_REPLACEMENT_CHARACTERS, 'replacement_code');
    const explanation = requireBoundedString(input, 'explanation', MAX_EDIT_EXPLANATION_CHARACTERS, 'explanation');
    const replacementLines = replacementCode.split('\n');
    if (replacementLines.length > MAX_REPLACEMENT_LINES) {
      invalidInput(`replacement_code may contain at most ${MAX_REPLACEMENT_LINES} lines.`);
    }
    if (this.app.getExecutionState() !== 'idle') {
      throw new WebMcpDomainError(
        'SIMULATION_BUSY',
        'Stop or wait for the active simulation before drafting an editor edit.',
        { retryable: true },
      );
    }

    const sourceLines = String(snapshot.files[input.file]).split('\n');
    if (input.end_line > sourceLines.length) {
      invalidInput(`end_line exceeds the file length of ${sourceLines.length} lines.`);
    }
    const originalLines = sourceLines.slice(input.start_line - 1, input.end_line);
    const originalSource = originalLines.join('\n');
    if (originalSource.length > MAX_EDIT_SOURCE_CHARACTERS) {
      invalidInput(`The selected source must not exceed ${MAX_EDIT_SOURCE_CHARACTERS} characters.`);
    }
    if (expectedSource !== originalSource) {
      throw new WebMcpDomainError(
        'SOURCE_MISMATCH',
        'The selected source no longer matches expected_source. Read the current workspace and retry.',
        { retryable: true },
      );
    }

    const edit = formatTemporaryCooperativeEdit({
      lines: originalLines,
      startLine: input.start_line,
      endLine: input.end_line,
      replacementCode,
      explanation,
    });
    const applied = this.app.applyTemporaryAgentWorkspaceEdit({
      file: input.file,
      startLine: input.start_line,
      endLine: input.end_line,
      replacement: edit.source,
      workingStartLine: edit.workingStartLine,
    });
    if (!applied) {
      throw new WebMcpDomainError(
        'WORKSPACE_NOT_READY',
        'The RoboBuddy workspace is no longer ready for an editor edit.',
        { retryable: true },
      );
    }
    return {
      sourceChanged: true,
      temporary: true,
      persistence: 'not_saved_refresh_reloads_workspace',
      file: input.file,
      disabledOriginal: true,
      startLine: input.start_line,
      endLine: input.end_line,
      workingStartLine: applied.workingStartLine,
      replacedLineCount: edit.replacedLineCount,
      addedLineCount: edit.addedLineCount,
      workspaceGeneration: applied.workspaceGeneration,
    };
  }

  async runProgram(snapshot, input = {}, signal) {
    assertPlainObject(input);
    assertOnlyKeys(input, []);
    if (this.app.getExecutionState() !== 'idle') {
      throw new WebMcpDomainError(
        'SIMULATION_BUSY',
        'Wait for the active RoboBuddy simulation to finish before starting another run.',
        { retryable: true },
      );
    }

    let aborted = Boolean(signal?.aborted);
    const stopOnAbort = () => {
      aborted = true;
      this.app.stop();
    };
    signal?.addEventListener('abort', stopOnAbort, { once: true });
    try {
      const completed = await this.app.run();
      if (aborted || signal?.aborted) return cancelledResult();
      const after = this.app.getAgentSnapshot();
      return {
        completed: Boolean(completed),
        workspaceGeneration: snapshot.workspaceGeneration,
        simulation: after.workspaceStatus === 'ready'
          ? this.inspectSimulation(after)
          : { workspaceStatus: after.workspaceStatus },
      };
    } finally {
      signal?.removeEventListener('abort', stopOnAbort);
    }
  }

  controlError(code, message) {
    const error = new Error(message);
    error.code = code;
    return error;
  }

  captureMicroduckControlSnapshot(expectedEpoch) {
    try {
      this.assertActive(expectedEpoch);
    } catch {
      throw this.controlError('OPERATION_CANCELLED', 'Agent Assist or this registration is no longer active.');
    }
    const snapshot = this.app.getAgentSnapshot();
    if (snapshot.workspaceStatus !== 'ready') {
      throw this.controlError('SIMULATION_NOT_READY', 'The MicroDuck policy simulation workspace is not ready.');
    }
    if (snapshot.profileId !== 'microduck' || snapshot.simulationMode !== 'policy_sim') {
      throw this.controlError('PROFILE_MISMATCH', 'control_microduck_simulation requires the active MicroDuck policy_sim workspace.');
    }
    if (!this.app.isAgentMicroduckSimulationReady()) {
      throw this.controlError('SIMULATION_NOT_READY', 'The MicroDuck policy simulation backend is not ready.');
    }
    if (this.app.getExecutionState() !== 'idle') {
      throw this.controlError('SIMULATION_BUSY', 'Stop or finish the active Python run before using WebMCP simulation control.');
    }
    return snapshot;
  }

  assertMicroduckControlCurrent(snapshot, expectedEpoch, signal) {
    if (signal?.aborted) throw this.controlError('OPERATION_CANCELLED', 'The WebMCP control call was cancelled.');
    try {
      this.assertActive(expectedEpoch);
    } catch {
      throw this.controlError('OPERATION_CANCELLED', 'Agent Assist or this registration changed during the control call.');
    }
    const current = this.app.getAgentSnapshot();
    if (current.workspaceStatus !== 'ready'
      || current.workspaceGeneration !== snapshot.workspaceGeneration
      || current.profileId !== snapshot.profileId
      || current.simulatorEpoch !== snapshot.simulatorEpoch
      || current.simulationMode !== 'policy_sim'
      || !this.app.isAgentMicroduckSimulationReady()) {
      throw this.controlError('OPERATION_CANCELLED', 'The profile, workspace, readiness, or backend epoch changed during the control call.');
    }
    if (this.app.getExecutionState() !== 'idle') {
      throw this.controlError('SIMULATION_BUSY', 'A Python run acquired the simulation while the WebMCP control call was active.');
    }
    return current;
  }

  async waitWithControlGuards({ snapshot, expectedEpoch, signal, until, timeoutMs, onTimeout }) {
    const started = performance.now();
    while (!until()) {
      this.assertMicroduckControlCurrent(snapshot, expectedEpoch, signal);
      if (performance.now() - started >= timeoutMs) {
        onTimeout?.();
        throw this.controlError('POLICY_TIMEOUT', `The MicroDuck command exceeded its ${timeoutMs} ms application ceiling.`);
      }
      await new Promise((resolve) => setTimeout(resolve, 20));
    }
    this.assertMicroduckControlCurrent(snapshot, expectedEpoch, signal);
  }

  async awaitControlOperation(operation, context) {
    let settled = false;
    let value;
    let failure;
    Promise.resolve(operation).then(
      (result) => { settled = true; value = result; },
      (error) => { settled = true; failure = error; },
    );
    await this.waitWithControlGuards({ ...context, until: () => settled });
    if (failure) throw failure;
    return value;
  }

  async controlMicroduck(input, signal, expectedEpoch) {
    const parsed = parseMicroDuckControlInput(input);
    const snapshot = this.captureMicroduckControlSnapshot(expectedEpoch);
    const definition = MICRODUCK_COMMANDS[parsed.command];
    const controllerId = `webmcp-${expectedEpoch}-${++this.controlSequence}`;
    if (this.activeControlId) {
      throw this.controlError('COMMAND_CONFLICT', 'Another bounded WebMCP MicroDuck control call is still active.');
    }
    this.activeControlId = controllerId;
    const abortOwned = () => this.app.abortAgentMicroduckCommand(parsed.command, controllerId);
    const abortListener = () => abortOwned();
    signal?.addEventListener('abort', abortListener, { once: true });
    try {
      this.assertMicroduckControlCurrent(snapshot, expectedEpoch, signal);
      const ceiling = Math.min(8000, Math.max(20, definition.timeoutMs || parsed.durationMs || 8000));
      const dispatchedAt = performance.now();
      const result = await this.awaitControlOperation(
        this.app.executeAgentMicroduckCommand(parsed.command, parsed.args, {
          source: 'webmcp',
          controllerId,
          durationMs: parsed.durationMs,
        }),
        { snapshot, expectedEpoch, signal, timeoutMs: ceiling, onTimeout: abortOwned },
      );

      if (isRetainedMicroDuckCommand(parsed.command, parsed.args)) {
        const deadline = dispatchedAt + parsed.durationMs;
        await this.waitWithControlGuards({
          snapshot,
          expectedEpoch,
          signal,
          timeoutMs: parsed.durationMs + 100,
          until: () => {
            if (performance.now() >= deadline) return true;
            if (!this.app.isAgentMicroduckControllerActive(controllerId)) {
              throw this.controlError('OPERATION_CANCELLED', 'A higher-priority controller preempted the WebMCP command lease.');
            }
            return false;
          },
          onTimeout: abortOwned,
        });
        abortOwned();
      } else if (result.completed === false) {
        await this.waitWithControlGuards({
          snapshot,
          expectedEpoch,
          signal,
          timeoutMs: Math.min(8000, definition.timeoutMs || 8000),
          until: () => this.app.isAgentMicroduckCommandComplete(parsed.command, controllerId),
          onTimeout: abortOwned,
        });
      }

      this.assertMicroduckControlCurrent(snapshot, expectedEpoch, signal);
      const state = this.app.getAgentMicroduckState();
      const requested = parsed.durationMs === undefined
        ? result.requested
        : { ...(result.requested || parsed.args), duration_ms: parsed.durationMs };
      return boundedMicroDuckResult(parsed.command, { ...result, requested, completed: true, state, audio: state?.audio });
    } catch (error) {
      abortOwned();
      if (CONTROL_ERROR_CODES.has(error?.code)) throw error;
      throw this.controlError('SIMULATION_NOT_READY', 'The MicroDuck control call could not complete safely in the current browser simulation.');
    } finally {
      signal?.removeEventListener('abort', abortListener);
      if (this.activeControlId === controllerId) this.activeControlId = null;
    }
  }
}

export const WEBMCP_TOOL_LIMITS = Object.freeze({
  readableFiles: READABLE_FILES,
  sourceCharacters: MAX_SOURCE_CHARACTERS,
  sourceLines: MAX_SOURCE_LINES,
  editSourceCharacters: MAX_EDIT_SOURCE_CHARACTERS,
  editSourceLines: MAX_EDIT_SOURCE_LINES,
  replacementCharacters: MAX_REPLACEMENT_CHARACTERS,
  replacementLines: MAX_REPLACEMENT_LINES,
});
