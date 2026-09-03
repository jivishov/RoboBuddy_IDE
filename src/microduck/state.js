import { deepFreeze } from './contract.js';

function boundedArray(values, length) {
  const source = Array.from(values || [], (value) => Number.isFinite(Number(value)) ? Number(value) : 0).slice(0, length);
  while (source.length < length) source.push(0);
  return Object.freeze(source);
}

export function createMicroDuckState(input = {}) {
  const state = {
    simulationMode: 'policy_sim',
    stateKind: 'browser_policy_sim',
    sourcePlantAvailable: false,
    policySimulationAvailable: true,
    hardwareValidated: false,
    rlEnvironmentParity: false,
    dynamicsFidelity: 'original configured approximate browser dynamics',
    time: Math.max(0, Number(input.time) || 0),
    enabled: Boolean(input.enabled),
    actuationEnabled: input.actuationEnabled !== false,
    lifecycle: String(input.lifecycle || 'loading'),
    mode: input.mode === 'roller' ? 'roller' : 'walking',
    movement: {
      requested: boundedArray(input.movement?.requested, 3),
      applied: boundedArray(input.movement?.applied, 3),
      limitedBy: Object.freeze(Array.from(input.movement?.limitedBy || [], String).slice(0, 8)),
    },
    head: boundedArray(input.head, 4),
    body: boundedArray(input.body, 6),
    mouth: Math.max(0, Math.min(1, Number(input.mouth) || 0)),
    activePolicy: String(input.activePolicy || 'stand'),
    phase: String(input.phase || 'idle'),
    safety: { recovery: String(input.safety?.recovery || 'none'), fallen: Boolean(input.safety?.fallen), resetFallback: Boolean(input.safety?.resetFallback) },
    loop: { rateHz: Number(input.loop?.rateHz) || 0, missedTicks: Math.max(0, Number(input.loop?.missedTicks) || 0), inferenceInFlight: Boolean(input.loop?.inferenceInFlight) },
    joints: boundedArray(input.joints, 14),
    targets: boundedArray(input.targets, 14),
    simulatedPose: { position: boundedArray(input.simulatedPose?.position, 3), quaternion: boundedArray(input.simulatedPose?.quaternion || [1, 0, 0, 0], 4) },
    contacts: { count: Math.max(0, Number(input.contacts?.count) || 0), ballContact: Boolean(input.contacts?.ballContact), model: 'MuJoCo contact over original primitive approximations' },
    ball: { position: boundedArray(input.ball?.position, 3), velocity: boundedArray(input.ball?.velocity, 3), attached: false },
    visualCues: { count: Math.max(0, Math.min(12, Number(input.visualCues?.count) || 0)) },
    virtualCamera: {
      mode: String(input.virtualCamera?.mode || 'orbit'),
      name: String(input.virtualCamera?.name || 'Overview'),
      purpose: String(input.virtualCamera?.purpose || 'Inspect the robot and modeled workcell context.'),
      frame: String(input.virtualCamera?.frame || 'world'),
      inset: false,
      modeled: true,
      transport: 'rendered simulation imagery only; no hardware video or media transport',
    },
    imu: { trunk: { frame: String(input.imu?.trunk?.frame || 'imu'), gyro: boundedArray(input.imu?.trunk?.gyro, 3), projectedGravity: boundedArray(input.imu?.trunk?.projectedGravity || [0, 0, -1], 3), modeled: true }, head: { frame: String(input.imu?.head?.frame || 'head_imu'), gyro: boundedArray(input.imu?.head?.gyro, 3), modeled: true } },
    tof: { rows: 8, cols: 8, valuesM: boundedArray(input.tof?.valuesM, 64), minimumM: Number(input.tof?.minimumM) || 0, usable: Math.max(0, Math.min(64, Number(input.tof?.usable) || 0)), source: String(input.tof?.source || 'synthetic'), frame: String(input.tof?.frame || 'tof'), modeled: true },
    audio: { unlocked: Boolean(input.audio?.unlocked), sound: input.audio?.sound || null, theremin: Boolean(input.audio?.theremin), thereminFrequencyHz: Number(input.audio?.thereminFrequencyHz) || 0, thereminHeld: Boolean(input.audio?.thereminHeld), chorale: Boolean(input.audio?.chorale), piece: input.audio?.piece || null, voices: Math.max(0, Math.min(4, Number(input.audio?.voices) || 0)), implementation: String(input.audio?.implementation || 'deterministic local Web Audio synthesis') },
    color: String(input.color || 'cream'),
  };
  return deepFreeze(state);
}
