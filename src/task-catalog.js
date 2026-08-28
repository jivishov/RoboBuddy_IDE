export const TASK_PATCH_REVISION = '75fe2669c0ab0b029986de424c69162071174df8';
export const TASK_PATCH_SOURCE = 'jivishov/RoboBuddy_AI';

const ROOT = `https://cdn.jsdelivr.net/gh/${TASK_PATCH_SOURCE}@${TASK_PATCH_REVISION}/missions/lab-assistant/v2/definitions`;

const task = (profileId, family, file, id, title, robotId) => Object.freeze({
  profileId, family, file, id, title, robotId,
  url: `${ROOT}/${family}/${file}`,
});

export const PATCH_TASKS = Object.freeze({
  openarm: Object.freeze([
    task('openarm', 'openarm', 'openarm-04-filtration-workcell.json', 'openarm-04-filtration-workcell', 'Bimanual Heater and Ring-Stand Stack', 'openarm_v2_bimanual'),
  ]),
  so101: Object.freeze([
    task('so101', 'so101', 'so101-v2-06-quantitative-transfer.json', 'so101-v2-06-quantitative-transfer', 'Measured Two-Bottle Transfer Workcell', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-08-burette-initial-reading.json', 'so101-v2-08-burette-initial-reading', 'Burette Receiver Clearance Calibration', 'so101_follower'),
    task('so101', 'so101', 'so101-v2-09-vacuum-filtration.json', 'so101-v2-09-vacuum-filtration', 'Vacuum Workcell Keep-Clear Preflight', 'so101_follower'),
  ]),
  lekiwi: Object.freeze([]),
});

const cache = new Map();

export function tasksForProfile(profileId) {
  return PATCH_TASKS[profileId] || [];
}

export function defaultTaskId(profileId) {
  return tasksForProfile(profileId)[0]?.id || '';
}

export function taskDescriptor(profileId, taskId) {
  return tasksForProfile(profileId).find((item) => item.id === taskId) || tasksForProfile(profileId)[0] || null;
}

export async function loadPatchedScenario(profileId, taskId) {
  const descriptor = taskDescriptor(profileId, taskId);
  if (!descriptor) return null;
  if (cache.has(descriptor.id)) return structuredClone(cache.get(descriptor.id));
  const response = await fetch(descriptor.url, { cache: 'force-cache' });
  if (!response.ok) throw new Error(`Pinned task ${descriptor.id} returned HTTP ${response.status}.`);
  const scenario = await response.json();
  if (scenario?.schema !== 'robobuddy.lab-scenario.v2') throw new Error(`${descriptor.id}: unexpected scenario schema.`);
  if (scenario.id !== descriptor.id) throw new Error(`${descriptor.id}: pinned task id mismatch (${scenario.id || 'missing'}).`);
  if (scenario.robotId !== descriptor.robotId) throw new Error(`${descriptor.id}: robot id mismatch (${scenario.robotId || 'missing'}).`);
  if (scenario.title !== descriptor.title) throw new Error(`${descriptor.id}: reviewed title mismatch; refusing silent task drift.`);
  const actions = scenario?.portablePython?.referenceActions;
  if (!Array.isArray(actions) || actions.length < 2) throw new Error(`${descriptor.id}: reviewed portablePython.referenceActions are missing.`);
  for (const [index, item] of actions.entries()) {
    if (!item || typeof item.action !== 'object' || Array.isArray(item.action)) throw new Error(`${descriptor.id}: reference action ${index + 1} is invalid.`);
    if (!Number.isFinite(Number(item.hold_seconds))) throw new Error(`${descriptor.id}: reference action ${index + 1} has no finite hold_seconds.`);
  }
  cache.set(descriptor.id, scenario);
  return structuredClone(scenario);
}

export function taskPatchProvenance(descriptor) {
  return descriptor ? {
    repository: TASK_PATCH_SOURCE,
    revision: TASK_PATCH_REVISION,
    scenarioId: descriptor.id,
    source: descriptor.url,
  } : null;
}
