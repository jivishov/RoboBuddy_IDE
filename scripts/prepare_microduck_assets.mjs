import { createHash } from 'node:crypto';
import { readFile, readdir, stat, writeFile } from 'node:fs/promises';
import { dirname, relative, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const repo = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const assetRoot = resolve(repo, 'assets/microduck');
const sourcePath = resolve(assetRoot, 'kinematics/robot_walk.xml');
const outputPath = resolve(assetRoot, 'generated/procedural-rig.json');
const manifestPath = resolve(assetRoot, 'manifest.json');
const sourceRevision = '590b986bd8c0d50ae02cb3ea2f59c463b6828168';
const wireJointOrder = Object.freeze([
  'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
  'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth',
  'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
]);
const mouthWireIndex = 9;

const numbers = (value = '') => value.trim().split(/\s+/).filter(Boolean).map(Number);
const attr = (line, name) => line.match(new RegExp(`${name}="([^"]*)"`))?.[1] ?? '';

function extractHierarchy(xml) {
  const bodies = [];
  const joints = [];
  const sites = [];
  const stack = [];
  for (const raw of xml.split(/\r?\n/)) {
    const line = raw.trim();
    if (line.startsWith('<body ')) {
      const body = {
        name: attr(line, 'name'),
        parent: stack.at(-1) || null,
        pos: numbers(attr(line, 'pos') || '0 0 0'),
        quatWxyz: numbers(attr(line, 'quat') || '1 0 0 0'),
        provenance: 'extracted_apache_runtime_xml',
      };
      bodies.push(body);
      stack.push(body.name);
    } else if (line.startsWith('</body')) {
      stack.pop();
    } else if (line.startsWith('<joint ')) {
      joints.push({
        name: attr(line, 'name'), body: stack.at(-1), type: attr(line, 'type'),
        axis: numbers(attr(line, 'axis')), rangeRad: numbers(attr(line, 'range')),
        provenance: 'extracted_apache_runtime_xml',
      });
    } else if (line.startsWith('<inertial ')) {
      const body = bodies.find((item) => item.name === stack.at(-1));
      if (body) body.inertial = {
        pos: numbers(attr(line, 'pos')), massKg: Number(attr(line, 'mass')),
        fullInertia: numbers(attr(line, 'fullinertia')),
        provenance: 'extracted_apache_runtime_xml',
      };
    } else if (line.startsWith('<site ')) {
      sites.push({
        name: attr(line, 'name'), body: stack.at(-1), pos: numbers(attr(line, 'pos')),
        quatWxyz: numbers(attr(line, 'quat') || '1 0 0 0'),
        provenance: 'extracted_apache_runtime_xml',
      });
    }
  }
  if (joints.length !== 14) throw new Error(`Expected 14 hinge joints, found ${joints.length}.`);
  return { bodies, joints, sites };
}

async function walk(directory) {
  const files = [];
  for (const entry of await readdir(directory, { withFileTypes: true })) {
    const path = resolve(directory, entry.name);
    if (entry.isDirectory()) files.push(...await walk(path));
    else if (path !== manifestPath) files.push(path);
  }
  return files;
}

function classify(path) {
  const rel = relative(assetRoot, path).replaceAll('\\', '/');
  if (rel.startsWith('policies/')) return { license: 'Apache-2.0', repository: 'pollen-robotics/microduck', revision: sourceRevision, sourcePath: rel, status: 'exact_source_byte', purpose: 'pinned deployed ONNX policy' };
  if (rel.startsWith('scores/')) return { license: 'Apache-2.0', repository: 'pollen-robotics/microduck', revision: sourceRevision, sourcePath: `sounds/${rel}`, status: 'exact_source_byte', purpose: 'original release-safe score input reserved for Cycle 03 generated browser audio' };
  if (rel === 'kinematics/robot_walk.xml') return { license: 'Apache-2.0', repository: 'pollen-robotics/microduck', revision: sourceRevision, sourcePath: 'kinematics/assets/alpha/robot_walk.xml', status: 'exact_source_byte', purpose: 'fourteen-joint hierarchy, inertials and named frames' };
  if (rel.startsWith('runtime/mujoco/')) return { license: 'Apache-2.0', repository: 'npm:@mujoco/mujoco', revision: '3.11.0', sourcePath: rel.replace('runtime/mujoco/', 'package/'), status: 'exact_distribution_byte', purpose: 'local browser MuJoCo runtime reserved for Cycle 02' };
  if (rel.startsWith('runtime/onnx/')) return { license: 'MIT', repository: 'npm:onnxruntime-web', revision: '1.27.0', sourcePath: rel.replace('runtime/onnx/', 'dist/'), status: 'exact_distribution_byte', purpose: 'local browser ONNX inference runtime' };
  if (rel === 'visual/duck.bin') return { license: 'Apache-2.0', repository: 'pollen-robotics/microduck', revision: sourceRevision, sourcePath: 'robotctl/assets/duck.bin', status: 'exact_source_byte', purpose: 'official compact articulated visual used by the source robotctl monitor' };
  if (rel === 'generated/procedural-rig.json') return { license: 'Apache-2.0 AND PolyForm-Noncommercial-1.0.0', repository: 'local + pollen-robotics/microduck', revision: `post-cycle-visual-repair + ${sourceRevision}`, sourcePath: 'scripts/prepare_microduck_assets.mjs + kinematics/assets/alpha/robot_walk.xml', status: 'deterministic_generated_derivative', purpose: 'Apache-extracted hierarchy plus configured official lower-bill articulation, roller and visual-alignment metadata' };
  if (rel.startsWith('fixtures/')) return { license: 'PolyForm-Noncommercial-1.0.0', repository: 'local', revision: 'cycle01', sourcePath: rel, status: 'recorded_generated_evidence', purpose: 'fixed CPU/browser exact-byte inference comparison' };
  throw new Error(`Unclassified asset: ${rel}`);
}

const xml = await readFile(sourcePath, 'utf8');
const hierarchy = extractHierarchy(xml);
const rig = {
  schema: 'robobuddy.microduck-procedural-rig.v1',
  units: 'metres',
  source: { repository: 'pollen-robotics/microduck', revision: sourceRevision, path: 'kinematics/assets/alpha/robot_walk.xml', license: 'Apache-2.0' },
  fidelity: {
    tier: 'reference-aligned policy demonstrator',
    extracted: ['body hierarchy', 'body transforms', 'body inertials', '14 joint names/axes/ranges', 'named site transforms', 'official compact robotctl monitor visual'],
    configuredApproximation: ['configured articulation of official lower-bill part', 'roller placement and geometry', 'visual floor alignment', 'collision geometry', 'contacts', 'dynamics'],
    excludedEvidence: ['microduck_rl meshes', 'microduck_rl MJCF', 'Hugging Face Space bytes'],
  },
  officialVisual: {
    path: 'visual/duck.bin', repository: 'pollen-robotics/microduck', revision: sourceRevision,
    sourcePath: 'robotctl/assets/duck.bin', license: 'Apache-2.0', format: 'DUCK v1',
    scope: 'exact compact source-monitor mesh, body hierarchy, part transforms and baked colors; two-sided centroid winding reconstruction, rendering normals and Z-up to Y-up conversion are local',
  },
  jointContract: {
    positionUnit: 'metres',
    angleUnit: 'radians',
    positiveDirection: 'right-hand rotation about each source XML joint axis',
    wireJointOrder,
    mouthWireIndex,
    policyJointOrder: wireJointOrder.filter((_name, index) => index !== mouthWireIndex),
    policyActionMapping: 'action[i] maps positionally to policyJointOrder[i]; mouth is not policy-controlled',
    sourcePaths: ['duck-ipc-proto/src/lib.rs', 'duck-control/src/model.rs', 'kinematics/assets/alpha/robot_walk.xml'],
  },
  ...hierarchy,
  primitives: [],
  configuredAttachments: {
    mouth: { parentBody: 'bottom_head_shell', officialPartIndices: [40], pivotM: [-0.006, 0, -0.065], axis: [0, 1, 0], closedRad: 0, openRad: 0.0872664626, provenance: 'exact_official_bill_mesh_with_configured_articulation' },
    rollers: [
      { parentBody: 'ankle_left', offsetM: [0, -0.027, -0.018], radiusM: 0.018, widthM: 0.014, provenance: 'original_configured_approximation' },
      { parentBody: 'ankle_right', offsetM: [0, 0.027, -0.018], radiusM: 0.018, widthM: 0.014, provenance: 'original_configured_approximation' },
    ],
  },
};
await writeFile(outputPath, `${JSON.stringify(rig, null, 2)}\n`);

const visualBytes = await readFile(resolve(assetRoot, 'visual/duck.bin'));
if (visualBytes.subarray(0, 4).toString('ascii') !== 'DUCK' || visualBytes.readUInt32LE(4) !== 1) throw new Error('Unsupported official MicroDuck baked visual.');
const visualContract = {
  path: 'visual/duck.bin', format: 'DUCK v1', meshes: visualBytes.readUInt16LE(8), bodies: visualBytes.readUInt16LE(10), parts: visualBytes.readUInt16LE(12),
  claim: 'exact Apache-2.0 source-monitor visual byte; local two-sided winding reconstruction, normals and coordinate conversion do not establish dynamics, contact, locomotion, RL-environment, or hardware parity',
};

const files = await walk(assetRoot);
const entries = [];
for (const path of files.sort()) {
  const bytes = await readFile(path);
  entries.push({ path: relative(assetRoot, path).replaceAll('\\', '/'), bytes: (await stat(path)).size, sha256: createHash('sha256').update(bytes).digest('hex'), ...classify(path) });
}
const manifest = {
  schema: 'robobuddy.microduck-asset-manifest.v1',
  generatedBy: 'node scripts/prepare_microduck_assets.mjs',
  manifestSelfExcludedFromEntries: true,
  pins: { microduck: sourceRevision, mujoco: '3.11.0', onnxruntimeWeb: '1.27.0' },
  policyContract: { input: [1, 61], output: [1, 14], claim: 'exact-byte deterministic inference only; no RL, MuJoCo, dynamics, or hardware parity' },
  policyPairing: { model: 'none claimed', hierarchy: 'kinematics/robot_walk.xml', fixture: 'fixtures/inference-parity.json', scope: 'policy bytes and inference outputs only' },
  visualContract,
  jointMapping: rig.jointContract,
  scoreContract: { inputs: ['scores/wistful.duckscore', 'scores/duck_strut.mid'], outputAudio: 'none in Cycle 01; Cycle 03 will synthesize locally in the browser', excluded: ['sounds/scores/outer_wilds.mid (upstream explicitly marks it copyrighted/test-only/not for release)'] },
  excludedSources: ['pollen-robotics/microduck_rl mesh/MJCF bytes', 'Hugging Face Space implementation/assets', 'pollen-robotics/microduck sounds/scores/outer_wilds.mid'],
  entries,
};
await writeFile(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
console.log(`wrote ${relative(repo, outputPath)} and ${entries.length}-entry manifest`);
