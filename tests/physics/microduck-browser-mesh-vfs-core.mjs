import { readFileSync } from 'node:fs';

const worker = readFileSync(new URL('../../src/physics/microduck-mujoco-worker.js', import.meta.url), 'utf8');
const walk = readFileSync(new URL('../../models/microduck/walk.xml', import.meta.url), 'utf8');

function assert(condition, message) { if (!condition) throw new Error(message); }

assert(/<compiler[^>]*meshdir="assets"/.test(walk), 'MicroDuck exact-mesh MJCF no longer declares meshdir=assets');
assert((walk.match(/<mesh\b[^>]*\bfile="[^"]+\.stl"/g) || []).length >= 10, 'MicroDuck walk plant no longer carries external STL assets');
assert(/new mj\.MjVFS\(\)/.test(worker), 'MicroDuck worker no longer constructs a MuJoCo VFS');
assert(/vfs\.addBuffer\(`assets\/\$\{file\}`/.test(worker), 'MicroDuck worker no longer mounts meshdir assets into the VFS');
assert(/mj\.MjModel\.from_xml_string\(xml, vfs\)/.test(worker), 'MicroDuck worker compiled XML without the mesh VFS');
assert(/url\.origin !== self\.location\.origin/.test(worker), 'MicroDuck worker lost same-origin dependency enforcement');
assert(/Worker rejected unsafe MicroDuck mesh dependency/.test(worker), 'MicroDuck worker lost dependency path validation');

console.log('MicroDuck browser exact-mesh VFS loader contract: OK');
