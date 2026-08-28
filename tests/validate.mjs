import fs from 'node:fs';
const root=new URL('../',import.meta.url);
const files=['index.html','styles.css','src/editor.js','src/profiles.js','src/python-runtime.js','src/canonical-rig.js','src/source-simulator.js','src/task-catalog.js','src/task-workspace.js','src/app-v2.js','README.md','.nojekyll'];
for(const f of files){if(!fs.existsSync(new URL(f,root)))throw new Error(`missing ${f}`)}
const index=fs.readFileSync(new URL('index.html',root),'utf8');
for(const token of ['id="editor"','id="simCanvas"','id="taskSelect"','src/app-v2.js','pyodide.js'])if(!index.includes(token))throw new Error(`index missing ${token}`);
if(/blockly/i.test(index))throw new Error('Blockly dependency found');
const all=files.filter(f=>f.endsWith('.js')||f.endsWith('.html')).map(f=>fs.readFileSync(new URL(f,root),'utf8')).join('\n');
for(const fake of ['robot.grasp(','robot.attach(','robot.teleport(','robot.move_to('])if(all.includes(fake))throw new Error(`fake physical method found: ${fake}`);
const css=fs.readFileSync(new URL('styles.css',root),'utf8');let depth=0;for(const c of css){if(c==='{')depth++;if(c==='}')depth--;if(depth<0)throw new Error('CSS braces unbalanced');}if(depth!==0)throw new Error('CSS braces unbalanced');
const profiles=fs.readFileSync(new URL('src/profiles.js',root),'utf8');
for(const token of ['SO101Follower','BiOpenArmFollower','LeKiwiClient','hardware_validation','RoboBuddy_AI@66d18a029a0caeb6a6075e681dbd9ecd6b22affa'])if(!profiles.includes(token))throw new Error(`profiles missing ${token}`);
if(/lightweight articulated browser model|intentionally simplified and is not a production OpenArm mesh/i.test(profiles))throw new Error('obsolete schematic-robot fidelity language remains');
const simulator=fs.readFileSync(new URL('src/source-simulator.js',root),'utf8');
for(const forbidden of ['_armVisual(','_buildSO101(','_buildLeKiwi(','segmentMesh('])if(simulator.includes(forbidden))throw new Error(`procedural placeholder rig remains: ${forbidden}`);
for(const required of ['CanonicalRobotRig','ScenarioV2Engine','engine.plant.sendAction','engine.plant.tick()'])if(!simulator.includes(required))throw new Error(`source simulator missing ${required}`);
console.log('static fidelity checks: OK');
