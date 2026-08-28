import { IdeEditor } from './editor.js';
import { PythonRuntime } from './python-runtime.js';
import { RobotSimulator } from './simulator.js';
import { PROFILES, cloneStarter, validateAction, FIDELITY_NOTICE, LEROBOT_REVISION } from './profiles.js';

const $=id=>document.getElementById(id);const sleep=ms=>new Promise(r=>setTimeout(r,ms));
class App{
  constructor(){
    this.profileId=localStorage.getItem('rbide.profile')||'openarm';if(!PROFILES[this.profileId])this.profileId='openarm';
    this.files={};this.currentFile='main.py';this.dirty=new Set();this.prepared=null;this.stepIndex=0;this.runToken=0;this.problems=[];this.commands=[];this.console={stdout:'',stderr:''};
    this.runtime=new PythonRuntime();this.sim=new RobotSimulator($('simCanvas'));
    this.editor=new IdeEditor($('editor'),{onChange:(f,v)=>this.onEdit(f,v),onSave:()=>this.save(),onRun:()=>this.run(),onCommandPalette:()=>this.openPalette(),onCursor:(f,l,c)=>$('statusCursor').textContent=`${f}:${l}:${c}`});
    this.bind();this.loadProfile(this.profileId,{preserve:false});this.setStatus('Ready');
  }
  storageKey(){return`rbide.workspace.${this.profileId}`;}
  loadStored(){try{const raw=localStorage.getItem(this.storageKey());return raw?JSON.parse(raw):null;}catch{return null;}}
  loadProfile(id,{preserve=true}={}){
    if(preserve)this.save(false);this.profileId=id;localStorage.setItem('rbide.profile',id);this.files=this.loadStored()||cloneStarter(id);this.currentFile='main.py';this.dirty.clear();this.prepared=null;this.stepIndex=0;this.problems=[];this.commands=[];this.console={stdout:'',stderr:''};
    const p=PROFILES[id];$('robotSelect').value=id;$('robotLabel').textContent=p.label;$('driverLabel').textContent=p.driver;$('driverStatus').textContent=p.driver;this.sim.reset(id,p.rest);this.sim.fit();this.renderFiles();this.openFile('main.py');this.renderTask();this.renderPanels();this.setStatus('Ready');
  }
  onEdit(file,value){this.files[file]=value;this.dirty.add(file);this.prepared=null;$('dirtyDot').hidden=false;this.renderFiles();}
  openFile(name){if(!Object.hasOwn(this.files,name))return;this.currentFile=name;this.editor.setFile(name,this.files[name]);this.renderFiles();this.renderTabs();}
  renderFiles(){const el=$('filesList');el.innerHTML='';Object.keys(this.files).forEach(name=>{const b=document.createElement('button');b.className='file-item'+(name===this.currentFile?' active':'');b.innerHTML=`<span class="py-icon">PY</span><span>${name}</span>${this.dirty.has(name)?'<span>●</span>':''}`;b.onclick=()=>this.openFile(name);el.appendChild(b);});this.renderTabs();}
  renderTabs(){const el=$('editorTabs');el.innerHTML='';Object.keys(this.files).forEach(name=>{const b=document.createElement('button');b.className='editor-tab'+(name===this.currentFile?' active':'');b.textContent=name+(this.dirty.has(name)?' ●':'');b.onclick=()=>this.openFile(name);el.appendChild(b);});}
  renderTask(){const p=PROFILES[this.profileId];$('taskPanel').innerHTML=`<h2>${p.task.title}</h2><p>${p.source}</p><ol>${p.task.steps.map((s,i)=>`<li class="${i===0?'task-current':''}">${s}</li>`).join('')}</ol><details><summary>Fidelity boundary</summary><p>${p.task.limitations}</p></details>`;$('fidelityText').textContent=`${FIDELITY_NOTICE} LeRobot revision ${LEROBOT_REVISION}. ${p.task.limitations}`;}
  save(show=true){localStorage.setItem(this.storageKey(),JSON.stringify(this.files));this.dirty.clear();$('dirtyDot').hidden=true;this.renderFiles();if(show)this.setStatus('Draft saved locally');}
  resetWorkspace(){if(!confirm('Reset all files for this robot profile to the starter workspace?'))return;localStorage.removeItem(this.storageKey());this.files=cloneStarter(this.profileId);this.dirty.clear();this.openFile('main.py');this.prepared=null;this.resetSimulation();this.renderFiles();}
  async prepare(){
    this.setStatus('Preparing Python…');this.problems=[];this.commands=[];this.renderPanels();
    let result;try{result=await this.runtime.compileWorkspace(this.files);}catch(e){this.problem('error','PYODIDE',e.message);this.setStatus('Python runtime error');throw e;}
    this.console={stdout:result.stdout||'',stderr:result.stderr||''};if(result.exception){this.problem('error','PYTHON',result.exception);this.setStatus('Python error');this.renderPanels();throw new Error('Python execution failed');}
    const events=[];for(const ev of result.events||[]){if(ev.kind==='send_action'){try{ev.action=validateAction(this.profileId,ev.action);}catch(e){this.problem('error','ACTION',`${ev.file}:${ev.line} — ${e.message}`);this.setStatus('Action rejected');this.renderPanels();throw e;}}events.push(ev);}
    this.prepared={events};this.stepIndex=0;this.commands=events.filter(e=>e.kind==='send_action');this.renderPanels();this.setStatus(`${this.commands.length} physical actions prepared`);return this.prepared;
  }
  resetSimulation({cancel=true}={}){if(cancel)this.runToken++;this.sim.reset(this.profileId,PROFILES[this.profileId].rest);this.sim.fit();this.editor.highlightLine(null);this.stepIndex=0;$('simActionLabel').textContent='Ready';this.renderPanels();this.setStatus('Simulation reset');}
  async _applyEvent(ev,token,{honorSleep=true}={}){
    if(token!==this.runToken)return false;
    if(ev.kind==='send_action'){
      if(this.currentFile===ev.file)this.editor.highlightLine(ev.line);$('simActionLabel').textContent=`${ev.file}:${ev.line} · send_action()`;try{await this.sim.applyAction(ev.action,{duration:.22});}catch(e){this.problem('error','COLLISION',`${ev.file}:${ev.line} — ${e.message}`);this.setStatus('Modeled motion rejected');throw e;}this.renderPanels();
    }else if(ev.kind==='sleep'&&honorSleep){if(this.profileId==='lekiwi')this.sim.advanceBase(ev.seconds);await sleep(Math.min(1000,ev.seconds*450));}
    return token===this.runToken;
  }
  async run(){
    const token=++this.runToken;this.resetSimulation({cancel:false});let prep;try{prep=await this.prepare();}catch{return;}this.stepIndex=0;this.setStatus('Running simulation…');
    try{for(let i=0;i<prep.events.length;i++){if(!(await this._applyEvent(prep.events[i],token)))return;this.stepIndex=i+1;}}catch{return;}
    this.editor.highlightLine(null);$('simActionLabel').textContent='Run complete';this.setStatus('Run complete');this.renderPanels();
  }
  async step(){
    if(!this.prepared){this.resetSimulation();try{await this.prepare();}catch{return;}}
    const events=this.prepared.events;while(this.stepIndex<events.length){const ev=events[this.stepIndex++];if(ev.kind!=='send_action')continue;const token=this.runToken;try{await this._applyEvent(ev,token,{honorSleep:false});}catch{return;}this.setStatus(`Stepped ${ev.file}:${ev.line}`);return;}this.setStatus('No more physical actions');
  }
  async runToCursor(){
    const file=this.currentFile,line=this.editor.getCursorLine();const token=++this.runToken;this.resetSimulation({cancel:false});let prep;try{prep=await this.prepare();}catch{return;}let hit=false;
    try{for(let i=0;i<prep.events.length;i++){const ev=prep.events[i];if(ev.kind==='send_action'&&ev.file===file&&ev.line>line)break;if(!(await this._applyEvent(ev,token)))return;this.stepIndex=i+1;if(ev.kind==='send_action'&&ev.file===file&&ev.line===line)hit=true;}}catch{return;}
    this.setStatus(hit?`Stopped at ${file}:${line}`:`Ran commands through ${file}:${line}`);
  }
  stop(){this.runToken++;this.editor.highlightLine(null);$('simActionLabel').textContent='Stopped';this.setStatus('Simulation stopped');}
  problem(level,code,message){this.problems.push({level,code,message});this.openBottom('problems');}
  renderPanels(){
    const p=$('problemsPanel');p.innerHTML=this.problems.length?this.problems.map(x=>`<div class="problem ${x.level}"><strong>${x.code}</strong><div>${escapeHtml(x.message).replace(/\n/g,'<br>')}</div></div>`).join(''):'<div class="empty-state">No problems.</div>';if(this.console.stdout||this.console.stderr)p.innerHTML+=`<div class="console-block">${this.console.stdout.split('\n').filter(Boolean).map(x=>`<div class="console-line">${escapeHtml(x)}</div>`).join('')}${this.console.stderr.split('\n').filter(Boolean).map(x=>`<div class="console-line stderr">${escapeHtml(x)}</div>`).join('')}</div>`;
    const tel=this.sim.getTelemetry();$('telemetryPanel').innerHTML=`<div class="panel-note">SIMULATED STATE — not measured hardware telemetry.</div><table><tr><th>Field</th><th>Modeled value</th></tr>${Object.entries(tel).map(([k,v])=>`<tr><td>${k}</td><td>${Number(v).toFixed(3)}</td></tr>`).join('')}</table>`;
    $('commandsPanel').innerHTML=this.commands.length?this.commands.map((c,i)=>`<div class="command-row ${i===this.stepIndex-1?'active':''}"><span>${i+1}</span><span>${c.file}:${c.line}</span><code>${escapeHtml(JSON.stringify(c.action))}</code><span>queued</span></div>`).join(''):'<div class="empty-state">Run or Step Action to prepare the physical command queue.</div>';
    const con=this.sim.getContacts();$('contactsPanel').innerHTML=`<div class="panel-note">MODELED CONTACT / SUPPORT GEOMETRY — no force, torque, current, or tactile sensor data.</div><div class="metric-grid">${Object.entries(con).map(([k,v])=>`<span>${escapeHtml(k)}</span><strong>${typeof v==='number'?v.toFixed(3):escapeHtml(String(v))}</strong>`).join('')}</div>`;
  }
  openBottom(name){$('bottomPanel').classList.remove('collapsed');document.querySelectorAll('.bottom-tab').forEach(b=>b.classList.toggle('active',b.dataset.panel===name));document.querySelectorAll('.panel-view').forEach(v=>v.hidden=true);const map={problems:'problemsPanel',telemetry:'telemetryPanel',commands:'commandsPanel',contacts:'contactsPanel',task:'taskBottomPanel'};$(map[name]).hidden=false;this.renderPanels();}
  toggleSidebar(){$('workspace').classList.toggle('sidebar-collapsed');setTimeout(()=>this.editor.refresh(),30);}
  togglePanel(){const b=$('bottomPanel');if(b.classList.contains('collapsed'))this.openBottom('problems');else b.classList.add('collapsed');setTimeout(()=>this.editor.refresh(),30);}
  setStatus(text){$('statusMessage').textContent=text;}
  download(name,text,type='text/plain'){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([text],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000);}
  exportWorkspace(){const text=Object.entries(this.files).map(([n,c])=>`# ===== ${n} =====\n${c}`).join('\n\n');this.download(`${this.profileId}-workspace.py`,text);}
  importFile(file){const r=new FileReader();r.onload=()=>{const name=file.name.endsWith('.py')?file.name:'main.py';this.files[name]=String(r.result);this.dirty.add(name);this.openFile(name);this.renderFiles();};r.readAsText(file);}
  openPalette(){const box=$('commandPalette');box.hidden=false;$('commandInput').value='';this.renderPalette('');setTimeout(()=>$('commandInput').focus(),0);}
  renderPalette(q){const commands=this.commandsList().filter(c=>c.label.toLowerCase().includes(q.toLowerCase()));$('commandList').innerHTML='';commands.forEach(c=>{const b=document.createElement('button');b.textContent=c.label;b.onclick=()=>{$('commandPalette').hidden=true;c.run();};$('commandList').appendChild(b);});}
  commandsList(){return[{label:'Run: Run simulation',run:()=>this.run()},{label:'Run: Step physical action',run:()=>this.step()},{label:'Run: Run to cursor',run:()=>this.runToCursor()},{label:'View: Toggle Explorer',run:()=>this.toggleSidebar()},{label:'View: Toggle diagnostics panel',run:()=>this.togglePanel()},{label:'View: Fit simulator',run:()=>this.sim.fit()},{label:'Robot: Contact diagnostics',run:()=>this.openBottom('contacts')},{label:'Robot: Simulated telemetry',run:()=>this.openBottom('telemetry')},{label:'File: Save draft',run:()=>this.save()},{label:'File: Export workspace',run:()=>this.exportWorkspace()}];}
  dispatch(action){const map={import:()=> $('importFile').click(),save:()=>this.save(),exportMain:()=>this.download('main.py',this.files['main.py']),exportWorkspace:()=>this.exportWorkspace(),resetWorkspace:()=>this.resetWorkspace(),undo:()=>this.editor.undo(),redo:()=>this.editor.redo(),find:()=>this.editor.find(),replace:()=>this.editor.replace(),toggleComment:()=>this.editor.toggleComment(),palette:()=>this.openPalette(),run:()=>this.run(),step:()=>this.step(),cursor:()=>this.runToCursor(),stop:()=>this.stop(),reset:()=>this.resetSimulation(),sidebar:()=>this.toggleSidebar(),panel:()=>this.togglePanel(),editorFocus:()=>this.editor.focus(),simulatorFocus:()=> $('simCanvas').focus(),fit:()=>this.sim.fit(),contacts:()=>this.openBottom('contacts'),telemetry:()=>this.openBottom('telemetry'),api:()=>this.openBottom('task'),shortcuts:()=>this.openBottom('task'),fidelity:()=>this.openBottom('task')};map[action]?.();}
  bind(){
    $('robotSelect').onchange=e=>this.loadProfile(e.target.value);$('runBtn').onclick=()=>this.run();$('stepBtn').onclick=()=>this.step();$('cursorBtn').onclick=()=>this.runToCursor();$('stopBtn').onclick=()=>this.stop();$('resetBtn').onclick=()=>this.resetSimulation();$('fitBtn').onclick=()=>this.sim.fit();$('panelToggle').onclick=()=>this.togglePanel();$('sidebarToggle').onclick=()=>this.toggleSidebar();$('bottomClose').onclick=()=> $('bottomPanel').classList.add('collapsed');
    $('mobileCodeBtn').onclick=()=>{ $('workspace').classList.remove('show-sim');$('mobileCodeBtn').classList.add('active');$('mobileSimBtn').classList.remove('active');setTimeout(()=>this.editor.refresh(),20);};$('mobileSimBtn').onclick=()=>{$('workspace').classList.add('show-sim');$('mobileCodeBtn').classList.remove('active');$('mobileSimBtn').classList.add('active');};
    document.querySelectorAll('.bottom-tab').forEach(b=>b.onclick=()=>this.openBottom(b.dataset.panel));
    document.querySelectorAll('.menu-button').forEach(b=>b.onclick=e=>{e.stopPropagation();const id=b.dataset.menu+'Menu';document.querySelectorAll('.menu-popover').forEach(m=>{if(m.id!==id)m.hidden=true;});$(id).hidden=!$(id).hidden;});
    document.querySelectorAll('[data-action]').forEach(b=>b.onclick=e=>{e.stopPropagation();document.querySelectorAll('.menu-popover').forEach(m=>m.hidden=true);this.dispatch(b.dataset.action);});document.addEventListener('click',()=>document.querySelectorAll('.menu-popover').forEach(m=>m.hidden=true));
    $('commandClose').onclick=()=> $('commandPalette').hidden=true;$('commandInput').oninput=e=>this.renderPalette(e.target.value);$('commandPalette').onclick=e=>{if(e.target===$('commandPalette'))$('commandPalette').hidden=true;};
    $('importFile').onchange=e=>{const f=e.target.files?.[0];if(f)this.importFile(f);e.target.value='';};
    document.addEventListener('keydown',e=>{if(e.key==='Escape'){if(!$('commandPalette').hidden){$('commandPalette').hidden=true;return;}this.stop();return;}if(e.key==='F10'&&!e.ctrlKey){e.preventDefault();this.step();}if(e.key==='F5'){e.preventDefault();if(e.shiftKey)this.stop();else this.run();}if(e.ctrlKey&&e.key==='F10'){e.preventDefault();this.runToCursor();}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='b'){e.preventDefault();this.toggleSidebar();}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='j'){e.preventDefault();this.togglePanel();}});
    let start=null;$('splitter').addEventListener('pointerdown',e=>{start={x:e.clientX,pct:parseFloat(getComputedStyle(document.documentElement).getPropertyValue('--editor-pct'))||52};document.body.classList.add('resizing');$('splitter').setPointerCapture(e.pointerId);});$('splitter').addEventListener('pointermove',e=>{if(!start)return;const rect=$('mainPanes').getBoundingClientRect();const pct=clamp(start.pct+(e.clientX-start.x)/rect.width*100,30,75);document.documentElement.style.setProperty('--editor-pct',pct+'%');this.editor.refresh();});$('splitter').addEventListener('pointerup',()=>{start=null;document.body.classList.remove('resizing');});$('splitter').ondblclick=()=>{document.documentElement.style.setProperty('--editor-pct','52%');this.editor.refresh();};
  }
}
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));const escapeHtml=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
window.addEventListener('DOMContentLoaded',()=>{try{new App();}catch(e){console.error(e);$('statusMessage').textContent='Startup failed';$('problemsPanel').innerHTML=`<div class="problem error"><strong>STARTUP</strong><div>${escapeHtml(e.message)}</div></div>`;$('bottomPanel').classList.remove('collapsed');}});
