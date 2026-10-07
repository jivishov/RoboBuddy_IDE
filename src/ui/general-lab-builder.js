import { GENERAL_SCENE_VERSION, GENERAL_LIMITS, normalizeGeneralScene } from '../physics/openarm-general-scene.js';
import { generalSceneSchema } from '../physics/openarm-general-schema.js';

const PROJECT_VERSION = 'robobuddy.lab.project.v2';
const stringify = value => JSON.stringify(value, null, 2);
const emptyScene = () => ({ schema_version: GENERAL_SCENE_VERSION, id: 'my_laboratory', reference: { mode: 'none' }, objects: [], inventory: [], assumptions: [] });
function element(tag, text, attributes = {}) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  for (const [key,value] of Object.entries(attributes)) node.setAttribute(key,String(value));
  return node;
}
function download(name, value) {
  const url = URL.createObjectURL(new Blob([stringify(value)+'\n'], { type:'application/json' }));
  const link=element('a',undefined,{href:url,download:name}); document.body.append(link); link.click(); link.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
/** Human review surface. Physical mutations share the IDE execution lease and never auto-run on import. */
export function installGeneralLabBuilder(app) {
  if(typeof document==='undefined'||typeof document.querySelector!=='function')return;
  const host=document.querySelector('.sim-head');
  if(!host||document.getElementById('generalLabPanel')) return;
  const css=element('style');
  css.textContent=`
    #generalLabPanel { position:fixed; inset:110px 12px auto auto; margin:0; width:550px; max-width:92vw; height:calc(100vh - 145px); overflow:auto; box-sizing:border-box; z-index:200; border:1px solid #89969f; border-radius:10px; padding:16px; background:#f4f6f7; color:#23313a; box-shadow:0 12px 40px #0005; font:13px/1.45 system-ui,sans-serif; }
    #generalLabPanel:not([open]) { display:none; }
    #generalLabPanel header, #generalLabPanel .lab-actions { display:flex; align-items:center; gap:7px; flex-wrap:wrap; margin-bottom:10px; }
    #generalLabPanel header strong { font-size:17px; flex:1; }
    #generalLabPanel p { margin:7px 0 12px; }
    #generalLabPanel button { background:#fff; color:#23313a; border:1px solid #94a4ad; border-radius:5px; padding:6px 9px; cursor:pointer; font:inherit; }
    #generalLabPanel button:disabled { opacity:.45; cursor:not-allowed; }
    #generalLabPanel label { display:block; font-weight:600; margin-top:10px; }
    #generalLabPanel textarea { display:block; width:100%; min-height:200px; resize:vertical; box-sizing:border-box; margin:6px 0 10px; padding:10px; font:12px/1.45 ui-monospace,monospace; background:#fff; color:#17252c; border:1px solid #94a4ad; border-radius:5px; tab-size:2; }
    #generalLabPanel #generalLabTask { min-height:130px; }
    #generalLabPanel pre { margin:8px 0; max-height:260px; overflow:auto; white-space:pre-wrap; overflow-wrap:anywhere; background:#e5ebee; padding:10px; border-radius:5px; font:12px/1.45 ui-monospace,monospace; }
    #generalLabPanel output { display:block; border-left:3px solid #577d91; padding:6px 9px; margin:8px 0; white-space:pre-wrap; }
    #generalLabPanel output[data-error=true] { border-color:#a33232; }
    #generalLabPanel summary { cursor:pointer; font-weight:600; padding:7px 0; }
    #generalLabPanel img { display:block; max-width:100%; max-height:210px; margin:8px 0; object-fit:contain; }
    #generalLabPanel input[type=file] { max-width:100%; }
  `;
  document.head.append(css);
  const launch=element('button','Lab Builder',{id:'generalLabOpen',type:'button','aria-expanded':'false','aria-controls':'generalLabPanel',title:'Author and review laboratory scenes. Requires the OpenArm physical workspace.'});host.append(launch);
  const panel=element('dialog',undefined,{id:'generalLabPanel','aria-label':'General laboratory scene builder'});
  const header=element('header');header.append(element('strong','Laboratory scene builder'));
  const close=element('button','Close',{type:'button'});header.append(close);panel.append(header);
  panel.append(element('p','Provide the photo to your multimodal agent. Author unfamiliar equipment using geometry, not a required equipment catalog. Applying a scene resets the simulation; importing a file never runs it.'));
  const state=element('output','Select a ready OpenArm physical workspace.',{id:'generalLabMessage',role:'status'});panel.append(state);
  const label=element('label','SceneSpec JSON',{for:'generalLabScene'}),editor=element('textarea',stringify(emptyScene()),{id:'generalLabScene',spellcheck:'false','aria-label':'Laboratory scene JSON'});panel.append(label,editor);
  const actions=element('div',undefined,{class:'lab-actions'}),mutations=[];
  const addButton=(parent,text,id,fn,mutating=false)=>{const b=element('button',text,{type:'button',id});b.addEventListener('click',event=>{if(!event.isTrusted)return;void Promise.resolve().then(fn).catch(error=>message(error.message,true));});parent.append(b);if(mutating)mutations.push(b);return b;};
  let localImage=null,poll=null;
  const backend=()=>app.sim?.backend;
  const ready=()=>app.profileId==='openarm'&&backend()?.isReady?.()&&typeof backend()?.stageGeneralScene==='function';
  function message(text,error=false){state.textContent=text;state.dataset.error=String(error);}
  async function mutate(fn){
    if(!ready())throw new Error('Open a ready OpenArm physical workspace.');
    if(app.getExecutionState()!=='idle')throw new Error('A program or scene operation is active. Stop or finish it first.');
    const b=backend(),generation=app.workspaceGeneration,token=app.beginExecution();
    if(token==null)throw new Error('Could not acquire the IDE execution lease');
    const guard=()=>{if(app.runToken!==token||backend()!==b||app.workspaceGeneration!==generation||app.profileId!=='openarm')throw new Error('Operation cancelled: workspace or execution ownership changed');};
    mutations.forEach(x=>x.disabled=true);
    try{guard();const result=await fn(b,guard);guard();message('Operation completed. Review reconstruction assumptions and physical evidence separately.');return result;}
    finally{app.finishExecution(token);refresh();}
  }
  const getStage=b=>{
    const staged=b.getGeneralSceneState().staged;
    if(!staged)throw new Error('Stage the scene before checking or applying it.');
    const stagedSpec=b.getGeneralSceneState('spec').stagedScene;
    if(JSON.stringify(normalizeGeneralScene(JSON.parse(editor.value)))!==JSON.stringify(stagedSpec))
      throw new Error('Draft differs from the staged scene. Stage again, or read the staged scene before checking/applying.');
    return staged.id;
  };
  addButton(actions,'Stage','generalLabStage',()=>mutate(b=>b.stageGeneralScene(JSON.parse(editor.value))),true);
  addButton(actions,'Check candidate','generalLabCheck',()=>mutate((b,guard)=>b.checkStagedGeneralScene(getStage(b),.3,guard)),true);
  addButton(actions,'Apply + reset','generalLabApply',()=>mutate((b,guard)=>b.applyStagedEquipment(getStage(b),true,guard)),true);
  addButton(actions,'Discard preview','generalLabDiscard',()=>mutate(b=>b.discardStagedEquipment()),true);
  panel.append(actions);
  const files=element('div',undefined,{class:'lab-actions'});
  addButton(files,'Read current / staged','generalLabRead',()=>{if(!ready())throw new Error('OpenArm not ready');const s=backend().getGeneralSceneState('spec');editor.value=stringify(s.stagedScene??s.scene??emptyScene());});
  addButton(files,'New empty draft','generalLabEmpty',()=>{editor.value=stringify(emptyScene());message('Empty draft only. Robot/mount/floor replace the current workcell only after Stage and Apply.');});
  addButton(files,'Download schema','generalLabSchema',()=>download('robobuddy-lab-scene-v2.schema.json',generalSceneSchema()));
  addButton(files,'Export draft','generalLabExport',()=>{const scene=normalizeGeneralScene(JSON.parse(editor.value));download('laboratory-project.json',{schema_version:PROJECT_VERSION,scene,task:taskEditor.value.trim()?JSON.parse(taskEditor.value):null,auto_start:false});});
  panel.append(files);
  panel.append(element('label','Import SceneSpec or project JSON (draft only)',{for:'generalLabImport'}));
  const file=element('input',undefined,{id:'generalLabImport',type:'file',accept:'.json,application/json'});panel.append(file);
  file.addEventListener('change',async()=>{
    try{const f=file.files?.[0];if(!f)return;if(f.size>800000)throw new Error('Project exceeds 800 kB');const raw=JSON.parse(await f.text());
      let scene=raw,task=null;
      if(raw.schema_version===PROJECT_VERSION){if(raw.auto_start!==false||Object.keys(raw).some(k=>!['schema_version','scene','task','auto_start'].includes(k)))throw new Error('Import accepts only a scene/task draft with auto_start:false, never execution evidence');scene=raw.scene;task=raw.task;}
      editor.value=stringify(normalizeGeneralScene(scene));taskEditor.value=task?stringify(task):'';message('Imported into the editor only. Stage, check and explicitly apply to change the workcell.');
    }catch(error){message(error.message,true);}finally{file.value='';}
  });
  const photoDetails=element('details');photoDetails.append(element('summary','Local photo reference (optional; not sent to the agent)'));
  photoDetails.append(element('p','This thumbnail is for human comparison only. It does not analyze the image, calibrate dimensions or make the pixels available to your agent.'));
  const photo=element('input',undefined,{type:'file',accept:'image/png,image/jpeg,image/webp','aria-label':'Local photo reference'}),image=element('img',undefined,{alt:'Local laboratory reference',hidden:''});photoDetails.append(photo,image);panel.append(photoDetails);
  photo.addEventListener('change',()=>{try{const f=photo.files?.[0];if(!f)return;if(!['image/png','image/jpeg','image/webp'].includes(f.type)||f.size>8000000)throw new Error('Use a PNG, JPEG or WebP photo up to 8 MB');if(localImage)URL.revokeObjectURL(localImage);localImage=URL.createObjectURL(f);image.src=localImage;image.hidden=false;}catch(error){message(error.message,true);}finally{photo.value='';}});
  const taskDetails=element('details');taskDetails.append(element('summary','Task assessment and independent evidence'));
  taskDetails.append(element('p','Optional dry_transfer or vertical insert task. Define before grasping; execute robot commands through WebMCP or the existing controls. Task definition does not move the robot.'));
  const taskEditor=element('textarea','',{id:'generalLabTask',spellcheck:'false','aria-label':'Laboratory task JSON',placeholder:'Paste robobuddy.lab.task.v2 JSON'});taskDetails.append(taskEditor);
  const taskActions=element('div',undefined,{class:'lab-actions'});
  addButton(taskActions,'Assess','generalLabAssess',async()=>{const result=await mutate(b=>b.defineGeneralTask(JSON.parse(taskEditor.value),true));message(stringify(result.preflight));},true);
  addButton(taskActions,'Define task','generalLabDefine',()=>mutate(b=>b.defineGeneralTask(JSON.parse(taskEditor.value),false)),true);
  addButton(taskActions,'Clear task','generalLabClearTask',()=>mutate(b=>b.clearGeneralTask()),true);
  taskDetails.append(taskActions);panel.append(taskDetails);
  const reportLabel=element('strong','Source inventory, uncertainty and physical checks'),report=element('pre','No authored scene yet.',{id:'generalLabReport'});panel.append(reportLabel,report);
  const viewActions=element('div',undefined,{class:'lab-actions'});
  const collision=addButton(viewActions,'Collision view','generalLabCollision',()=>{if(!ready())throw new Error('OpenArm not ready');const enabled=backend().setCollisionView(!backend().collisionView);collision.setAttribute('aria-pressed',String(enabled));});collision.setAttribute('aria-pressed','false');
  addButton(viewActions,'Export evidence','generalLabEvidence',()=>{if(!ready())throw new Error('OpenArm not ready');download('laboratory-evidence.json',backend().getGeneralSceneState('evidence'));});panel.append(viewActions);
  function refresh(){
    launch.hidden=app.profileId!=='openarm';launch.disabled=!ready();
    const idle=ready()&&app.getExecutionState()==='idle';mutations.forEach(b=>b.disabled=!idle);
    if(panel.open&&ready()){const s=backend().getGeneralSceneState();report.textContent=stringify({sceneMode:s.sceneMode,staged:s.staged,report:s.report,task:s.task});}
  }
  function hide(){panel.close();launch.setAttribute('aria-expanded','false');if(poll)clearInterval(poll);poll=null;launch.focus();}
  close.onclick=hide;launch.onclick=()=>{if(panel.open){hide();return;}panel.show();launch.setAttribute('aria-expanded','true');refresh();if(!poll)poll=setInterval(refresh,1000);};
  panel.addEventListener('keydown',event=>{if(['Escape','F5','F10'].includes(event.key)){event.preventDefault();event.stopImmediatePropagation();if(event.key==='Escape')hide();}},true);
  document.body.append(panel);app.onAgentContextChange?.(refresh);refresh();
  return {panel,refresh};
}
