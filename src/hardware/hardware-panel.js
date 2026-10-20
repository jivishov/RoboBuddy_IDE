import {HardwareClient, helperAddress} from './bridge-client.js';
import {JOINTS, visualPose} from './joint-map.js';
import {iconButton} from './icons.js';
import {GesturePanel} from '../gestures/gesture-panel.js';
import {hardwareGestureAdapter} from '../gestures/hardware-adapter.js';
const $ = id => document.getElementById(id);
const names = {shoulder_pan:'Shoulder pan', shoulder_lift:'Shoulder lift', elbow_flex:'Elbow', wrist_flex:'Wrist flex', wrist_roll:'Wrist roll', gripper:'Gripper'};
let state = null, transportOk = false, visualizer = null, visualLoading = false, visualFailed = false, mapping = {}, receivedAt = 0, setupWasReady = false, setupFocusPending = false;
for (const [id, name, label, tooltip] of [
  ['pairBtn','link','Pair helper'],
  ['refreshPorts','refresh','Refresh USB ports'],
  ['connectBtn','plug','Connect · read only','Connect read only. No homing or motor configuration changes.'],
  ['disconnectBtn','unplug','Disconnect','Disconnect the serial port. Motor torque will no longer be monitored.'],
  ['quitBtn','exit','Quit helper'],
  ['armBtn','power','Enable motors at current pose','Enable motors at the measured pose after confirming the arm is supported.'],
  ['disableBtn','powerOff','Disable motors · support arm','Remove motor torque. Support the arm first; it can fall.'],
  ['calStart','calibrate','Start calibration','1 · Start calibration with the arm supported and torque disabled.'],
  ['calMiddle','middle','Middle pose ready','2 · Confirm the documented middle pose. Writes homing offsets.'],
  ['calFinish','save','Ranges reviewed · save','3 · Save the reviewed joint ranges.'],
  ['calCancel','cancel','Cancel calibration'],
  ['stopBtn','stop','Stop motion','Stop · Esc. Clear the jog and request a hold. Motors remain enabled.'],
  ['quickStopBtn','stop','Stop hardware motion','Stop · Esc. Clear the jog and request a hold. Motors remain enabled.'],
]) iconButton($(id), name, label, tooltip);
const client = new HardwareClient({onState: value => {state=value;transportOk=true;receivedAt=performance.now();render();},onError: error => {transportOk=false;message(error.message,true);render();}});
const gestures = new GesturePanel($('gestureControlBtn'), hardwareGestureAdapter(client, () => {
  if (!state) return null;
  return {...state, fresh: transportOk && state.fresh && state.sampleAgeMs + performance.now() - receivedAt < 600};
}, () => $('supportAck').checked));
function message(text,error=false) {$('feedback').textContent=text;$('feedback').classList.toggle('error',error);}
function supported() {if (!$('supportAck').checked) throw new Error('Confirm the arm is supported and its movement area is clear.');return true;}
async function action(operation,args={}) {
  if (gestures.enabled || gestures.enabling) await gestures.pause('Manual control selected. Set neutral to use gestures again.');
  if(['connect','calibration_finish','calibration_import'].includes(operation)) setupFocusPending=$('setupDetails').contains(document.activeElement);
  try {message('Working…');const pending=operation==='stop' ? client.stop() : client.command(operation,args);render();const result=await pending;message(operation==='stop'?(result.state==='holding'?'Hold requested. This is a software stop; motors remain enabled.':'No panel jog was active. Check the observed motor state.'):operation==='disconnect'?'Disconnected. Motor torque is no longer monitored.':'Operation complete.');}
  catch(error) {message(error.message,true);}
  finally {setupFocusPending=false;render();}
}
for (const [index,joint] of JOINTS.entries()) {
  const row=document.createElement('div');row.className='joint-row';row.dataset.joint=joint;
  row.innerHTML='<div class="joint-name"></div><div class="encoder-readout"><output class="actual" aria-live="off">—</output><small class="raw-value">No data</small></div><div class="jog-buttons"><button class="minus" type="button"></button><span class="jog-step">'+(joint==='gripper'?'2%':'1°')+'</span><button class="plus" type="button"></button></div><div class="target-control"><input type="number" step="'+(joint==='gripper'?'1':'.1')+'"><button class="apply" type="button"></button></div>';
  row.querySelector('.joint-name').append(document.createTextNode(names[joint]));
  const note=document.createElement('small');note.textContent='Motor '+(index+1)+(joint==='gripper'?' · % open':' · degrees');row.querySelector('.joint-name').append(note);
  const input=row.querySelector('input');input.setAttribute('aria-label',names[joint]+' target');
  input.title='Absolute target in '+(joint==='gripper'?'percent open; maximum change 5%.':'degrees; maximum change 3°.');
  row.querySelector('.actual').setAttribute('aria-label',names[joint]+' encoder');
  for(const [selector,delta] of [['.minus',-1],['.plus',1]]) {
    const button=row.querySelector(selector);iconButton(button,delta>0?'plus':'minus','Jog '+names[joint]+' '+(delta>0?'positive':'negative'),names[joint]+' · '+(delta>0?'+':'−')+(joint==='gripper'?'2%':'1°'));
    button.addEventListener('click',()=>{const actual=state?.joints[joint]?.position;if(typeof actual==='number')void action('move',{targets:{[joint]:actual+delta*(joint==='gripper'?2:1)}});});
  }
  const apply=row.querySelector('.apply');iconButton(apply,'apply','Apply '+names[joint]+' target');
  apply.addEventListener('click',()=>void action('move',{targets:{[joint]:Number(input.value)}}));
  $('jointRows').append(row);
}
for(const joint of JOINTS.filter(j=>j!=='gripper')) {
  mapping[joint]={sign:1,offset:0};
  const row=document.createElement('label');row.className='alignment-row';
  const title=document.createElement('span');title.textContent=names[joint];
  const select=document.createElement('select');select.innerHTML='<option value="1">+ direction</option><option value="-1">− direction</option>';select.setAttribute('aria-label',names[joint]+' visual direction');
  const offset=document.createElement('input');offset.type='number';offset.value='0';offset.min='-360';offset.max='360';offset.setAttribute('aria-label',names[joint]+' visual offset');
  const change=()=>{mapping[joint]={sign:Number(select.value),offset:Number(offset.value)};render();};select.addEventListener('change',change);offset.addEventListener('change',change);
  row.append(title,select,offset);$('alignmentRows').append(row);
}
async function draw(fresh) {
  let pose;try {pose=visualPose(state ? {...state, fresh: Boolean(fresh)} : null,mapping);}catch(error){$('visualMessage').hidden=false;$('visualMessage').textContent=error.message;return;}
  if (!pose) {$('visualMessage').hidden=false;$('visualMessage').textContent=state?.connected?'Encoder view paused — waiting for fresh calibrated readings.':'The 3D view waits for calibrated encoder readings.';return;}
  if (!visualizer && !visualLoading && !visualFailed) {
    visualLoading=true;
    try {const {HardwareVisualizer}=await import('./hardware-visualizer.js');const view=new HardwareVisualizer();await view.init($('hardwareCanvas'));visualizer=view;}
    catch(error) {visualFailed=true;$('visualMessage').textContent='3D assets could not load. Numeric encoder readings remain available.';}
    finally {visualLoading=false;}
  }
  if(visualizer){visualizer.update(pose);$('visualMessage').hidden=true;}
}
function render() {
  const paired=!!client.session&&transportOk;
  const connected=paired&&!!state?.connected;
  const age=state?.sampleAgeMs===null?null:(state?.sampleAgeMs||0)+(performance.now()-receivedAt);
  const fresh=connected&&state?.fresh&&age!==null&&age<600;
  const busy=client.busy;
  const moving=connected&&state?.state==='moving';
  const gestureActive=connected&&!!state?.gesture;
  const canMove=fresh&&state.calibrated&&state.operatorEnabled&&['armed','holding'].includes(state.state)&&!busy&&!state.gesture;
  const readOnly=connected&&state?.state==='read_only'&&fresh&&!busy;
  const torque=Object.values(state?.joints||{}).some(j=>j.torqueEnabled);
  const setupReady=connected&&state?.calibrated&&!['calibrating','fault'].includes(state.state);
  if(setupReady&&!setupWasReady) {
    const restoreFocus=$('setupDetails').contains(document.activeElement)||(setupFocusPending&&document.activeElement===document.body);
    $('setupDetails').open=false;
    if(restoreFocus)$('setupToggle').focus({preventScroll:true});
  }
  setupWasReady=setupReady;
  $('setupSummary').textContent=connected?state.port+' · '+state.armId+' · Calibration '+(state.calibrated?'matched':'required'):paired?'Helper paired · Choose a USB port':'Helper not paired';
  $('demoBanner').hidden=state?.mode!=='demo';$('modeBadge').hidden=state?.mode==='demo';$('modeBadge').textContent='LOCAL HARDWARE';
  $('pairBtn').disabled=paired;iconButton($('pairBtn'),paired?'check':'link',paired?'Helper paired':'Pair helper');
  $('refreshPorts').disabled=!paired||busy;$('connectBtn').disabled=!paired||connected||busy||!$('portSelect').value;
  $('disconnectBtn').disabled=!connected||busy;$('quitBtn').disabled=!paired||busy;
  $('armBtn').disabled=!readOnly||!state?.calibrated||torque||!$('supportAck').checked;
  $('disableBtn').disabled=!connected||busy||!$('supportAck').checked;
  $('torqueHelp').hidden=!connected||state?.operatorEnabled||!torque;
  $('stopBtn').disabled=!connected;$('quickStopBtn').disabled=!connected;$('calStart').disabled=!readOnly||torque||!$('supportAck').checked;
  $('calMiddle').disabled=state?.calibrationStage!=='middle'||!connected||busy;
  $('calFinish').disabled=state?.calibrationStage!=='ranges'||!connected||busy;
  $('calCancel').disabled=state?.state!=='calibrating'||!connected||busy;
  $('calibrationFile').disabled=!readOnly||torque||!$('supportAck').checked;
  $('portSelect').disabled=connected;$('armId').disabled=connected;
  $('stateLabel').textContent=!paired?'Helper not paired':({disconnected:'USB not connected',read_only:'Connected · read only',armed:'Motors enabled',moving:'Bounded jog in progress',holding:'Hold requested',calibrating:'Calibration in progress',fault:'Fault · stop unconfirmed'})[state.state]||state.state;
  $('stateDetail').textContent=state?.fault?.message||(!paired?'Starting this page does not open USB.':connected?(state.mode==='demo'?'Synthetic demo readings':state.port+' · '+state.armId):'Choose a port and Connect.');
  $('stateDot').className='dot'+(state?.state==='fault'?' fault':connected?' active':'');
  $('calibrationBadge').textContent=state?.calibrated?'CALIBRATION MATCHED':'CALIBRATION REQUIRED';
  $('calibrationSummary').textContent=state?.calibrated?'Saved calibration matches the motor registers. Hardware commissioning and 3D alignment remain pending.':'Use the existing LeRobot arm name, or explicitly calibrate/import after connection.';
  $('sampleBadge').textContent=fresh?(state.mode==='demo'?'DEMO READBACK':'ENCODER READBACK'):'NO FRESH DATA';
  $('telemetryAge').textContent=age===null||!connected?'—':Math.round(age)+' ms';
  $('torqueState').textContent=!connected||state.state==='fault'?'Unknown / not monitored':torque?'Enabled':'Disabled';
  $('connectionState').textContent=connected?state.port:paired?'Paired · no USB':'Not paired';
  const fault=state?.state==='fault';
  $('controlBadge').textContent=fault?'FAULT':!connected?'CONTROLS LOCKED':!fresh?'STALE DATA':gestureActive?'HAND CONTROL':moving?'JOG IN PROGRESS':busy?'WORKING':state.state==='calibrating'?'CALIBRATING':canMove?'READY TO JOG':'READ ONLY';
  $('controlBadge').dataset.tone=fault?'fault':moving||busy?'moving':canMove?'ready':'';
  $('controlHint').textContent=fault?'Control locked. Check the arm locally; a physical stop is unconfirmed.':!paired?'Pair the local helper and connect an arm to unlock manual controls.':!connected?'Connect read only, then confirm support and enable motors.':!fresh?'Waiting for fresh encoder readings. Jog controls are locked.':!state.calibrated?'Calibration required. Open Connection & calibration to set up or restore it.':gestureActive?'Webcam hand control is active. Pause hand control to use manual jogs. Stop remains available.':moving?'A bounded jog is in progress. Stop remains available.':busy?'Command in progress. Stop remains available.':canMove?'Ready · use a jog step or enter an absolute target, then apply.':torque?'Motors are powered. Support the arm, disable motors, then enable control here.':!$('supportAck').checked?'Confirm the arm is supported, then enable motors at its current pose.':'Ready to enable motors at the measured pose.';
  if(state?.driver)$('driverLabel').textContent=state.driver;
  for(const row of $('jointRows').children) {
    const j=state?.joints?.[row.dataset.joint],input=row.querySelector('input');
    row.querySelector('.actual').textContent=typeof j?.position==='number'?j.position.toFixed(1)+(j.unit==='degrees'?'°':'%'):j?j.rawTicks+' ticks':'—';
    row.querySelector('.raw-value').textContent=j?j.rawTicks+' ticks':'No data';
    row.querySelector('.encoder-readout').classList.toggle('stale',!fresh);
    if(document.activeElement!==input&&typeof j?.position==='number')input.value=j.position.toFixed(1);
    if(j?.limits){input.min=j.limits[0];input.max=j.limits[1];}
    for(const el of row.querySelectorAll('button,input'))el.disabled=!canMove;
  }
  const stage=state?.calibrationStage;
  $('wizardText').textContent=stage==='middle'?'Support the arm. Place all joints at the middle pose in the official guide, then click Middle pose ready. This next step writes homing offsets.':stage==='ranges'?'With torque off, move each joint except wrist roll gently through its full usable range. Review the encoder ranges below; save only after checking them.':'Start, place joints at the documented middle pose, record their usable ranges by hand, then review and save. Wrist roll follows LeRobot\'s full-turn convention.';
  $('rangeReport').textContent=stage==='ranges'?JOINTS.filter(j=>j!=='wrist_roll').map(j=>names[j]+': '+(state.recordedRanges[j]||[]).join(' — ')+' ticks').join('\n'):'';
  $('eventList').replaceChildren(...(state?.events||[]).slice().reverse().map(event=>{const li=document.createElement('li');li.textContent=new Date(event.time*1000).toLocaleTimeString()+' · '+event.event.replaceAll('_',' ');return li;}));
  void draw(fresh);
}
$('supportAck').addEventListener('change',render);
$('portSelect').addEventListener('change',render);
$('pairForm').addEventListener('submit',async event=>{
  event.preventDefault();
  try {await client.pair($('helperAddress').value,$('pairingCode').value);await refreshPorts();message('Helper paired. Select the arm port; Connect stays read only.');}
  catch(error){message(error.message,true);render();}
});
async function refreshPorts() {
  const selected=$('portSelect').value, found=await client.ports();$('portSelect').replaceChildren();
  if(!found.length){const option=document.createElement('option');option.value='';option.textContent='No USB serial ports found';$('portSelect').append(option);}
  for(const port of found){const option=document.createElement('option');option.value=port.device;option.textContent=port.device+' · '+port.description;$('portSelect').append(option);}
  if(found.some(p=>p.device===selected))$('portSelect').value=selected;render();
}
$('refreshPorts').addEventListener('click',()=>refreshPorts().catch(error=>message(error.message,true)));
$('connectBtn').addEventListener('click',()=>void action('connect',{port:$('portSelect').value,armId:$('armId').value}));
$('disconnectBtn').addEventListener('click',()=>void action('disconnect'));
$('armBtn').addEventListener('click',()=>{try{void action('arm',{supported:supported()});}catch(error){message(error.message,true);}});
$('disableBtn').addEventListener('click',()=>{try{void action('disable',{supported:supported()});}catch(error){message(error.message,true);}});
$('stopBtn').addEventListener('click',()=>void action('stop'));
$('quickStopBtn').addEventListener('click',()=>void action('stop'));
$('quitBtn').addEventListener('click',async()=>{try{await client.quit();state=null;transportOk=false;message('Helper closed. Motor torque is no longer monitored.');render();}catch(error){message(error.message,true);}});
$('calStart').addEventListener('click',()=>{try{void action('calibration_start',{supported:supported()});}catch(error){message(error.message,true);}});
$('calMiddle').addEventListener('click',()=>void action('calibration_middle',{middleConfirmed:true}));
$('calFinish').addEventListener('click',()=>void action('calibration_finish',{rangesConfirmed:true}));
$('calCancel').addEventListener('click',()=>void action('calibration_cancel'));
$('calibrationFile').addEventListener('change',async event=>{
  try{const file=event.target.files[0];if(!file)return;if(file.size>16000)throw new Error('Calibration file is too large.');const calibration=JSON.parse(await file.text());await action('calibration_import',{calibration,supported:supported()});}
  catch(error){message(error.message,true);}finally{event.target.value='';}
});
document.addEventListener('keydown',event=>{if(event.key==='Escape'&&client.session){event.preventDefault();void action('stop');}});
document.addEventListener('visibilitychange',()=>{if(document.hidden)client.bestEffortStop();});
window.addEventListener('pagehide',()=>{client.bestEffortStop();client.close();});
setInterval(()=>{if(state)render();},300);
async function bootstrap() {
  if(!['127.0.0.1','localhost'].includes(location.hostname))return;
  $('helperAddress').value=location.origin;
  try {
    const response=await fetch('/api/bootstrap',{credentials:'omit'});if(!response.ok)return;
    const data=await response.json();$('pairingCode').value=data.token;
    await client.pair(location.origin,data.token);await refreshPorts();message('Local helper paired. Select a port and Connect; no USB has been opened.');
  } catch(error){message(error.message,true);}
}
render();void bootstrap();
if (['127.0.0.1','localhost'].includes(location.hostname)) $('launchHelp').hidden = true;
$('helperAddress').addEventListener('change', () => {try {$('openLocalPanel').href=helperAddress($('helperAddress').value)+'/hardware.html';} catch {}});

