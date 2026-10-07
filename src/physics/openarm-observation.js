// Read-only interpretation of named simulator contacts. Not tactile sensor emulation.
export function objectGeometryIds(observation, bodyId) {
  if (bodyId === 'flask') return ['flask_body_geom', 'flask_shoulder_geom', 'flask_grip_geom'];
  if (bodyId === 'beaker') return ['beaker_grip_geom'];
  return (observation.openarm?.equipment || []).find(e => e.bodyId === bodyId)?.geometryIds || [];
}
export function contactsForObject(observation, bodyId) {
  const ids = new Set(objectGeometryIds(observation, bodyId));
  return (observation.contacts || []).filter(c => ids.has(c.geom1Name) || ids.has(c.geom2Name));
}
export function contactEvidenceComplete(observation) {
  // An empty or partial/unreadable contact list is not evidence of release.
  return observation.contactsReadable === true && Array.isArray(observation.contacts)
    && Number.isInteger(observation.contactCount) && observation.contactCount >= 0
    && observation.contactCount === observation.contacts.length;
}
export function usableContact(c) { return Number.isFinite(c.normalForceN) && c.normalForceN > .01 && Number.isFinite(c.distanceM) && c.distanceM >= -.002; }
export function activeOrUncertainContact(c) {
  // Excessive penetration must REJECT a good-grasp claim, never erase contact
  // from the release test. Zero-force geometric touch and malformed telemetry
  // also conservatively prevent a release claim. A positive gap with zero load
  // is only a proximity contact and does not prevent release.
  return !Number.isFinite(c.distanceM) || !Number.isFinite(c.normalForceN)
    || c.distanceM <= 0 || c.normalForceN !== 0;
}
export function graspState(observation, side, bodyId) {
  const raw = contactsForObject(observation, bodyId);
  const complete = contactEvidenceComplete(observation);
  const contacts = complete ? raw.filter(usableContact) : [];
  const matches = (c, prefix) => [c.geom1Name, c.geom2Name].some(n => typeof n === 'string' && n.startsWith(prefix));
  const padForce = kind => contacts.filter(c => matches(c, `finger_${kind}_${side}_collision_`)).reduce((sum, c) => sum + c.normalForceN, 0);
  const innerForceN = padForce('inner'), outerForceN = padForce('outer');
  const anyFingerContact = raw.some(c => activeOrUncertainContact(c) && ['inner', 'outer'].some(kind => matches(c, `finger_${kind}_${side}_collision_`)));
  const anyPalmContact = raw.some(c => activeOrUncertainContact(c) && matches(c, `ee_base_link_${side}_collision_`));
  const excessivePenetrationSeen = raw.some(c => Number.isFinite(c.distanceM) && c.distanceM < -.002);
  return { innerForceN, outerForceN, bilateralContact: complete && !excessivePenetrationSeen && innerForceN > .01 && outerForceN > .01,
    anyFingerContact, anyPalmContact, anyGripperContact: anyFingerContact || anyPalmContact,
    contactsReadable: complete, excessivePenetrationSeen,
    source: 'MuJoCo contact forces; not hardware tactile measurements' };
}
export function worldPoint(body, local) {
  const [w, x, y, z] = body.quaternionWxyz;
  const [a, b, c] = local;
  const t = [2 * (y*c-z*b), 2 * (z*a-x*c), 2 * (x*b-y*a)];
  return [a+w*t[0]+y*t[2]-z*t[1], b+w*t[1]+z*t[0]-x*t[2], c+w*t[2]+x*t[1]-y*t[0]].map((v, i) => v + body.positionM[i]);
}
const finiteVec3 = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
export function conditionMet(observation, condition) {
  const o = observation;
  if (condition.type === 'funnel_seated') return funnelSeatingState(o, condition.object_id, condition.receiver_id).seated;
  if (condition.type === 'bilateral_grasp' || condition.type === 'released' || condition.type === 'supported') {
    if (!contactEvidenceComplete(o) || !o.bodies?.[condition.object_id] || !objectGeometryIds(o, condition.object_id).length) return false;
  }
  if (condition.type === 'bilateral_grasp') return graspState(o, condition.side, condition.object_id).bilateralContact;
  if (condition.type === 'released') return !graspState(o, condition.side, condition.object_id).anyGripperContact;
  // This predicate establishes force-bearing contact with the named geometry;
  // it is not, alone, a certificate of stable placement or upward load support.
  if (condition.type === 'supported') return contactsForObject(o, condition.object_id).some(c => usableContact(c) && [c.geom1Name, c.geom2Name].includes(condition.support_geom));
  if (condition.type === 'in_region') {
    const p = o.bodies?.[condition.object_id]?.positionM;
    return Boolean(finiteVec3(p) && p.every((v, i) => Math.abs(v - condition.center_m[i]) <= condition.half_extents_m[i]));
  }
  if (condition.type === 'equipment_joint') {
    const j = o.openarm?.equipmentJoints?.find(j => j.id === condition.joint_id);
    return Boolean(j && Number.isFinite(j.position) && j.position >= condition.minimum && j.position <= condition.maximum);
  }
  if (condition.type === 'tool_reached') {
    const p = o.openarm?.pinchReferences?.[condition.side]?.positionM;
    return Boolean(finiteVec3(p) && Math.hypot(...p.map((v, i) => v - condition.position_m[i])) <= condition.tolerance_m);
  }
  return false;
}

/** Instantaneous seating evidence, not proof of the preceding pick-and-place.
 * A program must separately require grasp/lift and a sustained seating dwell.
 * Fail closed on incomplete contacts, held objects, motion or impossible fit.
 */
export function funnelSeatingState(o, objectId, receiverId) {
  const no = reason => ({ seated: false, reason, evidence: 'instantaneous simulator ground truth; not transfer history' });
  const items = o.openarm?.equipment || [];
  const f = items.find(e=>e.bodyId===objectId), b = items.find(e=>e.bodyId===receiverId);
  if (f?.kind !== 'funnel' || f.dynamic !== true || b?.kind !== 'burette' || b.dynamic !== false) return no('requires a dynamic funnel and fixed burette');
  const fb = o.bodies?.[objectId], bb = o.bodies?.[receiverId];
  const validBody = body => body && finiteVec3(body.positionM) && Array.isArray(body.quaternionWxyz) && body.quaternionWxyz.length===4 && body.quaternionWxyz.every(Number.isFinite) && Math.abs(Math.hypot(...body.quaternionWxyz)-1)<.001;
  if (!validBody(fb) || !validBody(bb) || !contactEvidenceComplete(o)) return no('unreadable or incomplete physical state');
  if (!finiteVec3(fb.linearVelocityMS) || !finiteVec3(fb.angularVelocityRadS)) return no('missing free-body velocities');
  const axis = body => worldPoint(body,[0,0,1]).map((v,i)=>v-body.positionM[i]);
  const fa=axis(fb), ba=axis(bb), dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const tip=worldPoint(fb,f.affordances.stemTipM), mouth=worldPoint(bb,b.affordances.openingCenterM);
  const relative=tip.map((v,i)=>v-mouth[i]);
  const depth=-dot(relative,ba), radial=Math.hypot(...relative.map((v,i)=>v+depth*ba[i]));
  const alignment=Math.max(-1,Math.min(1,dot(fa,ba)));
  const tilt=Math.acos(alignment), clearance=b.affordances.innerRadiusM-f.affordances.stemRadiusM;
  const maxDepth=f.affordances.stemLengthM + (b.affordances.outerRadiusM-f.affordances.stemRadiusM) * (f.affordances.heightM-f.affordances.stemLengthM)/(f.affordances.bowlRadiusM-f.affordances.stemRadiusM) + .002;
  const geometryFits=ba[2]>.98 && alignment>0 && clearance>0 && depth>=f.affordances.stemLengthM*.7 && depth<=maxDepth && radial+Math.sin(tilt)*f.affordances.stemLengthM<=clearance+.0005;
  const contacts=contactsForObject(o,objectId), receiverGeoms=new Set(b.geometryIds), funnelGeoms=new Set(f.geometryIds);
  const other=c=>funnelGeoms.has(c.geom1Name)?c.geom2Name:c.geom1Name;
  const receiverContacts=contacts.filter(c=>receiverGeoms.has(other(c)) && usableContact(c));
  const receiverForceN=receiverContacts.reduce((s,c)=>s+c.normalForceN,0);
  const released=['left','right'].every(side=>!graspState(o,side,objectId).anyGripperContact);
  const excessivePenetration=contacts.some(c=>!Number.isFinite(c.distanceM) || c.distanceM<-.002);
  const otherSupport=contacts.some(c=>!receiverGeoms.has(other(c)) && activeOrUncertainContact(c));
  const settled=Math.hypot(...fb.linearVelocityMS)<.01 && Math.hypot(...fb.angularVelocityRadS)<.15;
  const supported=receiverForceN>Math.max(.01,f.mass_kg*9.81*.2);
  return { seated: geometryFits && released && !excessivePenetration && !otherSupport && settled && supported,
    geometryFits, released, settled, supported, excessivePenetration, otherSupport,
    insertionDepthM: depth, radialOffsetM: radial, tiltRad: tilt, receiverForceN,
    evidence: 'instantaneous simulator ground truth; require dwell and independent grasp/lift observations to establish a transfer' };
}
