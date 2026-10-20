export function openHand(shift = 0) {
  const p = Array.from({length:21}, () => ({x:.5,y:.5,z:0}));
  p[0]={x:.5,y:.83,z:0};
  [[1,.42,.72],[2,.35,.63],[3,.29,.52],[4,.26,.40],
   [5,.38,.60],[6,.37,.45],[7,.37,.32],[8,.37,.21],
   [9,.49,.57],[10,.49,.39],[11,.49,.26],[12,.49,.15],
   [13,.60,.59],[14,.61,.43],[15,.61,.31],[16,.61,.23],
   [17,.69,.64],[18,.71,.53],[19,.72,.43],[20,.73,.35]].forEach(([i,x,y])=>p[i]={x,y,z:0});
  p.forEach(point=>point.x+=shift);
  return {landmarks:[p],handedness:[[{categoryName:'Right',score:.99}]]};
}
export function fist() {
  const hand=openHand();
  for(const i of [8,12,16,20]){hand.landmarks[0][i]={x:.5,y:.72,z:0};hand.landmarks[0][i-1]={x:.5,y:.68,z:0};}
  return hand;
}
