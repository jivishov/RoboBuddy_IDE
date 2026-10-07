/** Cooperative guard; an already submitted physics batch cannot be undone. */
export function asimovAdvanceGuard({signal,assertActive,deadlineMs}={}) {
  if(signal?.aborted) throw new Error('Asimov operation cancelled');
  if(deadlineMs!=null && performance.now()>=deadlineMs) throw new Error('Asimov operation wall deadline exceeded');
  assertActive?.();
}
export function asimovCommandDuration(seconds,dt) {
  if(!Number.isFinite(seconds)||seconds<0||seconds>20||!Number.isFinite(dt)||dt<=0) throw new RangeError('Invalid bounded Asimov command duration');
  if(seconds!==0 && (Math.round(seconds/dt)<1||Math.abs(Math.round(seconds/dt)*dt-seconds)>1e-9)) throw new RangeError('Advance must align with this scene timestep');
}
