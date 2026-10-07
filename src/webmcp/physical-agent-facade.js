export function installPhysicalAgentFacade(facade) {
  if (!facade || facade.__physicalFacadeInstalled) return facade;
  const describeTask = facade.describeTask.bind(facade);
  const inspectSimulation = facade.inspectSimulation.bind(facade);

  facade.describeTask = (snapshot, input = {}) => {
    const result = describeTask(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    const openarm = snapshot?.profileId === 'openarm';
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      fidelityBoundaries: openarm ? [
        'Browser MuJoCo through one PhysicsSession is the OpenArm V2 physical authority; both arms, renderer, live Python, WebMCP and task evaluation consume that same state.',
        'The flask and beaker are true free bodies. The only equality constraints are source-derived mechanical finger couplings; no task object is welded, parented, snapped, teleported or advanced by presentation code.',
        'V2 kinematics, mirrored joint frames, inertials and simulator actuator settings are source-derived from pinned enactic/openarm_mujoco. Robot rendering and contacts share source-matched convex components. The servo operating profile and dry workcell remain simulator estimates, not hardware calibration. Inspect, stage and explicitly apply equipment through the OpenArm workcell tools; custom programs support bounded joint/Cartesian targets and observed condition checks.',
      ] : [
        'Browser MuJoCo through one PhysicsSession is the SO-101 physical authority; the renderer, live Python, WebMCP, and task evaluator consume that same state.',
        'The visible block is a true free body. Task evidence is derived from named gripper contacts, lift, carried motion, release, gravity/contact settling, and final rest; no snap, weld, parenting, transform overwrite, or synthetic success event is used.',
        'The workcell is a declared synthetic rigid-body benchmark. Source-derived SO-101 geometry/model provenance and simulator-only numerical verification do not constitute hardware calibration.',
      ],
    };
  };

  facade.inspectSimulation = (snapshot, input = {}) => {
    const result = inspectSimulation(snapshot, input);
    if (snapshot?.simulationMode !== 'physical_mujoco') return result;
    return {
      ...result,
      simulationMode: 'physical_mujoco',
      stateKind: snapshot.stateKind,
      sourcePlantAvailable: false,
      physicalSimulationAvailable: true,
      policySimulationAvailable: false,
      hardwareValidated: false,
      taskEvaluation: snapshot.simulation?.taskEvaluation ? structuredClone(snapshot.simulation.taskEvaluation) : null,
      physicalAuthority: snapshot.simulation?.physicalAuthority ? structuredClone(snapshot.simulation.physicalAuthority) : null,
    };
  };

  Object.defineProperty(facade, '__physicalFacadeInstalled', { value: true, enumerable: false });
  return facade;
}
