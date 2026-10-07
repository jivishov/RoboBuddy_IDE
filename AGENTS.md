# RoboBuddy IDE v0.2.0 Guardrails

## Repository targets

- `jivishov/RoboBuddy_IDE` is the application and deployment write target. Emil explicitly authorized replacing its original application with the complete v0.2.0 application.
- `jivishov/RoboBuddy_IDE_v020` is the source of the replacement and is read-only for this transfer.
- `jivishov/RoboBuddy_AI` remains read-only.
- Do not create branches, tags, issues, pull requests, deployments, releases, or settings changes in either read-only repository.

## Physics rules

- Physical mode must derive motion from declared dynamics, contacts, constraints, and controllers.
- Do not directly overwrite root pose/velocity, object velocity, or joint state to create successful task outcomes.
- Do not use hidden grasp welding, synthetic successful kicks, boundary clamping-as-physics, or reset-as-recovery.
- Setup/reset interventions must be explicit and logged.
- Rendering must not be the authoritative physical clock.
- WebMCP remains an opt-in bounded tool layer; it is not the physics transport.

## Evidence labels

Keep execution backend, capability, and evidence separate. Never claim hardware fidelity without named measurements and scope.
