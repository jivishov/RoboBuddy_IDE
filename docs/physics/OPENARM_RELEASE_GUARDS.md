# OpenArm observation guard follow-up

The integration review found that filtering contacts for grasp quality also erased them from the WebMCP `released` predicate. A deeply penetrated finger contact could therefore be interpreted as release. A missing/unreadable contact list could produce the same false result. Palm contact was not considered.

The observation interpreter now separates usable grasp contacts from physical or uncertain gripper contact. Release requires a complete readable contact snapshot for a known object, and no active or uncertain contact with either finger or the requested gripper's palm. Excessive penetration invalidates bilateral-grasp evidence rather than manufacturing release. Zero-force geometric touch conservatively prevents release; a positive gap with zero load does not. The predicate is scoped to the selected gripper and does not assert that the object was previously held.

`graspState` additionally exposes `anyPalmContact`, `anyGripperContact`, `contactsReadable`, and `excessivePenetrationSeen`. Contact-based program conditions fail closed on incomplete observations. Position predicates reject malformed/nonfinite observed vectors.

`tests/physics/openarm-observation-core.mjs` covers the positive and negative cases and is imported by the existing OpenArm model/schema gate. Run it alone with `node tests/physics/openarm-observation-core.mjs`. The normal OpenArm CI additionally exercises the shipped MuJoCo worker, native nominal/negative/timestep tests, full IDE Python, equipment programs, and Agent Assist access lifecycle.

The `supported` program predicate still means sustained force-bearing contact with the named geometry. By itself it is not a stable-placement or upward-load certificate. The reference task has separate release, position, settling and retreat checks. Equipment controls remain rigid-body simulation, not laboratory-process or hardware validation.
