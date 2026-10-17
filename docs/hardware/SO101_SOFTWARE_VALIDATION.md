# SO-101 software validation and hardware handoff

Date: October 17, 2026. Implementation: fix/so101-hardware-panel in jivishov/RoboBuddy_IDE (PR #5).

## Completed without an arm

- Local Windows driver environment: LeRobot 0.6.0, pyserial 3.5, Feetech SDK 1.0.0, deepdiff 8.6.2 and orderly-set 5.5.0. Constructed the real driver with an unopened placeholder port; no serial connection occurred.
- 42 Python checks passed locally: controller state transitions, read-only connect, measured goals before torque, fresh sampling after mode selection, lease expiry during diagnostics, bounds, calibration, fault/close behavior, HTTP authentication/origin/Host rules, and adapter/mapping comparisons using the actual released LeRobot classes with a replaced wire.
- JavaScript mapping/loopback checks passed. Priority Stop, mutation serialization and stale-session response checks passed.
- Windows setup and launcher syntax passed. The real helper launcher started successfully; its authenticated state was hardware mode, disconnected, no operator and no serial bus. Starting it does not connect the arm.
- The initial full-repository CI run 39076955099 passed on Windows and Linux and passed the Chromium demo panel test. PR checks are the authoritative status for the final revision, including the strengthened canvas/always-visible Stop checks.

## Current feature

A Hardware entry opens a dedicated manual panel. Local helpers pair their own page automatically. Real and demo modes are separate server launch choices; demo never imports or opens a serial motor connection. The runtime supports explicit read-only connect, supported-arm torque enable/disable, bounded joint/gripper jogs, guided calibration, saved-calibration restore and encoder-driven illustrative geometry. Targets, samples and measured torque state remain separate. Stop is available in the sticky header as well as the monitor card.

The simulator, user Python, Reset and WebMCP have no hardware handle. Hardware code does not call SO101Follower.connect/configure or its torque-disabled configuration context. Closing does not silently disable torque. The bridge uses an expiring operator lease, monotonic target scheduling and no motion queue.

## Still intentionally pending

Actual controller/motor identity and power rating, USB handshake, physical calibration, response of all six motors, true stopping latency, failure behavior of the assembled controller, cable clearance and visual zero/direction alignment have not been tested. Passing demo/software checks does not establish these facts. Follow the final commissioning procedure in SO101_MANUAL_CONTROL.md beside the supported arm before treating it as hardware-tested.

