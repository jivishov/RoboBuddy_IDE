"""Source/reference geometry invariants for the standalone OpenArm scene.

These values connect the standalone IDE to RoboBuddy_AI's existing OpenArm V2
reference mission and canonical visual rig. They are not hardware measurements
and must never be used as hardware safety/certification evidence.
"""

WORKTOP_TOP_MM = 320.0
FLASK_GRIP_SOCKET_FROM_BOTTOM_MM = 96.0

# World tool-frame locations independently calibrated from the canonical
# RoboBuddy_AI OpenArm hierarchy with the j7 -> tool offset [0, -168, 0] mm.
CONTACT_TOOL_MM = (309.834, 415.996, -339.979)
LIFT_TOOL_MM = (309.919, 535.857, -339.783)
TRANSFER_TOOL_MM = (439.739, 536.322, -339.345)
PLACE_TOOL_MM = (439.626, 462.329, -339.239)

HOTPLATE_TOP_MM = PLACE_TOOL_MM[1] - FLASK_GRIP_SOCKET_FROM_BOTTOM_MM
HOTPLATE_HEIGHT_MM = HOTPLATE_TOP_MM - WORKTOP_TOP_MM
TRANSPORT_FLASK_BOTTOM_MM = TRANSFER_TOOL_MM[1] - FLASK_GRIP_SOCKET_FROM_BOTTOM_MM

assert abs(CONTACT_TOOL_MM[0] - 310.0) < 0.25
assert abs(CONTACT_TOOL_MM[1] - 416.0) < 0.25
assert abs(CONTACT_TOOL_MM[2] + 340.0) < 0.25
assert LIFT_TOOL_MM[1] - CONTACT_TOOL_MM[1] > 119.0
assert abs(TRANSFER_TOOL_MM[0] - 440.0) < 0.5
assert 45.0 < HOTPLATE_HEIGHT_MM < 48.0
assert TRANSPORT_FLASK_BOTTOM_MM - HOTPLATE_TOP_MM > 70.0

# Public LeRobot OpenArm gripper semantics used by the source mission.
OPEN_GRIPPER_DEG = -65.0
CLOSED_GRIPPER_DEG = 0.0
assert OPEN_GRIPPER_DEG < CLOSED_GRIPPER_DEG

print('OpenArm canonical reference geometry invariants: OK')
print('  canonical contact tool mm:', CONTACT_TOOL_MM)
print('  canonical lift tool mm:', LIFT_TOOL_MM)
print('  canonical transfer tool mm:', TRANSFER_TOOL_MM)
print('  configured hotplate height mm:', HOTPLATE_HEIGHT_MM)
print('  transport clearance above hotplate mm:', TRANSPORT_FLASK_BOTTOM_MM - HOTPLATE_TOP_MM)
