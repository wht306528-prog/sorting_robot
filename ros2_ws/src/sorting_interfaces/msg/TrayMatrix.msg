# Full vision output for the three-tray sorting workspace.
#
# Normal frame content:
# - 3 trays
# - 50 cells per tray
# - 150 TrayCell records total

std_msgs/Header header

# Monotonic frame id from the vision side. This is useful when debugging
# retransmission, dropped frames, and F407 matrix updates.
uint32 frame_id

# Full list of detected cells. Expected length is 150.
TrayCell[] cells
