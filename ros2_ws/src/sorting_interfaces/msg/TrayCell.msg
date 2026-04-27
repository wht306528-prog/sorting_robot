# One detected cell in a seedling tray.
#
# The vision side outputs one TrayCell per tray hole. Three trays produce
# 150 cells in total: 3 trays * 5 columns * 10 rows.

uint8 CLASS_EMPTY=0
uint8 CLASS_GOOD=1
uint8 CLASS_WEAK=2

# Tray index from left to right.
# 1 = left tray, 2 = middle tray, 3 = right tray.
uint8 tray_id

# Tray grid position.
# The top-left hole is row=1, col=1.
# Columns increase to the right: 1..5.
# Rows increase downward: 1..10.
uint8 col
uint8 row

# Class id:
# 0 = empty or no valid seedling
# 1 = good seedling
# 2 = weak or not-emerged seedling
uint8 class_id

# Classification confidence, range 0.0..1.0.
float32 confidence

# Pixel center and depth from the RGB-D image.
# u and v are image pixel coordinates.
# z is depth in millimeters.
float32 u
float32 v
float32 z
