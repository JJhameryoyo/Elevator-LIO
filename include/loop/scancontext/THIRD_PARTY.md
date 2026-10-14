Scan Context descriptor, ring/sector keys, alignment and KD-tree code are
ported from `mid360-campus-slam-main/src/sc_liorf_localization` in this workspace.
The source package's MIT notice is preserved in `LICENSE`; individual author
notices in nanoflann, its adaptor and tictoc are retained.

Adaptations: remove ROS/cv_bridge/OpenCV includes, guard nonfinite points and
empty sectors, use atan2 for axis/origin points, cap KNN count by available
history, and build the search tree from the eligible time/height subset.
The backend adapts the source package's distance + Scan Context + ICP + Cauchy
BetweenFactor + iSAM2 flow to Elevator-LIO keyframes and coordinate frames.
