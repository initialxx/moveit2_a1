"""Names shared by single/dual A1 applications; no ROS or SDK initialization."""

JOINTS = [f"joint{i}-a1_r" for i in range(1, 8)]
LEFT_JOINTS = [f"joint{i}-a1_l" for i in range(1, 8)]

PLANNING_PIPELINES = ("ompl", "chomp", "stomp")
OMPL_PLANNERS = (
    "RRTConnect", "RRTstar", "PRM", "EST", "LBKPIECE", "BiTRRT", "AnytimePathShortening",
)
