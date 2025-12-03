ON_CAR = False
MODE_INT = 1 # ["TIME_TRIAL", "OVERTAKE"]
SPEED_MODE_INT = 1 # ["FAST", "MEDIUM"], choose between sectors/speed_sectors TODO: add choosing pp config too

CAR_NAME = "car_8"


class DriveMode:
    STOP = 0
    PP = 1
    FTG = 2

class Sectors:
    FREE = 0
    MID = 1
    DANGER = 2


# Racelines are the paths that the overtaker can choose between
RACELINES = ["mincurve", "mincurve", "mindist_boundary", "mindist", "mincurve", "demoline", "center", "wide"]

# Racelines in the order the overtaker can choose between
RACELINES_IN_ORDER = ["mincurve"]

OBSTACLE_WINDOW = 10
RACELINE_LOOKAHEAD = 3.0
SAFETY_DISTANCE = 0.4

POINT_GROUP_DISTANCE = 0.1
MIN_POINT_GROUP_SIZE = 5

CC_DISTANCE = 1

if ON_CAR:
    PATH_FOLDER = "nvidia"
else:
    PATH_FOLDER = "volta"

MODES = ["TIME_TRIAL", "OVERTAKE"]
MODE = MODES[MODE_INT]

SPEED_MODES = ["FAST", "MEDIUM"]
SPEED_MODE = SPEED_MODES[SPEED_MODE_INT]
