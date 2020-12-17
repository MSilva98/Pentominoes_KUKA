float64 feederCapacity # m
float64 consumptionRate # m/sec
# Minimum level for robot to feed (below this level human intervention is required) %
uint8 minLevelToFeed
# Maximum level for robot to feed (feeder cannot hold a new complete pile) %
uint8 maxLevelToFeed
# Level below which two piles can be feeded %
uint8 levelToDoubleFeed