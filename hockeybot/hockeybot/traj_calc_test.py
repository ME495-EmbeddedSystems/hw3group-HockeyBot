import numpy as np
import matplotlib.pyplot as plt
import random
import time


def traj_puck(c1,c2):
    """
    Args:
        + c1, c2 - x and y coordinates of the puck at two different locations
    Returns:
        + c - y axis intersection
        + m - Trajectory slope
    """
    if c1[1] == c2[1]:
        m = 0
    else:
        m = (c1[1]-c2[1])/(c1[0]-c2[0])

    c = c2[1]-m*c2[0]

    return c, m

def play_waypoints():
    """
    Calculates the x coordinates of the two waypoints for the robot

    Returns:
        + wx1, wx2 - x coordinates fot robots two waypoints
    """
    wx2 = (wy2-c)/m
    wx1 = (wy1-c)/m
    return wx1,wx2


# Planar workspace of arm relative to robot base while playing
xmin = -0.3
xmax = 0.29
ymin = 0.44
ymax = 0.71

# Airhockey table 4 corners
Table_xmin = -0.35
Table_xmax = 0.35
Table_ymin = 0.30
Table_ymax = 1.85

# Robot arm waypoints 1 and 2 y-values
wy1 = 0.45
wy2 = 0.7

# Two puck center positions
p1 = np.array([0.22,1.6])
p2 = np.array([0.19,1.5])
p1 = np.array([0.2,1.6])
p2 = np.array([0.2,1.5])

not_straight_traj = 0
Slope_Zero = 0
# Random puck centre coordinates
# p1 = np.array([0.0,0.0])
# p2 = np.array([0.0,0.0])
# p1[0] = random.uniform(-0.25,0.25)
# p2[0] = random.uniform(-0.25,0.25)
# p1[1] = random.uniform(1.2,1.6)
# p2[1] = random.uniform(1.2,1.6)

# p1[0] = p2[0] situation where x1 = x2 then m = infinity (Slope)
if p1[0] == p2[0]:
    Slope_Zero = 1
    wx1 = p1[0]
    wx2 = wx1
    m = 0
    c = np.inf
else:
    c, m = traj_puck(p1,p2)

    wx1, wx2 = play_waypoints()

    if wx1 > xmin and wx1 < xmax:
        if wx2 > xmin and wx2 < xmax:
            print(f"wx1 & wx2 inside workspace!!")
            # not_straight_traj = 1
    else:
        print(f"wx1 or wx2 outside workspace!!")

print(f"\n m = {m}, c = {c}, wx1 = {wx1}, wx2 = {wx2}")


# Airhockey table visualization
# Trajectory - Y-axis intercect
plt.plot([0],[0.405], 'o', color = 'pink', label = 'Robot home location')
# Two puck points from CV
plt.plot([p1[0],p2[0]],[p1[1],p2[1]], 'ro', color = 'red', label = 'CV puck centres')
# Intersect at boundary W2 & W1
plt.plot([wx1,wx2],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
# Trajectory - Y-axis intercect
plt.plot([0],[c], 'x', color = 'red', label = 'Traj intercect with y-axis')
# Puck trajectory line
if Slope_Zero == 0:
    plt.axline((p1[0], p1[1]), slope=m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')
else:
    plt.plot([p1[0],wx1],[p1[1],wy1], color = 'blue', linestyle=(0, (5, 5)), label = 'Puck trajectory line')
# Table boundaries
plt.plot([Table_xmin,Table_xmin],[Table_ymin,Table_ymax], color="black", label = 'Table boundary')
plt.plot([Table_xmax,Table_xmax],[Table_ymin,Table_ymax], color="black")
plt.plot([Table_xmin,Table_xmax],[Table_ymin,Table_ymin], color="black")
plt.plot([Table_xmin,Table_xmax],[Table_ymax,Table_ymax], color="black")
# Robot workspace
plt.plot([xmin,xmin],[ymin,ymax], color="orange", label = 'Robot workspace')
plt.plot([xmax,xmax],[ymin,ymax], color="orange")
plt.plot([xmin,xmax],[ymin,ymin], color="orange")
plt.plot([xmin,xmax],[ymax,ymax], color="orange")
# Center lines of table
plt.plot([0,0],[Table_ymin,Table_ymax], '--', color = 'grey', label = 'Table center lines') # Center y-axis
plt.plot([Table_xmin,Table_xmax],[(Table_ymax+Table_ymin)/2,(Table_ymax+Table_ymin)/2], '--', color = 'grey') # Center x-axis
# Robot axis and plot axis
plt.plot([0,0],[0,0.2], color = 'black', label = 'Robot axis') # Robot y-axis
plt.plot([0,0.2],[0,0], color = 'black') # Robot x-axis
plt.axis([-1, 1, -0.1, 1.9]) # Window size
# plt.legend(loc="upper right")
plt.ylabel("Robot Y-axis ")
plt.xlabel("Robot X-axis ")
plt.show()