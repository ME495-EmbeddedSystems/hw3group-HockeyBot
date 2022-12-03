import numpy as np
import matplotlib.pyplot as plt
import random
import time


def traj_puck(c1,c2):
    # y = mx+c
    if c1[1] == c2[1]:
        m = 0
    else:
        m = (c1[1]-c2[1])/(c1[0]-c2[0])

    c = c2[1]-m*c2[0] # y intersection
    return c, m

def after_impact_traj_puck(c1,m):
    """ Traj calc after impact."""
    c = c1[1]-m*c1[0] # y intersection
    return c

def play_waypoints():
    wx2 = (wy2-c)/m
    wx1 = (wy1-c)/m
    return wx1,wx2


# Workspace of arm
xmin = -0.3
xmax = 0.29
ymin = 0.44
ymax = 0.71

# Airhockey table 4 corners [meters] TODO: Minus this with puck radius for
# collision detection
Table_xmin = -0.32
Table_xmax = 0.32
Table_ymin = 0.3683
Table_ymax = 1.7526

# Boundary w1&2 y values
wy1 = 0.45
wy2 = 0.7

# Two puck center positions
# p1 = np.array([0.22,1.6])
# p2 = np.array([0.19,1.5])

p1 = np.array([0.0,0.0])
p2 = np.array([0.0,0.0])
# After first impact
Imp1 = np.array([0.0,0.0]) # Impact point
# After second impact
Imp2 = np.array([0.0,0.0]) # Impact point
# After third impact
Imp3 = np.array([0.0,0.0]) # Impact point

""" Select trajectory type:
    + sim:
        0 - Collsion with side allowed
        1 - No collision with side allowed
"""
sim = 0

if sim == 0:

        p1[0] = random.uniform(-0.25,0.25)
        p2[0] = random.uniform(-0.25,0.25)

        p1[1] = random.uniform(1.2,1.6)
        p2[1] = random.uniform(1.2,1.6)

        # p1[0] = p2[0] # TODO handle situation where x1 = x2 then m = infinity (Slope)

        c, m = traj_puck(p1,p2)





        """ Do wall collision."""
        collision = True
        while collision == True:
            collision = False # TODO Delete later

            y_xmin = m*Table_xmin + c
            y_xmax = m*Table_xmax + c
            print(f"y_xmin = {y_xmin}, y_xmax = {y_xmax}")


            if y_xmin < Table_ymax and y_xmin > Table_ymin and y_xmin < y_xmax:
                m1 = -m # Reflect impact angle
                # Impact coordinates
                Imp1[0] = Table_xmin
                Imp1[1] = y_xmin

                # Generating another point by reflecting one of the previous trajectory points
                # Imp2[0] = p1[0]
                # Imp2[1] = p1[1] - 2*(p1[1] - Imp1[1])

                # Call trajectory calculation for new trajectory y-axis intercept
                c2 = after_impact_traj_puck(Imp1,m1)


                print(" Wall collision left")
                # collision = True # TODO: Add back in so that collisions will keep on be corrected
            elif y_xmax < Table_ymax and y_xmax > Table_ymin and y_xmax < y_xmin:
                m1 = -m # Reflect impact angle
                # Impact coordinates
                Imp1[0] = Table_xmax
                Imp1[1] = y_xmax

                # Generating another point by reflecting one of the previous trajectory points
                # Imp2[0] = p1[0]
                # Imp2[1] = p1[1] - 2*(p1[1] - Imp1[1])

                # Call trajectory calculation for new trajectory y-axis intercept
                c2 = after_impact_traj_puck(Imp1,m1)

                print(" Wall collision right")
                # collision = True # TODO: Add back in so that collisions will keep on be corrected
            else: # No collision
                collision = False







        wx1, wx2 = play_waypoints()

        if wx1 > xmin and wx1 < xmax:
            if wx2 > xmin and wx2 < xmax:
                print(f"wx1 & wx2 inside workspace!!")
                not_straight_traj = 1
        else:
            print(f"wx1 or wx2 outside workspace!!")

        # if wx1 > xmin and wx1 < xmax:
        #     print(f"wx1 inside workspace!!")
        #     not_straight_traj
        # else:
        #     print(f"wx1 outside workspace!!")

        # if wx2 > xmin and wx2 < xmax:
        #     print(f"wx2 inside workspace!!")
        #     not_straight_traj
        # else:
        #     print(f"wx2 outside workspace!!")

        print(f"\n m = {m}, c = {c}, wx1 = {wx1}, wx2 = {wx2}")



        # Airhockey table visualization
        # plt.ion() # Interactive plot
        # Trajectory - Y-axis intercect
        plt.plot([0],[0.405], 'o', color = 'pink', label = 'Robot home location')
        # Two puck points from CV
        plt.plot([p1[0],p2[0]],[p1[1],p2[1]], 'ro', color = 'red', label = 'CV puck centres')
        # Intersect at boundary W2 & W1
        plt.plot([wx1,wx2],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
        # Trajectory - Y-axis intercect
        plt.plot([0],[c], 'x', color = 'red', label = 'Traj intercect with y-axis')
        # Puck trajectory line
        plt.axline((p1[0], p1[1]), slope=m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')


        # Impact point 1
        plt.plot([Imp1[0]],[Imp1[1]], 'ro', color = 'blue', label = 'Impact point')
        # New after impact trajectory line
        plt.axline((Imp1[0],Imp1[1]), slope=m1, color="red", linestyle=(0, (5, 5)), label = 'New puck trajectory line')
        # Trajectory - Y-axis intercect
        plt.plot([0],[c2], 'x', color = 'red', label = 'Traj1 intercect with y-axis')
        # # New reflection generated point
        # plt.plot([Imp2[0]],[Imp2[1]], 'ro', color = 'blue', label = 'New reflection generated point')

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

        # plt.pause(1.0) # pause for 1 second
        # plt.clf() # Clear the plot to not show previous plot











elif sim == 1:
    # not_straight_traj = 0
    # i = 0

    while True:

        not_straight_traj = 0
        # time.sleep(1)
        # plt.close('all')

        while not_straight_traj == 0:
        # if i < 3:
        #     i += 1
            p1[0] = random.uniform(-0.25,0.25)
            p2[0] = random.uniform(-0.25,0.25)

            p1[1] = random.uniform(1.2,1.6)
            p2[1] = random.uniform(1.2,1.6)

            # p1[0] = p2[0] # TODO handle situation where x1 = x2 then m = infinity (Slope)

            c, m = traj_puck(p1,p2)

            wx1, wx2 = play_waypoints()

            if wx1 > xmin and wx1 < xmax:
                if wx2 > xmin and wx2 < xmax:
                    print(f"wx1 & wx2 inside workspace!!")
                    not_straight_traj = 1
            else:
                print(f"wx1 or wx2 outside workspace!!")

            # if wx1 > xmin and wx1 < xmax:
            #     print(f"wx1 inside workspace!!")
            #     not_straight_traj
            # else:
            #     print(f"wx1 outside workspace!!")

            # if wx2 > xmin and wx2 < xmax:
            #     print(f"wx2 inside workspace!!")
            #     not_straight_traj
            # else:
            #     print(f"wx2 outside workspace!!")

            print(f"\n m = {m}, c = {c}, wx1 = {wx1}, wx2 = {wx2}")



        # Airhockey table visualization
        plt.ion() # Interactive plot
        # Trajectory - Y-axis intercect
        plt.plot([0],[0.405], 'o', color = 'pink', label = 'Robot home location')
        # Two puck points from CV
        plt.plot([p1[0],p2[0]],[p1[1],p2[1]], 'ro', color = 'red', label = 'CV puck centres')
        # Intersect at boundary W2 & W1
        plt.plot([wx1,wx2],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
        # Trajectory - Y-axis intercect
        plt.plot([0],[c], 'x', color = 'red', label = 'Traj intercect with y-axis')
        # Puck trajectory line
        plt.axline((p1[0], p1[1]), slope=m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')
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
        # plt.show()

        plt.pause(1.0) # pause for 1 second
        plt.clf() # Clear the plot to not show previous plot