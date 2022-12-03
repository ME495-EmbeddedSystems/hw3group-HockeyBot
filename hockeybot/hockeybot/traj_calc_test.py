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

def play_waypoints(c,m):
    wx2 = (wy2-c)/m
    wx1 = (wy1-c)/m
    return wx1,wx2


# Workspace of arm
xmin = -0.3
xmax = 0.3
ymin = 0.42 # Changed from 0.44 to 0.42 TODO
ymax = 0.70

# Puck Radius
Puck_radius = 0.063/2

# Airhockey table 4 corners [meters] - Minus this with puck radius for
# collision detection - Centre of Puck movement space
Table_xmin = -(0.345 - Puck_radius)
Table_xmax = 0.345 - Puck_radius
Table_ymin = 0.3683 + Puck_radius
Table_ymax = 1.7526 - Puck_radius
# Real table inside
Real_Table_xmin = -0.345
Real_Table_xmax = 0.345
Real_Table_ymin = 0.3683
Real_Table_ymax = 1.7526

# Boundary & waypoint w1&2 y values
wy1 = ymin
wy2 = ymax
"""NEW"""
wx1 = np.array([])
wx2 = np.array([])
wx1_CORRECT = 0
wy1_CORRECT = ymin
wx2_CORRECT = 0
wy2_CORRECT = ymax

# Two puck center positions
# p1 = np.array([0.22,1.6])
# p2 = np.array([0.19,1.5])

p1 = np.array([0.0,0.0])
p2 = np.array([0.0,0.0])

""" New Var """
# Amount of impacts
impact_count = 0
# y-axis intercept
c_list = np.array([])
# Slope
m_list = np.array([])
# After first impact
ImpX = np.array([0]) # Impact point X
ImpY = np.array([0]) # Impact point Y
# Added 0 to start just to make same length as m_list
# Flag for collision in the workspace
Flag_collision_WS = 0
y_ws_impact_Right = 0
y_ws_impact_Left = 0

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

        # Test cases
        # p1 = np.array([-0.18848623, 1.32055732])
        # p2 = np.array([0.0948983, 1.43185305])
        # p1 = np.array([-0.110167, 1.2817489])
        # p2 = np.array([0.08224015, 1.39055462])
        # p1 = np.array([-0.11210456, 1.44002429])
        # p2 = np.array([0.22423767, 1.25521714])
        # p1 = np.array([0.24935121, 1.40058753])
        # p2 = np.array([-0.13393464, 1.44759295])
        print(f"\n p1 = {p1}, p2 = {p2} \n")

        # p1[0] = p2[0] # TODO handle situation where x1 = x2 then m = infinity (Slope)

        c, m = traj_puck(p1,p2)
        m_list = np.append(m_list, m)
        c_list = np.append(c_list, c)
        mf = m
        cf = c
        # print(f"c = {c}, m = {m}")
        # print(f"\n 1 clist = {c_list}, 1 m_list = {m_list} \n")
        # x1, x2 = play_waypoints(cf,mf)
        # wx1 = np.append(wx1, x1)
        # wx2 = np.append(wx2, x2)





        """ Do wall collision."""
        collision = True
        while collision == True:

            if impact_count == 30:
                break

            y_xmin = mf*Table_xmin + cf
            y_xmax = mf*Table_xmax + cf

            x1, x2 = play_waypoints(cf,mf)
            wx1 = np.append(wx1, x1)
            wx2 = np.append(wx2, x2)

            if Flag_collision_WS == 1:
                break # Immediately stops while loop

            # print(f"y_xmin = {y_xmin}, y_xmax = {y_xmax}")

            # print(f"\n clist = {c_list}, m_list = {m_list} \n")


            if y_xmin < Table_ymax and y_xmin > Table_ymin and y_xmin < y_xmax:

                # Check for collisions in the ws
                if y_xmin < ymax and y_xmin > ymin and y_xmin < y_xmax:
                    # ImpX[impact_count]
                    # ImpY[impact_count]
                    # Calculate new y in workspace
                    y_ws_impact_Left = xmin*m_list[impact_count] + c_list[impact_count]
                    # print(f"y_ws left = {y_ws_impact_Left}")

                    # y_xmin

                    # break
                    collision = False

                    # print("Left collision in ws")
                    Flag_collision_WS = 1
                else:
                    impact_count += 1

                    m_list = np.append(m_list,-m_list[impact_count-1]) # Reflect impact angle
                    # print(f"\n clist = {c_list}, m_list = {m_list} \n")
                    # Impact coordinates
                    ImpX = np.append(ImpX, Table_xmin)
                    ImpY = np.append(ImpY, y_xmin)
                    # Call trajectory calculation for new trajectory y-axis intercept
                    c_list = np.append(c_list, after_impact_traj_puck([ImpX[impact_count], ImpY[impact_count]],m_list[impact_count]))
                    # Set current c,m equal to final cf and mf
                    cf = c_list[impact_count]
                    mf = m_list[impact_count]

                    # print(" Wall collision left")
                    collision = True # TODO: Add back in so that collisions will keep on be corrected
            elif y_xmax < Table_ymax and y_xmax > Table_ymin and y_xmax < y_xmin:

                if y_xmax < ymax and y_xmax > ymin and y_xmax < y_xmin:

                    # ImpX[impact_count]
                    # ImpY[impact_count]
                    # Calculate new y in workspace
                    y_ws_impact_Right = xmax*m_list[impact_count] + c_list[impact_count]
                    # print(f"y_ws right = {y_ws_impact_Right}")

                    # y_xmin

                    # break
                    collision = False

                    print("Rigth collision in ws")
                    Flag_collision_WS = 1
                else:
                    impact_count += 1

                    m_list = np.append(m_list,-m_list[impact_count-1]) # Reflect impact angle
                    # print(f"\n clist = {c_list}, m_list = {m_list} \n")
                    # Impact coordinates
                    ImpX = np.append(ImpX, Table_xmax)
                    ImpY = np.append(ImpY, y_xmax)
                    # Call trajectory calculation for new trajectory y-axis intercept
                    c_list = np.append(c_list, after_impact_traj_puck([ImpX[impact_count], ImpY[impact_count]],m_list[impact_count]))
                    # Set current c,m equal to final cf and mf
                    cf = c_list[impact_count]
                    mf = m_list[impact_count]


                    # print(" Wall collision right")
                    collision = True # TODO: Add back in so that collisions will keep on be corrected

            else: # No collision
                collision = False


        print(f"wx1 = {wx1}, wx2 = {wx2}")
        # print(f"clist = {c_list}, m_list = {m_list}")

        # Select best waypoint x values to publish
        for i in range(len(wx1)):
            if wx1[i] < xmax and wx1[i] > xmin:
                for j in range(len(wx2)):
                    if wx2[j] < xmax and wx2[j] > xmin:
                        wx1_CORRECT = wx1[i]
                        wx2_CORRECT = wx2[i]
                        # print(f" wx1[{i}] = {wx1[i]} ")
                        # print(f" wx2[{j}] = {wx2[j]} ")





        if wx1[impact_count] > xmin and wx1[impact_count] < xmax:
            if wx2[impact_count] > xmin and wx2[impact_count] < xmax:
                print(f"wx1 & wx2 inside workspace!!")
                not_straight_traj = 1
        else:
            # TODO send home blocking config
            print(f"wx1 or wx2 outside workspace!!")

        # Airhockey table visualization
        # plt.ion() # Interactive plot
        # Trajectory - Y-axis intercect
        plt.plot([0],[0.405], 'o', color = 'pink', label = 'Robot home location')
        # Two puck points from CV
        plt.plot([p1[0],p2[0]],[p1[1],p2[1]], 'ro', color = 'red', label = 'CV puck centres')

        # # TODO NB GOOD FOR DEBUGGIN
        # """ Plot all wx intersectiona """
        # # Intersect at boundary W2 & W1
        # plt.plot([wx1,wx2],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
        # """ Plot last wx intersectiona """
        # # Intersect at boundary W2 & W1
        # plt.plot([wx1[-1],wx2[-1]],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')

        if wx1[impact_count] > xmin and wx1[impact_count] < xmax:
            if wx2[impact_count] > xmin and wx2[impact_count] < xmax:
                # print(f"wx1 & wx2 inside workspace!!")
                not_straight_traj = 1
        else:
            # print(f"wx1 or wx2 outside workspace!!")
            pass

        # """ Plot best wx intersection """
        for i in range(len(wx2)):
            if wx2[i] < xmax and wx2[i] > xmin:
                # print(f" wx2[{i}] = {wx2[i]} ")
                if y_ws_impact_Right == 0 and y_ws_impact_Left == 0:
                    for j in range(len(wx1)):
                        # print(f" wx1[{j}] = {wx1[j]} ")
                        if wx1[j] < xmax and wx1[j] > xmin:
                            # Intersect at boundary W2 & W1
                            plt.plot([wx1[j],wx2[i]],[wy1,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                            print(f" w1 = {wx1[i],wy1}, w2 = {wx2[j],wy2} ")

                else:
                    if y_ws_impact_Right != 0:
                        # print("RIIIIGHT")
                        plt.plot([xmax,wx2[i]],[y_ws_impact_Right,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                        print(f" w1 = {xmax,y_ws_impact_Right}, w2 = {wx2[i],wy2} ")
                    elif y_ws_impact_Left != 0:
                        # print("LEEEEEFTT")
                        plt.plot([xmin,wx2[i]],[y_ws_impact_Left,wy2], 'ro', color = 'green', label = 'Robot move waypoints')
                        print(f" w1 = {xmin,y_ws_impact_Left}, w2 = {wx2[i],wy2} ")
            else: # Block/Stay in front of goal
                print(f"JUST BLOCK")


        # Trajectory - Y-axis intercect
        plt.plot([0],[c], 'x', color = 'red', label = 'Traj intercect with y-axis')
        # Puck trajectory line
        plt.axline((p1[0], p1[1]), slope=m, color="blue", linestyle=(0, (5, 5)), label = 'Puck trajectory line')

        """New Plot after impact"""
        if len(c_list) != 0:
            for imp in range(1,len(c_list)):
                # Impact point 1
                plt.plot([ImpX[imp]],[ImpY[imp]], 'ro', color = 'blue', label = 'Impact point')
                # New after impact trajectory line
                plt.axline((ImpX[imp],ImpY[imp]), slope=m_list[imp], color="red", linestyle=(0, (5, 5)), label = 'New puck trajectory line')
                # Trajectory - Y-axis intercect
                plt.plot([0],[c_list[imp]], 'x', color = 'red', label = 'Traj1 intercect with y-axis')

        # Table boundaries
        plt.plot([Table_xmin,Table_xmin],[Table_ymin,Table_ymax], '--', color="grey", label = 'Table boundary for puck centre - Collision')
        plt.plot([Table_xmax,Table_xmax],[Table_ymin,Table_ymax], '--', color="grey")
        plt.plot([Table_xmin,Table_xmax],[Table_ymin,Table_ymin], '--', color="grey")
        plt.plot([Table_xmin,Table_xmax],[Table_ymax,Table_ymax], '--', color="grey")
        plt.plot([Real_Table_xmin,Real_Table_xmin],[Real_Table_ymin,Real_Table_ymax], color="black", label = 'Actual Table boundary')
        plt.plot([Real_Table_xmax,Real_Table_xmax],[Real_Table_ymin,Real_Table_ymax], color="black")
        plt.plot([Real_Table_xmin,Real_Table_xmax],[Real_Table_ymin,Real_Table_ymin], color="black")
        plt.plot([Real_Table_xmin,Real_Table_xmax],[Real_Table_ymax,Real_Table_ymax], color="black")
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