#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion
import math
from nav_msgs.msg import Odometry
import numpy as np

x_initial = 0
y_initial = 0
x_current = 0
y_current = 0
roll = 0
pitch = 0
yaw = 0
starting = True
x_final = 3
y_final = 4
grid_size = 0.5
robot_radius = 5.0
target_reached = False
obstacle_x = [-4, -2, 1, 1.4, 0]
obstacle_y = [-1, -3.49, -3.45, -1, 1.53]
area_width = 30.0
kp = 5.0
eta = 100.0
minx = min(obstacle_x)-area_width/2.0
miny = min(obstacle_y)-area_width/2.0
maxx = max(obstacle_x)+area_width/2.0
maxy = max(obstacle_y)+area_width/2.0
xw = int(round((maxx-minx)/grid_size))
yw = int(round((maxy-miny)/grid_size))
print(xw, yw)
potential_map_size = max(xw, yw)
potential_field_map = np.zeros((max(xw, yw), max(xw, yw)))
print(potential_field_map[0, 0])
print(potential_field_map[1, 2])


def potential_field_planning():
    global x_final, y_final, obstacle_x, obstacle_y, grid_size, robot_radius, potential_field_map
    # calculate potential field
    potential_field_map, minx, miny = calculate_potential_field(
        x_final, y_final, obstacle_x, obstacle_y, grid_size, robot_radius)
    print(potential_field_map)
    path_matrix_x, path_matrix_y = pathfinder(
        potential_field_map, x_initial, x_final, y_initial, y_final, minx, miny)
    # TODO: Publish velocity command to path_x,path_y
    print(path_matrix_x, " : ", path_matrix_y)
    print("Target reached")


def pathfinder(potential_field_map, x_initial, x_final, y_initial, y_final, minx, miny):
    path_x=[]
    path_y=[]
    distance=np.hypot(x_initial-minx,y_initial-minx)
    #TODO: replace with x_current
    x_in_grid = int(round((x_initial-minx)/grid_size))
    y_in_grid =int(round((y_initial-miny)/grid_size))
    x_target = int(round((x_final-minx)/grid_size))
    y_target = int(round((y_final-miny)/grid_size))
    p=potential_field_map[x_in_grid,y_in_grid]
    path_x.append(x_initial)
    path_y.append(y_initial)
    motion=get_motion_model()
    while distance>=1:
        for i in range(len(motion)):
            #print(motion[i])
            new_x_in_grid=int(x_in_grid+motion[i][0])
            new_y_in_grid=int(y_in_grid+motion[i][1])
            if new_x_in_grid>=potential_map_size or new_y_in_grid>=potential_map_size:
                new_p=float("inf")
            else:                
                new_p=potential_field_map[new_x_in_grid,new_y_in_grid]
            #print(new_p)
            if new_p<p:
                p=new_p
                x_in_grid=new_x_in_grid
                y_in_grid=new_y_in_grid
        
        x_in_field=x_in_grid*grid_size+minx
        y_in_field=y_in_grid*grid_size+miny
        distance=np.hypot(x_in_field-minx,y_in_field-minx)
        print(distance)
        path_x.append(x_in_grid)
        path_y.append(y_in_grid)

    return path_x,path_y

    '''
    while distance>=1:
        minp=float("inf")
        minix,miniy=-1,-1
        for i in range(len(motion)):
            inx=int(ix+motion[i][0])
            iny=int(iy+motion[i][1])
            if inx>=potential_map_size or iny >=potential_map_size:
                p=float("inf")
            else:
                p=potential_field_map[inx][iny]
            if minp >p:
                minp=p
                minix=inx 
                miniy=iny
        ix=minix
        iy=miniy
        xp=ix*grid_size+minx
        yp=iy*grid_size+miny
        distance=np.hypot(x_final-xp,y_final-yp)
        path_x.append(xp)
        path_y.append(yp)
    '''
    '''
    global grid_size,potential_map_size
    distance = np.hypot(x_initial-x_final, y_initial-y_final)
    ix = round((x_initial-minx)/grid_size)
    iy = round((y_initial-miny)/grid_size)
    #goal_ix = round((x_final-minx)/grid_size)
    #goal_iy = round((y_final-miny)/grid_size)

    path_x, path_y = [x_initial], [y_initial]
    motion_matrix = get_motion_model()
    while distance >= grid_size:
        minp = float("inf")
        minix, miniy = -1, -1
        for i in range(len(motion_matrix)):
            index_x = int(ix+motion_matrix[i][0])
            index_y = int(iy+motion_matrix[i][1])
            #print(potential_field_map[1,2])
            if index_x >= potential_map_size or index_y>=potential_map_size:
                p = float("inf")
            else:
                print(index_x,index_y, distance)
                #print(potential_field_map[index_x,index_y])
                p = potential_field_map[index_x,index_y]
            if minp > p:
                minp = p
                minix = index_x
                miniy = index_y
        ix = minix
        iy = miniy

        xp = ix*grid_size+minx
        yp = iy*grid_size+miny
        distance = np.hypot(x_final-xp, y_final-yp)
        #print(distance,":",grid_size)
        path_x.append(xp)
        path_y.append(yp)
    '''
    #return path_x, path_y
       
    


def get_motion_model():
    motion = [[1, 0], [0, 1], [-1, 0], [0, -1],
              [-1, -1], [-1, 1], [1, -1], [1, 1]]
    return motion


def calculate_potential_field(x_final, y_final, obstacle_x, obstacle_y, grid_size, robot_radius):
    global xw, minx, yw, miny
    local_potential_map = np.zeros((max(xw, yw), max(xw, yw)))
    for i in range(xw):
        x = i*grid_size+minx
        for j in range(yw):
            y = j*grid_size+miny
            p_attractive = attractive_potential(x, y, x_final, y_final)
            p_repulsive = repulsive_potential(x, y, obstacle_x, obstacle_y)
            total_potential = p_attractive+p_repulsive
            local_potential_map[i][j] = total_potential

    return local_potential_map, minx, miny


def attractive_potential(x, y, x_final, y_final):
    global kp
    Ua = 0.5*kp*np.hypot(x-x_final, y-y_final)
    return Ua


def repulsive_potential(x, y, obstacle_x, obstacle_y):
    global robot_radius, eta
    # find nearest obstacle
    minimum_ob_index = -1
    distance_ob_minimum = float("inf")
    for i in range(len(obstacle_x)):
        distance_current = np.hypot(x-obstacle_x[i], y-obstacle_y[i])
        if distance_ob_minimum >= distance_current:
            distance_ob_minimum = distance_current
            minimum_ob_index = i
    # repulsive potential
    Ur = np.hypot(x-obstacle_x[minimum_ob_index],
                  y-obstacle_y[minimum_ob_index])
    if Ur <= robot_radius:
        if Ur <= 0.2:
            Ur = 0.2
        return 0.5*eta*pow(((1/Ur) - (1/robot_radius)), 2)
    else:
        return 0


def callback(msg):
    """
    global x_current, x_initial, y_current, y_initial
    # print(x_current)
    # print("-------------------------------------------")
    # print(y_current)
    # print("*"*30)

    global target_reached
    # print(target_reached)
    if(target_reached is False):

        if abs(x_current-x_initial) >= 5 or abs(y_current-y_initial) >= 5:
            move.angular.z = 0.5
            move.linear.x = 0.1
            #TODO:pub.publish(move)
        else:
            target_angle = math.atan2(y_final-y_current, x_final-x_current)
            #TODO:print("Yaw - ", yaw, " Angle - ", target_angle,"Current pos -", x_current, " ", y_current)
            if abs(target_angle-yaw) >= 1:
                move.angular.z = 0.5
                move.linear.x = 0

                #TODO:pub.publish(move)
            else:
                if msg.ranges[0] > 2:

                    move.linear.x = 0.5
                    move.angular.z = 0.0
                    #TODO:pub.publish(move)
                '''
                print("-------------------------------------------")
                print("Number of ranges: ", len(msg.ranges))
                print("Reading at position 0:", msg.ranges[0])
                print("-------------------------------------------")
                '''

                #turn_angle = LaserScan.angle_min + ((move.linear.x) * LaserScan.angle_increment)

                if msg.ranges[0] < 2:

                    move.angular.z = 0.5
                    #move.angular.z = turn_angle
                    move.linear.x = 0.1
                    #print ("Alert: object at position ", msg.ranges[0])
                    #TODO:pub.publish(move)

        if abs(x_current-x_final) <= 0.2 and abs(y_current-y_final) <= 0.2:
            move.angular.z = 0
            move.linear.x = 0
            target_reached = True
            print(x_current, ":", y_current)
            rospy.loginfo("Target Reached")
        #TODO:pub.publish(move)
    """


def Position(odom_data):
    """
    # the x,y,z pose and quaternion orientation
    robot_position = odom_data.pose.pose
    global roll, pitch, yaw
    (roll, pitch, yaw) = euler_from_quaternion(
        [robot_position.orientation.x, robot_position.orientation.y, robot_position.orientation.z, robot_position.orientation.w], 'sxyz')
    #curr_time = odom_data.header.stamp
    global starting
    global x_current, x_initial, y_current, y_initial
    if starting is True:
        x_initial = robot_position.position.x
        y_initial = robot_position.position.y
        starting = False

    x_current = robot_position.position.x
    y_current = robot_position.position.y
    """


rospy.init_node('route_map_node')
sub = rospy.Subscriber('/scan', LaserScan, callback)
sub = rospy.Subscriber('/odom', Odometry, Position)
pub = rospy.Publisher('/cmd_vel', Twist,  queue_size=10)
potential_field_planning()
move = Twist()

rospy.spin()
