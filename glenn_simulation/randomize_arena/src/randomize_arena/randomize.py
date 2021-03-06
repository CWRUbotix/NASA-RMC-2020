#!/usr/bin/env python3
import rospy
import numpy
import random 
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion 
from std_srvs.srv import Trigger, TriggerResponse
import os
import png
from scipy.spatial.transform import Rotation


width =  513
height = width

grayscale_bitdepth = 8

sizeX = 6.8
sizeY = 6.8
sizeZ = 0.4

rock_radius = 0.25
rock_z = 0.2

obstacleZoneY = 3.2
arena_length = 6.8
arena_width = 2.5
arena_height = 0.4

obstacle_data_amount = 3
hole_amount = 2 

def intersecting_circles(circle1, circle2):

    d = numpy.sqrt((circle1[0]-circle2[0])*(circle1[0]-circle2[0]) + (circle1[1]-circle2[1])*(circle1[1]-circle2[1]))
    r = circle1[2]+circle2[2]

    return d < r 

def intersecting_multiple_circles(circle_array,circle):

    if(circle_array.ndim == 1):

        return intersecting_circles(circle_array,circle)

    for i in circle_array:

        if(intersecting_circles(i,circle)):
            return True
    
    return False

def random_spot(radius):

    obstacle_circle = numpy.zeros(obstacle_data_amount)

    obstacle_x =  random.uniform(radius,arena_width-radius)
    obstacle_y =  random.uniform(radius,obstacleZoneY-radius)
            
    obstacle_circle[0] = obstacle_x
    obstacle_circle[1] = obstacle_y 
    obstacle_circle[2] = radius  

    return obstacle_circle  

def obstacle_spot_chooser(obstacles,radius):

    print("length of obstacles=%d" % len(obstacles))

    obstacle_length = len(obstacles)

    if(obstacles.ndim == 1):
        obstacle_length = 1
        


    while True:
        
        #print("obstacles in loop")
        #print(obstacles)
    
        obstacle_circle = random_spot(radius)

        #print(obstacles)
        #print(obstacle_circle)

        if( not(intersecting_multiple_circles(obstacles,obstacle_circle))):
            break

    new_obstacles = numpy.zeros((obstacle_length+1,obstacle_data_amount))
   
    for j in range(obstacle_length):
        new_obstacles[j] = obstacles[j]

    new_obstacles[obstacle_length] = obstacle_circle

    print("old obstacles")
    print(obstacles)

    print("obstacle circle")
    print(obstacle_circle)

    print("new obstacles")
    print(new_obstacles)

    return new_obstacles

def pixel_value(holes,x_pixel,y_pixel):

    max_8bit_grayscale_val = pow(2,grayscale_bitdepth)-1
    pixel_color = max_8bit_grayscale_val

    resolutionX = sizeX/width
    resolutionY = sizeY/height

    x_meters = x_pixel*resolutionX - arena_width
    y_meters = sizeY - y_pixel*resolutionY 

    pixel_point = numpy.zeros(obstacle_data_amount)
    pixel_point[0] = x_meters
    pixel_point[1] = y_meters
    pixel_point[2] = 0

    for i in holes:
        if(intersecting_circles(i,pixel_point)):
            '''
            print("within hole")
            print(i[0])
            print(i[1])
            print(i[2])
            '''
            delta_x = x_meters-i[0]
            delta_y = y_meters-i[1]

            min_radius_depth = 0.3
            max_radius_depth = 0.4

            radius_xy = i[2]
            radius_depth = random.uniform(min_radius_depth,max_radius_depth)

            x_term = (delta_x*delta_x)/(radius_xy*radius_xy)
            y_term = (delta_y*delta_y)/(radius_xy*radius_xy)
            z_term = 0

            combined_term = 1 - x_term  - y_term
            '''
            print(delta_x)
            print(delta_y)

            print(radius_x)
            print(radius_y)
            print(radius_depth)

            print(x_term)
            print(y_term)

            print(combined_term)
            '''
            
            z_term =  radius_depth*numpy.sqrt(combined_term)
        
           # print(z_meters)
            pixel_color = int(max_8bit_grayscale_val - z_term*max_8bit_grayscale_val/sizeZ)
            break

   
    return pixel_color

def print_greyscale_png(img):
  height = len(img)
  width = len(img[0])
  print("height=%d" % height)
  print("width=%d" % width)
  for y in range(height):
    for x in range(width):
      #print("x=%d " % x)
      #print("y=%d " % y)
      print("%d " % img[y][x], end='')
    print()
  
def write_greyscale_png(filename, img):
  height = len(img)
  width = len(img[0])
  print("write8bitGreyscalePng height=%d" % height)
  print("write8bitGreyscalePng width=%d" % width)

  f = open(filename, 'wb')
  w = png.Writer(width, height, greyscale=True, bitdepth=grayscale_bitdepth)
  w.write(f, img)
  f.close()

  print("write8bitGreyscalePng wrote %s" % filename)

def janky_gazebo_path(filename,desired_path,folder_name):

    relative_path = '/'

    file_path =__file__
    file_path_array = file_path.split('/')

    for i in file_path_array:

        relative_path = os.path.join(relative_path,i)
        if(i == folder_name):
            break

    return os.path.join(relative_path,desired_path,filename)

#Randomizes the robot state
def randomize_robot(req):
    robot_state = ModelState()
    robot_state.model_name = "robot"
    random_x = uniform(0, 2.5)
    random_y = uniform(0, 6.8)

    robot_state.pose.position = Point(random_x, random_y, 0.2)
    send_state(robot_state)

#Randomizes the rock state 
def randomize_obstacles(req):
    print("randomize obstacle Start")
    rock_1_state = ModelState()
    rock_2_state = ModelState()
    rock_1_state.model_name = "rock_1"
    rock_2_state.model_name = "rock_2"


    #Randomize rock 1 
    one_obstacles = random_spot(rock_radius)

    #Randomize rock 2
    two_obstacles = obstacle_spot_chooser(one_obstacles,rock_radius)

    #Send to gazebo

    points = numpy.zeros(len(two_obstacles),dtype=Point)

    for i in range(len(two_obstacles)):

        points[i] = Point(two_obstacles[i][0],two_obstacles[i][1], rock_z)

    print('Switch from rocks to holes here')

    hole_terrain_state = ModelState()
    hole_terrain_state.model_name = "hole_terrain"
   

    hole_max_radius = 0.2
    hole_min_radius = 0.1

    three_obstacle = obstacle_spot_chooser(two_obstacles,random.uniform(hole_min_radius,hole_max_radius))
    four_obstacle = obstacle_spot_chooser(two_obstacles,random.uniform(hole_min_radius,hole_max_radius))

    holes = numpy.zeros( (hole_amount,obstacle_data_amount) )
    holes[0] = four_obstacle[len(four_obstacle)-2]
    holes[1] = four_obstacle[len(four_obstacle)-1]

    height_map_name = 'height_map.png'
    height_map_path = 'glenn_simulation/glenn_description/models/hole_terrain/materials/textures'
    folder_name = 'NASA-RMC-2020'
    height_map_relative_path = '/'
    
    file_name_and_path = janky_gazebo_path(height_map_name,height_map_path,folder_name)

    img = [[pixel_value(holes,x,y) for x in range(width)] for y in range(height)]
           
    write_greyscale_png(file_name_and_path,img)

    print("wrote new height map")

    rock_1_state.pose.position = points[0]
    rock_2_state.pose.position = points[1]
    send_state(rock_1_state)
    send_state(rock_2_state)

    reset_ground(hole_terrain_state)

#Sends the model state to gazebo
def send_state(model_state):
    rospy.loginfo("Waiting for set model state service...")
    rospy.wait_for_service("/gazebo/set_model_state")

    try:
        service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        res = service(model_state)
        rospy.loginfo("Successfully sent model state for %s"%(model_state.model_name))

    except rospy.ServiceException as e:
        rospy.logerr("Failed to send model state for %s"%(model_state.model_name))
        rospy.logerr(e)

def reset_ground(ground_state):

    height_map_name = 'model.sdf'
    height_map_path = 'glenn_simulation/glenn_description/models/hole_terrain'
    folder_name = 'NASA-RMC-2020'
    height_map_relative_path = '/'
    
    file_name_and_path = janky_gazebo_path(height_map_name,height_map_path,folder_name)


    rospy.loginfo("Waiting for set model state service...")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    try:
        delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        with open(file_name_and_path, "r") as f:
            product_xml = f.read().replace('\n', '')

        orient =Rotation.from_euler('xyz', [90, 0, 0], degrees=True).as_quat()

        print(orient)

        orient = Quaternion(orient[0],orient[1],orient[2],orient[3])

        print(orient)

        delete_model('hole_terrain')

        print(ground_state.model_name)

        
        item_pose   =   Pose(Point(x=10/2, y=arena_length/2,    z=-arena_height),   orient)

        #spawn_model(ground_state.model_name, product_xml, "", item_pose, "world")
        rospy.loginfo("Successfully reset the ground for %s"%(ground_state.model_name))

    except rospy.ServiceException as e:
        rospy.logerr("Failed to reset the ground for %s"%(ground_state.model_name))
        rospy.logerr(e)



#Listens for triggers to randomize the arena
def server():

    rospy.Service("/randomize_robot_state", Trigger, randomize_robot)
    rospy.Service("/randomize_obstacle_state", Trigger, randomize_obstacles)

