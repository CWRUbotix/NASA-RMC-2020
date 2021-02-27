#!/usr/bin/env python3

import rospy
import png 
import random
import numpy
import os

width =  513
height = width

min_8bit_grayscale_val = 0
max_8bit_grayscale_val = 255

grayscale_bitdepth = 8

sizeX = 6.8
sizeY = 6.8
sizeZ = 0.4

obstacleZoneY = 3.2
arena_height = 6.8
arena_width = 2.5

resolutionX = sizeX/width
resolutionY = sizeY/height

rockRadius = 0.5

x1 = random.uniform(0,arena_width)
x2 = random.uniform(0,arena_width)
x3 = random.uniform(0,arena_width)

y1 = random.uniform(obstacleZoneY,sizeY)
y2 = random.uniform(obstacleZoneY,sizeY)
y3 = random.uniform(obstacleZoneY,sizeY)

obstacle_data_amount = 3
rock_amount = 3
hole_amount = 2 

def intersecting_circles(circle1, circle2):

    d = numpy.sqrt((circle1[0]-circle2[0])*(circle1[0]-circle2[0]) + (circle1[1]-circle2[1])*(circle1[1]-circle2[1]))
    r = circle1[2]+circle2[2]

    return d < r 

def intersecting_multiple_circles(circle_array,circle):

    for i in circle_array:

        if(intersecting_circles(i,circle)):
            return True
    
    return False

def hole_spot_chooser(obstacles):

    hole_max_radius = 0.2
    hole_min_radius = 0.1
    hole_circle = numpy.zeros(obstacle_data_amount)
    
    while True:
        
        print("obstacles in loop")
        print(obstacles)
        
        hole_radius = random.uniform(hole_min_radius,hole_max_radius)

        hole_x =  random.uniform(hole_radius,arena_width-hole_radius)
        hole_y =  random.uniform(hole_radius,obstacleZoneY-hole_radius)
                
        hole_circle[0] = hole_x
        hole_circle[1] = hole_y 
        hole_circle[2] = hole_radius

        if( not(intersecting_multiple_circles(obstacles,hole_circle))):
            break

    new_obstacles = numpy.zeros((len(obstacles)+1,obstacle_data_amount))
   
    for j in range(len(obstacles)):

        new_obstacles[j] = obstacles[j]
        print("new obstacles within loop")
        print(new_obstacles)

    new_obstacles[len(obstacles)] = hole_circle

   
    print("append obstacles")
    print(new_obstacles)

    return new_obstacles

def pixel_value(holes,x_pixel,y_pixel):

    pixel_color = max_8bit_grayscale_val

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


def print8bitGreyscalePng(img):
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
  
  
def write8bitGreyscalePng(filename, img):
  height = len(img)
  width = len(img[0])
  print("write8bitGreyscalePng height=%d" % height)
  print("write8bitGreyscalePng width=%d" % width)

  f = open(filename, 'wb')
  w = png.Writer(width, height, greyscale=True, bitdepth=8)
  w.write(f, img)
  f.close()

  print("write8bitGreyscalePng wrote %s" % filename)

def generate_height_map():

    
    obstacles = numpy.zeros((rock_amount,obstacle_data_amount))
    obstacles_one_hole = numpy.zeros(((len(obstacles)+1), obstacle_data_amount))
    obstacles_two_holes = numpy.zeros((len(obstacles_one_hole)+1, obstacle_data_amount))

    obstacles[0][0] = x1
    obstacles[0][1] = y1
    obstacles[0][2] = rockRadius

    obstacles[1][0] = x2
    obstacles[1][1] = y2
    obstacles[1][2] = rockRadius

    obstacles[2][0] = x3
    obstacles[2][1] = y3
    obstacles[2][2] = rockRadius

    obstacles_one_hole = hole_spot_chooser(obstacles)
    print("obstacles one hole")
    print(obstacles_one_hole)
    obstacles_two_holes = hole_spot_chooser(obstacles_one_hole)
    print("obstacles two hole")
    print(obstacles_two_holes)

    print("we made it")

    holes = numpy.zeros((hole_amount,obstacle_data_amount))
    holes[0] = obstacles_two_holes[len(obstacles_two_holes)-2]
    holes[1] = obstacles_two_holes[len(obstacles_two_holes)-1]

    height_map_name = 'height_map.png'
    height_map_path = 'glenn_description/models/hole_terrain/materials/textures'
    height_map_relative_path = '/'

    file_path =__file__
    file_path_array = file_path.split('/')

    for i in file_path_array:

        height_map_relative_path = os.path.join(height_map_relative_path,i)
        if(i == 'NASA-RMC-2020'):
            break

    height_map_name_and_path = os.path.join(height_map_relative_path,height_map_path,height_map_name)

    print(height_map_name_and_path)
    

    img = [[pixel_value(holes,x,y) for x in range(width)] for y in range(height)]
           
    
    write8bitGreyscalePng(height_map_name_and_path,img)

    
def main():
    rospy.loginfo("generate height map main")
    rospy.init_node("GenerateHeightMap")
    generate_height_map()
    rospy.spin()


if __name__ == "__main__":
    main()