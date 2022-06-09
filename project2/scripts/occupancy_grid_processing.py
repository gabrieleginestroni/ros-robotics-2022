#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid


pub = rospy.Publisher('my_map', OccupancyGrid, queue_size=1, latch=True)

def callback(data):
    width = data.info.width
    height = data.info.height
    size =  (width , height)
    image = np.zeros(size)
    
    counter = 0
    #print (len(data.data))
    for p in data.data:
      image[counter%width][int(counter/width)] = p
      counter+=1
    
    
    
    kernel = np.ones((2,2),np.uint8)
    image = cv2.dilate(image,kernel,iterations = 1)
    image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    custom_map = OccupancyGrid()

    custom_map.header = data.header
    custom_map.info = data.info
    custom_map.data =  np.transpose(image).flatten().astype (int)
    pub.publish(custom_map)



    image = cv2.rotate(image, cv2.ROTATE_180)
    image[image == -1] = 150
    image[image == 0] = 255
    image[image == 100] = 0
    
    filename = '../maps/grid.png'
    print ("saving image")
    cv2.imwrite (filename,image)


    

 
  

def listener():

    rospy.init_node('depth_saver', anonymous=False)

    rospy.Subscriber("/map", OccupancyGrid, callback)
    

    
    
    
   
    rospy.spin()

if __name__ == '__main__':
    listener()
