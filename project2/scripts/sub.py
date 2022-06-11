#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from project2.srv import DrawPath, DrawPathResponse
import math
import os

os.chdir("../")
res = 0.05
width = 640
height = 672
min_x = -22.8
min_y = -10
max_x = min_x + res * width
max_y = min_y + res * height
radius = 3
color = 0  # must be 0 (black) <= color <= 255


def to_bytes(n, length, endianess='big'):
    h = '%x' % n
    s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    return s if endianess == 'big' else s[::-1]


def read_pgm(name):
    with open(os.getcwd() + "/robotics/src/project2/map/" + name + ".pgm", "rb") as pgmf:
        '''
        for i in range(2):
            print(pgmf.readline())
        (width, height) = [int(i) for i in pgmf.readline().split()]
        depth = int(pgmf.readline())
        '''
        header = b''
        for i in range(4):
            header += pgmf.readline()

        raster = []
        for y in range(height):
            row = []
            for y in range(width):
                row.append(ord(pgmf.read(1)))
            raster.append(row)
        return header, raster


header, raster = read_pgm("map")


def write_pgm(name):
    with open(os.getcwd() + "/robotics/src/project2/map/" + name + ".pgm", "wb") as pgmf:
        pgmf.write(header)

        for y in range(height):
            for x in range(width):
                pgmf.write(to_bytes(raster[y][x],1))


def get_indexes(x, y):
    if min_x <= x <= max_x and min_y <= y <= max_y:
        row = height - int((y - min_y) // res) - 1
        col = int((x - min_x) // res)
        return row, col
    return 0, 0


def write_circle_on_raster(row, col):
    for i in range(row - radius, row + radius + 1):
        dist = int(math.sqrt(radius ** 2 - abs(i - row) ** 2))
        for j in range(col - dist, col + dist + 1):
            if 0 <= i <= height - 1 and 0 <= j <= width - 1:
                raster[i][j] = color


def callback(data):
    row, col = get_indexes(data.pose.pose.position.x, data.pose.pose.position.y)
    write_circle_on_raster(row, col)


def save_img(mess):
    write_pgm("map_with_path")
    return DrawPathResponse()


def listener():
    rospy.init_node('path_finder', anonymous=True)
    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
    service = rospy.Service('draw_path', DrawPath, save_img)
    rospy.spin()


if __name__ == '__main__':
    listener()
