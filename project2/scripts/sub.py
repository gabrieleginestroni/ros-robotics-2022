#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from project2.srv import DrawPath, DrawPathResponse
import math
import rospkg 

rospack = rospkg.RosPack()

res = 0.05
width = 640
height = 672
min_x = -22.8
min_y = -10
max_x = min_x + res * width
max_y = min_y + res * height

radius = 1
color = 0  # must be 0 (black) <= color <= 255
interpolation = 1


def to_bytes(n, length, endianess='big'):
    h = '%x' % n
    s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
    return s if endianess == 'big' else s[::-1]


def read_pgm(name):
    with open(rospack.get_path('project2') + "/map/" + name + ".pgm", "rb") as pgmf:
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
last_row = -1
last_col = -1


def write_pgm(name):
    with open(rospack.get_path('project2') + "/map/" + name + ".pgm", "wb") as pgmf:
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


def is_neighbor(new_r, new_c, r, c):
	if abs(new_r - r) <= 1 and abs(new_c - c) <= 1:
		return True
	return False


def write_circle_on_raster(row, col):
    for i in range(row - radius, row + radius + 1):
        dist = int(math.sqrt(radius ** 2 - abs(i - row) ** 2))
        for j in range(col - dist, col + dist + 1):
            if 0 <= i <= height - 1 and 0 <= j <= width - 1:
                raster[i][j] = color

def interpolate0(row, col, last_row, last_col):
	if last_col < col:
		for c in range(last_col + 1, col + 1):
			write_circle_on_raster(last_row, c)
	else:
		for c in range(col, last_col):
			write_circle_on_raster(last_row, c)
	if last_row < row:
		for r in range(last_row + 1, row + 1):
			write_circle_on_raster(r, col)	
	else:
		for r in range(row, last_row):
			write_circle_on_raster(r, col)	


def callback(data):
    row, col = get_indexes(data.pose.pose.position.x, data.pose.pose.position.y)
    global last_row
    global last_col
    if last_row == -1 and last_col == -1:
        write_circle_on_raster(row, col)
    else:
	if interpolation == 0:
		# 0-th order interpolation
		interpolate0(row, col, last_row, last_col)
	else:
		# 1-st order interpolation
		if last_col == col:
		    if last_row > row:
		        for r in range(row, last_row):
		            write_circle_on_raster(r, col)
		    else:
		        for r in range(last_row + 1, row + 1):
		            write_circle_on_raster(r, col)
		else:
		    m = - float(last_row - row) / (last_col - col)
		    temp_c = last_col
		    temp_r = last_row
		    while last_col != col:
			prev_col = int(last_col)			
			prev_row = int(last_row)
		        if last_row >= row:
		            last_col += 1 * math.copysign(1, m)
		            last_row = temp_r + (temp_c - last_col) * m
			    if is_neighbor(int(math.floor(last_row)), int(last_col), prev_row, prev_col):
		            	write_circle_on_raster(int(math.floor(last_row)), int(last_col))
			    else:
				interpolate0(int(math.floor(last_row)), int(last_col), prev_row, prev_col)
		        else:
		            last_col -= 1 * math.copysign(1, m)
		            last_row = temp_r + (temp_c - last_col) * m
		            if is_neighbor(int(math.floor(last_row)), int(last_col), prev_row, prev_col):
		            	write_circle_on_raster(int(math.floor(last_row)), int(last_col))
			    else:
				interpolate0(int(math.floor(last_row)), int(last_col), prev_row, prev_col)
    last_row = row
    last_col = col


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
