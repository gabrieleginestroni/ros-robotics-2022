#! /usr/bin/env python

import rosbag
import csv

bag = rosbag.Bag('../bags/N_bag.bag')

f = open('../csv/N_calibration.csv', 'w')
writer = csv.writer(f)

header = ['sec', 'nsec', 'x', 'y', 'q_x', 'q_y', 'q_w', 'q_z', 'fl','fr', 'rl', 'rr']
writer.writerow(header)

for topic, msg, t in bag.read_messages(topics=['/pose_vel_sync']):
    writer.writerow([msg.sec, msg.nsec, msg.poseX, msg.poseY, msg.q_x, msg.q_y, msg.q_w, msg.q_z,  msg.ticks_fl, msg.ticks_fr, msg.ticks_rl, msg.ticks_rr])

bag.close()
f.close()

