#! /usr/bin/env python

import rosbag
import csv

bag = rosbag.Bag('../bags/calibration1.bag')

f = open('../csv/calibration1.csv', 'w')
writer = csv.writer(f)

header = ['sec', 'nsec', 'x', 'y','fl','fr', 'rl', 'rr']
writer.writerow(header)

for topic, msg, t in bag.read_messages(topics=['/pose_vel_sync']):
    writer.writerow([msg.sec, msg.nsec, msg.poseX, msg.poseY, msg.rpm_fl, msg.rpm_fr, msg.rpm_rl, msg.rpm_rr])

bag.close()
f.close()

