#! /usr/bin/env python

import rosbag
import csv

bag = rosbag.Bag('../bags/calibration3.bag')

f = open('../csv/calibration3.csv', 'w')
writer = csv.writer(f)

header = ['timestamp', 'x', 'y','fl','fr', 'rl', 'rr']
writer.writerow(header)

for topic, msg, t in bag.read_messages(topics=['/pose_vel_sync']):
    writer.writerow([t, msg.poseX, msg.poseY, msg.rpm_fl, msg.rpm_fr, msg.rpm_rl, msg.rpm_rr])

bag.close()
f.close()

