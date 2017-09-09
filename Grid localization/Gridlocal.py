#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

roslib.load_manifest('lab4')
thsd = 0.2
lns = Marker()	
arptn = np.zeros((35,35,4))
vtg = [0,0,125,525,125,325,125,125,425,125,425,325,425,525];

def rtadj(zrt):
	zrt = zrt - 360 if zrt > 180 else zrt + 360 if zrt < -180 else zrt 
	return zrt

def readbag():
	try:
		bag = rosbag.Bag(sys.argv[1] + '/scripts/grid.bag')
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			if topic == 'Observations':

				dt = msg.range * 100
				bearingval = msg.bearing
				zrt = np.degrees((euler_from_quaternion([bearingval.x, bearingval.y, bearingval.z, bearingval.w]))[2])
				observations_func(msg.tagNum, dt, zrt)

	
			elif topic == 'Movements': 
				zrt1, tt, zrt2 = msg.rotation1, msg.translation, msg.rotation2
				zrt1 = np.degrees((euler_from_quaternion([zrt1.x,zrt1.y,zrt1.z,zrt1.w]))[2])
				zrt2 = np.degrees((euler_from_quaternion([zrt2.x,zrt2.y,zrt2.z,zrt2.w]))[2])
				movements_func(zrt1, tt*100, zrt2)		
	finally:
		bag.close()


def observations_func(tagnum, tns, zrt):
	global arptn
	tt1,tt2,tp,tt3 = 0,0,0,0
	while(tt1 < 35):
		ttm = np.sqrt(((10 + 20*tt1) - vtg[2*(tagnum+1)]) ** 2 + ((10 + 20*tt2)  - vtg[2*(tagnum+1) + 1]) ** 2)
		rtm = np.degrees(np.arctan2(vtg[2*(tagnum+1)]-(10 + 20*tt2), vtg[2*(tagnum+1) + 1] - (10 + 20*tt1))) - (-135 + tt3*90)
		rtm = rtadj(rtm)
		rtp = 0.0088653 * np.power(2.718, -1.0 * (((rtm - zrt)**2)/4050.0))
		trp = 0.0398942 * np.power(2.718, -1.0 * (((ttm - tns)**2)/200.0))
		tp += arptn[tt1, tt2, tt3] * trp * rtp
		arptn[tt1, tt2, tt3] *= trp * rtp
		tt3 += 1
		if(tt3 == 4):
			tt3 = 0
			tt2 += 1
		if(tt2 == 35):
			tt3,tt2 = 0,0
			tt1 += 1
		
	arptn /= tp
	mx = np.argmax(arptn)
	ma = mx % 35
	y = (mx/4) % 35
	x = (mx/140) % 35
	rviz1(x, y, ma)



def movements_func(zrt1, tns, zrt2):
	global arptn, thsd
	tt1,tp,tt2,rt1,rt2,rt3,rt4 =0,0,0,0,0,0,0
	while(rt2 < 35):	
		if arptn[rt2, rt3, rt4] >= thsd:
			tns1x,tns1y,zrt11 = 20*tt1 + 10, 20*tt2 + 10, -180 +rt1*90 + 45
			tns2x,tns2y,zrt21 = 20*rt2 + 10, 20*rt3 + 10, -180 +rt4*90 + 45
			tnst = np.sqrt((tns1x - tns2x) ** 2 + (tns1y - tns2y) ** 2)
			zrt1t = np.degrees(np.arctan2(tns1y-tns2y, tns1x - tns2x)) - zrt21
			zrt2t = zrt11 - np.degrees(np.arctan2(tns1y-tns2y, tns1x - tns2x))
			zrt1t = rtadj(zrt1t)
			zrt2t = rtadj(zrt2t)
			zrt1p = 0.0088653 * np.power(2.718, -1.0 * (((zrt1t - zrt1)**2)/4050.0))
			tnsp = 0.0398942 * np.power(2.718, -1.0 * (((tnst - tns)**2)/200.0))
			zrt2p = 0.0088653 * np.power(2.718, -1.0 * (((zrt2t - zrt2)**2)/4050.0))
			arptn[tt1, tt2, rt1] += arptn[rt2, rt3, rt4] * tnsp * zrt1p * zrt2p
			tp += arptn[rt2, rt3, rt4] * tnsp * zrt1p * zrt2p
		if arptn[rt2, rt3, rt4] < thsd:
			rt4 += 1
			if(rt4 == 4):
				rt1,tt2,tt1,rt4 = 0,0,0,0
				rt3 += 1
			if(rt3 == 35):
				rt1,tt2,tt1,rt4,rt3 =0,0,0,0,0
				rt2 += 1
		else:
			rt1 += 1
			if(rt1 == 4):
				rt1= 0
				tt2 += 1
			if(tt2 == 35):
				rt1,tt2 = 0,0
				tt1 += 1
			if(tt1 == 35):
				rt1,tt2,tt1 = 0,0,0
				rt4 += 1
			if(rt4 == 4):
				rt1,tt2,tt1,rt4 = 0,0,0,0
				rt3 += 1
			if(rt3 == 35):
				rt1,tt2,tt1,rt4,rt3 =0,0,0,0,0
				rt2 += 1


	arptn /= tp
	mx = np.argmax(arptn)
	ma = mx % 35
	y = (mx/4) % 35
	x = (mx/140) % 35
	rviz1(x, y, ma)


def rviz1(i, j, k):
	global arptn,lns
	mx, my, ma = 20*i + 10, 20*j + 10, -180 +k*90 + 45	
	shape = Marker.LINE_STRIP
	pub = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
	lns.header.frame_id = "/my_frame"
	lns.header.stamp = rospy.Time.now()
	lns.ns = "local"
	lns.id = 0
	lns.type = shape
	lns.scale.x = 0.1
	lns.scale.y = 0.0
	lns.scale.z = 0.0 
        
	lns.color.r = 0.0
	lns.color.g = 1.0
	lns.color.b = 0.0
	lns.color.a = 1.0
	p = Point()
	p.x = mx/100.0
	p.y = my/100.0
	print("Index for Publish", mx/100.0, my/100.0)
	p.z = 0
	lns.points.append(p)

	ang = quaternion_from_euler(0,0,ma)
	lns.action = Marker.ADD
	pub.publish(lns)


def drawboundary():
	rate = rospy.Rate(100)
	arptn[12,28,3] = 1
	pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
	ii = 0
	while(ii < 14):
		boundary = Marker()
		boundary.header.frame_id = "/my_frame"
		boundary.header.stamp = rospy.Time.now()
		boundary.ns = "boundary"
		boundary.id = (ii+1)
		boundary.type = Marker.CUBE
		boundary.pose.position.x = vtg[ii]/100.0
		boundary.pose.position.y = vtg[ii+1]/100.0
		boundary.pose.position.z = 0
		boundary.scale.x = 0.1
		boundary.scale.y = 0.1
		boundary.scale.z = 0.1 
		
		boundary.color.r = 0.0
		boundary.color.g = 0.0
		boundary.color.b = 1.0
		boundary.color.a = 1.0
		boundary.action = Marker.ADD
		pub.publish(boundary)
		while (pub.get_num_connections() < 1):
			continue
		ii += 2
	readbag()


if __name__ == '__main__':
	rospy.init_node('motion')
	drawboundary()
