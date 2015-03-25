#!/usr/bin/env python

import csv
import rospy
# import rospkq

class DataLoader:
	def __init__(self, filename):
		rospy.init_node('DataLoader')
		rospy.loginfo('Data loader node started')
		self._init_params(filename)
		self._init_pubsub()

	def _init_params(self, filename):
		rospack = rospkg.RosPack()
		path = rospack.get_path('mqp')
		self.file = path+'/data/'+name+'.csv'
		pass

	def _init_pubsub(self):
		self.pub_readings = rospy.Publisher('/softhand/readings', Reading)
		# pub readings
		# pub hand state

	def loop(self):
		while not rospy.is_shutdown():
			pass
			# Check for incoming messages
			# Check for messages to send


	# Send readings to linear fit
	# Get linear fits
	# Send data to grasp evaluator
	# Command finger state
	# Recieve finger state

if __name__ == '__main__':
	# master = SoftHandMaster()
	#master.loop()
	with open('pressure_calib.csv', 'rb') as f:
		reader = csv.reader(f)
		for row in reader:
			try:
				msg = Reading()
				msg.series = 'pre_ff'
				msg.xValue = float(row[1])
				msg.yValue = float(row[2])
				master.pub_readings.publish(msg)
				rospy.sleep(rospy.Duration(0.01))
			except:
				print 'Bad row: ', row


	# rospy.sleep(rospy.Duration(1))
	# rospy.wait_for_service('get_linear_fit')
	# get_linear_fit = rospy.ServiceProxy('get_linear_fit', LinearFitSrv)
	# rsp = get_linear_fit('pos_ff', 0.01)
	# print rsp
	while not rospy.is_shutdown():
		rospy.spin()