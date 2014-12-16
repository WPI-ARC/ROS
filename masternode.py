#!/usr/bin/env python

import rospy
import csv
from realtimeUDP import RealtimeUDPMaster
from collections import deque
from mqp.msg import Reading
from mqp.srv import LinearFitSrv, LinearFitSrvResponse

class SoftHandMaster:
	def __init__(self):
		rospy.init_node('SoftHandMaster')
		rospy.loginfo('Soft hand master node started')
		self._init_params()
		self._init_pubsub()

	def _init_params(self):
		#self.k64f = RealtimeUDPMaster()
		#self.msgqueue = deque([])
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
	master = SoftHandMaster()
	#master.loop()
	with open('pos_calibration.csv', 'rb') as f:
		reader = csv.reader(f)
		rowcount = len(reader)
		for row in reader:
			try:
				msg = Reading()
				msg.series = 'pos_ff'
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