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
		self.k64f = RealtimeUDPMaster('192.168.1.2', 10001, 10002)
		self.msgqueue = deque([])

	def _init_pubsub(self):
		self.pub_readings = rospy.Publisher('/softhand/readings', Reading)
		# pub readings
		# pub hand state

	def loop(self):
		while not rospy.is_shutdown():
			try:
				[command, response] = self.k64f.recv()
				self.processFrame(command, response)
			except(e):
				print e

			try:
				(cmd, msg) = self.msgqueue.pop()
				self.k64f.send('192.168.1.3', cmd, msg)
			except(e):
				print e

	def processFrame(self, command, response):
		if command == 2:
			addReadings(response)
		else:
			pass
		
	# Send readings to linear fit
	# Get linear fits
	# Send data to grasp evaluator
	# Command finger state
	# Recieve finger state

	def addReadings(self, response):
		msg = Reading()
		msg.series = 'pos_ff'
		msg.xValue = struct.unpack('f', response[0:4])[0]
		msg.yValue = struct.unpack('f', response[4:8])[0]
		master.pub_readings.publish(msg)

if __name__ == '__main__':
	master = SoftHandMaster()
	master.loop()
	# with open('pos_calibration.csv', 'rb') as f:
	# 	reader = csv.reader(f)
	# 	rowcount = len(reader)
	# 	for row in reader:
	# 		try:
	# 			msg = Reading()
	# 			msg.series = 'pos_ff'
	# 			msg.xValue = float(row[1])
	# 			msg.yValue = float(row[2])
	# 			master.pub_readings.publish(msg)
	# 			rospy.sleep(rospy.Duration(0.01))
	# 		except:
	# 			print 'Bad row: ', row


	# rospy.sleep(rospy.Duration(1))
	# rospy.wait_for_service('get_linear_fit')
	# get_linear_fit = rospy.ServiceProxy('get_linear_fit', LinearFitSrv)
	# rsp = get_linear_fit('pos_ff', 0.01)
	# print rsp
	while not rospy.is_shutdown():
		rospy.spin()