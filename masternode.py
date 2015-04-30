#!/usr/bin/env python

import rospy
import csv, struct
from realtimeUDP import RealtimeUDPMaster
from collections import deque
from mqp.msg import *
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
		self.sub_setpoints = rospy.Subscriber('/softhand/setpoints', HandSetpoint, self.setSetpoints, queue_size=1)

	def loop(self):
		while not rospy.is_shutdown():
			try:
				pass
				# [command, response] = self.k64f.recv()
				# self.processFrame(command, response)
			except Exception as e:
				print e

			try:
				(cmd, msg) = self.msgqueue.pop()
				self.k64f.send('192.168.1.3', cmd, msg)
			except IndexError as e:
				pass
				# print e

	def processFrame(self, command, response):
		if command == 2:
			addReadings(response)
		else:
			pass
		
	def setSetpoints(self, msg):
		response = bytearray(96)
		index = 0
		for finger in msg.fingers:
			print finger
			response[index:index+24] = struct.pack("fffffbxxx", finger.dutycycle, finger.pressure, finger.force, finger.position, finger.jointangle, finger.controlmode)
			index += 24
		self.msgqueue.appendleft((1, response))
	# Send readings to linear fit
	# Get linear fits
	# Send data to grasp evaluator
	# Command finger state
	# Recieve finger state

	def addReadings(self, response):
		byte_key = struct.unpack('i', response[0:4])
		msg = Reading()
		msg.series = 'pre_ff'
		msg.xValue = struct.unpack('f', response[0:4])[0] # Dutycycle
		msg.yValue = struct.unpack('f', response[4:8])[0] # Raw Pressure
		master.pub_readings.publish(msg)

		msg.series = 'pos_ff'
		msg.xValue = struct.unpack('f', response[8:12])[0] # Pressure
		msg.yValue = struct.unpack('f', response[12:16])[0] # Position
		master.pub_readings.publish(msg)

		msg.series = 'for_ff'
		msg.xValue = struct.unpack('f', response[8:12])[0] # Pressure - (Position_ff)
		msg.yValue = struct.unpack('f', response[16:20])[0] # Force
		master.pub_readings.publish(msg)

# Calibrate pressure
# Calibrate position
# Calibrate force

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