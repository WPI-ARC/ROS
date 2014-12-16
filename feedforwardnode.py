#!/usr/bin/env python

import rospy
from mqp.msg import Reading
from mqp.srv import LinearFitSrv, LinearFitSrvResponse
from feedforward import FeedForward

class FeedForwardEstimator:
	def __init__(self):
		rospy.init_node('FeedForwardEstimator')
		rospy.loginfo('Feed forward estimator node started')
		self._init_params()
		rospy.on_shutdown(self._on_shutdown)
		self._init_pubsub()

	def _init_params(self):
		self.feedforwards = {}

	def _on_shutdown(self):
		rospy.loginfo('Feed forward estimator node shutting down')
		for ff in self.feedforwards.values():
			ff.saveChanges()

	def _init_pubsub(self):
		self.datasub = rospy.Subscriber('/softhand/readings', Reading, self.addReading, queue_size=1)
		self.service = rospy.Service('get_linear_fit', LinearFitSrv, self.linearFitService)

	def linearFitService(self, req):
		if not req.series in self.feedforwards.keys():
			self.createFeedForward(req.series)
		rsp = LinearFitSrvResponse()
		(rsp.error, ranges, slopes, offsets) = (self.feedforwards[req.series]).getSegments(req.tolerance)
		(rsp.ranges, rsp.slopes, rsp.offsets) = (self.feedforwards[req.series]).invert(ranges, slopes, offsets)
		return rsp

	def addReading(self, msg):
		if not msg.series in self.feedforwards.keys():
			self.createFeedForward(msg.series)
		(self.feedforwards[msg.series]).addReading(msg.xValue, msg.yValue)

	def createFeedForward(self, series):
		rospy.loginfo('Adding fitting object for '+series)
		self.feedforwards[series] = FeedForward(series)

if __name__ == '__main__':
	feedforward = FeedForwardEstimator()

	while not rospy.is_shutdown():
		rospy.spin()
