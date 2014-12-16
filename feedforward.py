#!/usr/bin/env python

import csv
import rospy
import rospkg
from math import sqrt

class FeedForward:
	def __init__(self, name):
		self._init_params(name)
		self._load_data()

	def _init_params(self, name):
		rospack = rospkg.RosPack()
		path = rospack.get_path('mqp')
		self.file = path+'/data/'+name+'.csv'
		self.ranges = []
		self.values = []
		self.counts = []
		self.threshold = 0.0

	def _load_data(self):
		try:
			with open(self.file, 'rb') as f:
				reader = csv.reader(f)
				for row in reader:
					self.ranges.append(float(row[0]))
					self.values.append(float(row[1]))
					self.counts.append(float(row[2]))
				f.close()
		except:
			rospy.loginfo(self.file+' not found')

	def saveChanges(self):
		if len(self.ranges) > 0:
			with open(self.file, 'wb') as f:
				writer = csv.writer(f)
				row = [0,0,0]
				for n in range(len(self.ranges)):
					row[0] = self.ranges[n]
					row[1] = self.values[n]
					row[2] = self.counts[n]
					writer.writerow(row)
				f.close()

	def addReading(self, xValue, yValue):
		if xValue in self.ranges:
			# Just continue building an average
			index = self.ranges.index(xValue)
			if self.counts[index] < 100:
				self.values[index] = ((self.values[index] * self.counts[index]) + yValue) / (self.counts[index] + 1)
				
			# Slowly decrease the significance of older readings
			else:
				self.values[index] = ((self.values[index] * 99) + yValue) / 100

			self.counts[index] = self.counts[index] + 1
		else:
			# Add a new data point to the end of the list
			self.ranges.append(xValue)
			self.values.append(yValue)
			self.counts.append(1)

			# Sort by ranges
			zipped = zip(*sorted(zip(self.ranges, self.values, self.counts))) #wtfmagic
			self.ranges = list(zipped[0])
			self.values = list(zipped[1])
			self.counts = list(zipped[2])

	def getSegments(self, tolerance):
		self.threshold = tolerance
		return self.segment(0, len(self.ranges)-1)

	def segment(self, start, end):
		# Calculate equation for the line from start to end
		slope = (self.values[end] - self.values[start]) / (self.ranges[end] - self.ranges[start])
		offset = self.values[start] - (slope * self.ranges[start])

		maxdistance = 0
		maxindex = 0
		# Find the point with the maximum error
		for n in range(start+1,end):
			distance = self.calcDistance(slope, offset, self.ranges[n], self.values[n])
			if distance > maxdistance:
				maxdistance = distance
				maxindex = n

		# Calculate error for the points on either side of the point of maximum error
		lower = self.calcDistance(slope, offset, self.ranges[maxindex-1], self.values[maxindex-1])
		upper = self.calcDistance(slope, offset, self.ranges[maxindex+1], self.values[maxindex+1])

		# Check if the error is too great AND not just a noisy reading
		if maxdistance > self.threshold and ((upper > self.threshold) or (lower > self.threshold)) and ((end - start) > 2):
			# Split data on the point of greatest error and recurse
			(error1, start1, slope1, offset1) = self.segment(start, maxindex)
			(error2, start2, slope2, offset2) = self.segment(maxindex, end)

			# Combine linear functions
			start1.extend(start2)
			slope1.extend(slope2)
			offset1.extend(offset2)
			return (max(error1, error2), start1, slope1, offset1)
		else:
			return (maxdistance, [self.ranges[start]], [slope], [offset])

	def calcDistance(self, slope, offset, x, y):
		return abs((slope * x) + ((-1) * y) + offset) / sqrt((slope ** 2) + ((-1) ** 2))

	def invert(self, ranges, slopes, offsets):
		newranges = []
		newslopes = []
		newoffsets = []
		for n in range(len(ranges)):
			newranges.append((slopes[n] * ranges[n]) + offsets[n])
			newslopes.append(1/slopes[n])
			newoffsets.append(-(offsets[n]/slopes[n]))
		return (newranges, newslopes, newoffsets)

def feedForwardMain():
	ff_obj = FeedForward('pos_ff')
	with open('pos_calib.csv', 'rb') as f:
		reader = csv.reader(f)
		for row in reader:
			try:
				ff_obj.addReading(float(row[1]), float(row[3]))
			except:
				print row
	(error, ranges, slopes, offsets) = ff_obj.getSegments(0.01)
	print (error, ranges, slopes, offsets)
	print ff_obj.invert(ranges, slopes, offsets)
	ff_obj.saveChanges()

if __name__ == '__main__':
	feedForwardMain()