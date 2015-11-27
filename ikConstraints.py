import numpy as np

class Limits:
	def __init__(self, robot):
		self.numJoints = robot.getDOF()
		self.robot = robot
		self.minVals = np.matrix(np.zeros((self.numJoints, 1)))
		self.maxVals = np.matrix(np.zeros((self.numJoints, 1)))
		for i in range(0, self.numJoints):
			node = robot.getActiveNodeWithIndex(i)
			self.minVals[i, 0] = node.minValue
			self.maxVals[i, 0] = node.maxValue
			assert(self.minVals[i, 0] < self.maxVals[i, 0])
	
	def getError(self, curValues):
		error = np.matrix(np.zeros((self.numJoints, 1)))
		for i in range(0, self.numJoints):
			val = curValues[i, 0]
			if val < self.minVals[i, 0]:
				error[i, 0] = self.minVals[i, 0] - val
			if val > self.maxVals[i, 0]:
				error[i, 0] = self.maxVals[i, 0] - val
		return error
