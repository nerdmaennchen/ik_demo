import numpy as np
from numpy.linalg import inv
from copy import copy
from enum import Enum
import math

class TargetMethod:
	def __init__(self):
		pass

class LineTargetMethod(TargetMethod):
	def __init__(self, supportPoint, direction):
		self.setTarget(supportPoint, direction)
		
	def setTarget(self, supportPoint, direction):
		directionHelper = np.matrix([[-direction[1, 0], direction[0, 0]]])
		self.direction  = directionHelper * 1. / np.linalg.norm(directionHelper)
		self.d          = self.direction * supportPoint

	def getTarget(self):
		return self.d
		
	def transform(self, point):
		return self.direction * point

class PointTargetMethod(TargetMethod):
	def __init(self):
		pass
	
	def transform(self, point):
		return point

	def setTarget(self, target):
		self.target = target
	
	def getTarget(self):
		return self.target

class Task:
	def __init__(self, robot, name=""):
		self.robot = robot
		self.dof = self.robot.getDOF()
		self.method = PointTargetMethod()
		self.method.setTarget(self.getCurrentValue())
		self.name = name
	
	def setName(self, name):
		self.name = name
	def getName(self):
		return self.name
		
	def getNumRows(self):
		return self.target.shape[0]

	def getMethod(self):
		return self.method
		
	def setMethod(self, method):
		self.method = method
		
	def getError(self):
		return self.method.getTarget() - self.method.transform(self.getCurrentValue())
		
	def getJacobian(self):
		return self.method.transform(self.getJacobian_())
		
class Direction(Enum):
	FROM_PARENT = 1
	FROM_CHILD = 2
	LINK = 3
	
class Path:
	def __init__(self, start, end):
		fromlist = []
		backlist = []
		end_ = end
		start_ = start
		while end_ != None:
			fromlist.insert(0, (end_, Direction.FROM_CHILD))
			end_ = end_.parent
		while start_ != None:
			backlist.insert(0, (start_, Direction.FROM_PARENT))
			start_ = start_.parent
		
		while len(backlist) > 1 and len(fromlist) > 1 and backlist[1][0] == fromlist[1][0]:
			backlist.pop(0)
			fromlist.pop(0)
		
		if len(backlist) > 0 and len(fromlist) > 0  and backlist[0][0] == fromlist[0][0]:
			linknode, direction = backlist[0]
			backlist.pop(0)
			fromlist.pop(0)
			backlist.insert(0, (linknode, Direction.LINK))
			
		backlist.reverse()
		self.path = backlist + fromlist
		
	def copy(self):
		ret = copy(self)
		ret.path = copy(self.path)
		return ret
		
	def __str__(self):
		ret = ""
		for p in self.path:
			node, direction = p
			ret += "(" + node.name + " " + str(direction) + ") "
		return ret
		
class PathedTask(Task):
	def __init__(self, robot, path):
		self.path = path.copy()
		self.path.path.reverse()
		Task.__init__(self, robot)
		
class LocationTask(PathedTask):
	def __init__(self, robot, path):
		PathedTask.__init__(self, robot, path)
		
	def getJacobian_(self):
		transform = np.matrix(np.eye(3, 3))
		jacobian  = np.matrix(np.zeros((2, self.dof)))
		for node, direction in self.path.path:
			subtransform = np.matrix(np.eye(2, 2))
			if node.numDOF != 0:
				idx = self.robot.getIndexOfActiveNode(node.name)
				if direction == Direction.FROM_CHILD:
					jacobian[:,idx] = node.getLocationDerivative(transform[0:2,2])
				elif direction == Direction.FROM_PARENT:
					jacobian[:,idx] = -node.getLocationDerivative(transform[0:2,2])
			if direction == Direction.FROM_CHILD:
				subtransform = node.getTransform()[0:2,0:2]
				transform = node.getTransform() * transform
			elif direction == Direction.FROM_PARENT:
				subtransform = node.getBackTransform()[0:2,0:2]
				transform = transform * node.getBackTransform()
			elif direction == Direction.LINK:
				transform = transform * node.getBackTransform()
			
			jacobian = subtransform * jacobian
		return jacobian
	
	def getCurrentValue(self):
		return self.robot.getTransformForPath(self.path)[0:2, 2]


class OrientationTask(PathedTask):
	def __init__(self, robot, path, dim=0):
		self.dim = dim
		PathedTask.__init__(self, robot, path)
		
	def getJacobian_(self):
		transform = np.matrix(np.eye(3, 3))
		jacobian  = np.matrix(np.zeros((2, self.dof)))
		for node, direction in self.path.path:
			subtransform = np.matrix(np.eye(2, 2))
			if node.numDOF != 0:
				idx = self.robot.getIndexOfActiveNode(node.name)
				if direction == Direction.FROM_CHILD:
					jacobian[:,idx] = node.getOrientationDerivative(transform[0:2, self.dim])
				elif direction == Direction.FROM_PARENT:
					jacobian[:,idx] = -node.getOrientationDerivative(transform[0:2, self.dim])
			if direction == Direction.FROM_CHILD:
				subtransform = node.getTransform()[0:2,0:2]
				transform = node.getTransform() * transform
			elif direction == Direction.FROM_PARENT:
				subtransform = node.getBackTransform()[0:2,0:2]
				transform = transform * node.getBackTransform()
			elif direction == Direction.LINK:
				transform = transform * node.getBackTransform()
			
			jacobian = subtransform * jacobian
		return jacobian
	
	def getCurrentValue(self):
		return self.robot.getTransformForPath(self.path)[0:2, self.dim]


class PreferedAngleTask(Task):
	def __init__(self, robot, node):
		self.node = node
		Task.__init__(self, robot)
		self.getMethod().setTarget(np.matrix([[0]]))
		
	def getJacobian_(self):
		transform = np.matrix(np.eye(3, 3))
		jacobian  = np.matrix(np.zeros((1, self.dof)))
		jacobian[0, self.robot.getIndexOfActiveNode(self.node)] = 1
		return jacobian
	
	def getCurrentValue(self):
		return np.matrix(self.robot.getNodeByName(self.node).getValue())
