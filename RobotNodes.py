import numpy as np
from numpy.linalg import inv
import cairo
import math
from collections import OrderedDict
import ikTasks as ik

def npMat2CairoMat(mat):
	return cairo.Matrix(mat[0,0], mat[1,0], mat[0,1], mat[1,1], mat[0,2], mat[1,2])

def createAffineMatrix(translation, rotation):
	t = np.mat(np.eye(3, 3))
	rot = np.mat(np.eye(3, 3))
	t[0:2,2] = translation
	c = math.cos(rotation)
	s = math.sin(rotation)
	rot[0, 0] = c
	rot[0, 1] = -s
	rot[1, 0] = s
	rot[1, 1] = c
	return t * rot

class RobotMass():
	def __init__(self, description):
		self.position = np.array(description["position"])
		self.mass     = np.array(description["mass"])

class RobotNode():
	def __init__(self, description, parent):
		self.name = description["name"] or ""
		self.children = []
		self.masses   = []
		self.parent = parent
		if "rotation" in description:
			self.rotation = np.matrix(np.array(description["rotation"]) / 180. * math.pi).transpose()
		else:
			self.rotation = np.array([0])
		if "position" in description:
			self.position = np.matrix(np.array(description["position"])).transpose()
		else:
			self.position = np.matrix(np.array([0, 0])).transpose()
		if "masses" in description:
			for m in description["masses"]:
				self.masses.append(RobotMass(m))
		if "children" in description:
			for c in description["children"]:
				self.children.append(Robot.buildNode(c, self))
		self.forwardTransform = createAffineMatrix(self.position, self.rotation)
		self.postTransform = np.matrix(np.eye(3, 3))
		self.numDOF = 0
		
	def __str__(self):
		childStrings = "\n"
		for c in self.children:
			childStrings += str(c)
		childStrings = childStrings.replace("\n", "\n\t")
		return self.name + " at " + str(self.position) + "\nrotation: " + str(self.rotation) + childStrings
		
	def draw(self, ctx):
		ctx.save()
		ctx.transform(npMat2CairoMat(self.forwardTransform))
		self.drawPosMarker(ctx)
		ctx.transform(npMat2CairoMat(self.postTransform))
		
		for c in self.children:
			ctx.move_to(0, 0)
			ctx.set_source_rgb(0., 0., 0.)
			ctx.line_to(c.position[0], c.position[1])
			ctx.stroke()
			c.draw(ctx)
		ctx.set_source_rgb(0., 0., 255.)
		ctx.move_to(0, 0)
		ctx.line_to(.2, 0)
		ctx.stroke()
		ctx.set_source_rgb(0., 255., 0.)
		ctx.move_to(0, 0)
		ctx.line_to(0, .2)
		ctx.stroke()
		ctx.restore()
		
	def getTransform(self):
		return self.forwardTransform * self.postTransform
		
	def getBackTransform(self):
		return inv(self.getTransform())
		
class DummyNode(RobotNode):
	def __init__(self, description, parent):
		super(DummyNode, self).__init__(description, parent)
	
	def drawPosMarker(self, ctx):
		ctx.set_source_rgb(0., 0., 0.)
		ctx.rectangle(-.05, -.05, .1, .1)
		ctx.fill()
		
class FixedNode(DummyNode):
	def __init__(self, description, parent):
		super(FixedNode, self).__init__(description, parent)
		
	def drawPosMarker(self, ctx):
		ctx.set_source_rgb(0., 0., 0.)
		ctx.rectangle(-.05, -.05, .1, .1)
		ctx.fill()
		
class ActiveNode(RobotNode):
	def __init__(self, description, parent):
		RobotNode.__init__(self, description, parent)
		self.numDOF = 1
		if "value" in description:
			self.setValue(description["value"])
		else:
			self.setValue(0)
			
		if "minValue" in description:
			self.minValue = description["minValue"]
		else:
			self.minValue = float('-inf')
		if "maxValue" in description:
			self.maxValue = description["maxValue"]
		else:
			self.maxValue = float('inf')
			
	def setValue(self, val):
		self.value = val
		self.setValue_()
	
	def getValue(self):
		return self.value
		
class RotationNode(ActiveNode):
	def __init__(self, description, parent):
		ActiveNode.__init__(self, description, parent)
		self.setValue(self.getValue() / 180. * math.pi)
		self.minValue = self.minValue / 180. * math.pi
		self.maxValue = self.maxValue / 180. * math.pi
		
	def drawPosMarker(self, ctx):
		ctx.set_source_rgb(0., 0., 0.)
		ctx.arc(0, 0, .05, 0, math.pi * 2)
		ctx.fill()
		ctx.set_source_rgb(0, 0, 255.)
		ctx.arc(0, 0, .1, min(0, self.value), max(0, self.value))
		ctx.stroke()
		ctx.save()
		ctx.rotate(self.value)
		ctx.move_to(-.2, 0.15)
		ctx.set_font_size(0.1)
		ctx.scale(1, -1)
		ctx.show_text("{:10.4f}".format(float(self.value) / math.pi * 180.))
		ctx.restore()
		
	def setValue_(self):
		self.postTransform = createAffineMatrix(np.matrix(np.zeros((2, 1))), self.value)
	
	def getLocationDerivative(self, location):
		return np.matrix([[-location[1, 0]],[location[0, 0]]])
		
	def getOrientationDerivative(self, orientation):
		return np.matrix([[-orientation[1, 0]],[orientation[0, 0]]])
		
class PistonNode(ActiveNode):
	def __init__(self, description, parent):
		ActiveNode.__init__(self, description, parent)
		
	def drawPosMarker(self, ctx):
		ctx.move_to(.1, -.05)
		ctx.set_source_rgb(0., 0., 0.)
		ctx.line_to(-.1, -.05)
		ctx.line_to(-.1, .05)
		ctx.line_to(.1, .05)
		ctx.stroke()
		ctx.set_source_rgb(0, 0, 255.)
		ctx.move_to(0, 0)
		ctx.line_to(self.value, 0)
		ctx.stroke()
		ctx.move_to(-.2, 0.15)
		ctx.save()
		ctx.scale(1, -1)
		ctx.set_font_size(0.1)
		ctx.show_text("{:10.4f}".format(float(self.value)))
		ctx.restore()
		
	def setValue_(self):
		self.postTransform = createAffineMatrix(np.matrix([[self.value],[0]]), 0)
		
	def getLocationDerivative(self, location):
		return np.matrix([[1],[0]])
		
	def getOrientationDerivative(self, orientation):
		return np.matrix([[0],[0]])

class Robot:
	def __init__(self, description):
		self.rootNode = Robot.buildNode(description)
		self.nodes = OrderedDict()
		self.activeNodes = OrderedDict()
		self.initHelper(self.rootNode)
		
	def initHelper(self, node):
		self.nodes[node.name] = node
		if node.numDOF != 0:
			self.activeNodes[node.name] = node
		for c in node.children:
			self.initHelper(c)
		
	def getNodeByName(self, name):
		return self.nodes[name]
	
	def draw(self, ctx):
		self.rootNode.draw(ctx)
	
	def getDOF(self):
		dof = 0
		for node in self.activeNodes.items():
			dof += node[1].numDOF
		return dof
	
	def getConfiguration(self):
		q = np.matrix(np.zeros((self.getDOF(), 1)))
		for node in self.activeNodes.items():
			q[self.getIndexOfActiveNode(node[1].name), 0] = node[1].getValue()
		return q
		
	def setConfiguration(self, q):
		for idx in range(0, q.shape[0]):
			self.getActiveNodeWithIndex(idx).setValue(q[idx])

	def getIndexOfNode(self, node):
		return list(self.nodes.keys()).index(node)
		
	def getIndexOfActiveNode(self, node):
		return list(self.activeNodes.keys()).index(node)
		
	def getActiveNodeWithIndex(self, idx):
		return list(self.activeNodes.values())[idx]
		
	def getTransformForPath(self, path):
		transform = np.matrix(np.eye(3, 3))
		for i in range(1, len(path.path)):
			fromNode, fromDirection = path.path[i - 1]
			toNode,   toDirection   = path.path[i]
			if fromDirection == ik.Direction.FROM_CHILD:
				transform = fromNode.getTransform() * transform
			elif fromDirection == ik.Direction.FROM_PARENT:
				transform = toNode.getBackTransform() * transform
			elif fromDirection == ik.Direction.LINK:
				transform = toNode.getBackTransform() * transform
		return transform
		
	def __str__(self):
		return str(self.rootNode)
	
	class AABB:
		def __init__(self, xmin, xmax, ymin, ymax):
			self.xmin = xmin
			self.xmax = xmax
			self.ymin = ymin
			self.ymax = ymax
		def merge(self, other):
			self.xmin = min(self.xmin, other.xmin)
			self.xmax = max(self.xmax, other.xmax)
			self.ymin = min(self.ymin, other.ymin)
			self.ymax = max(self.ymax, other.ymax)
		
		def getWidth(self):
			return self.xmax - self.xmin
		def getHeight(self):
			return self.ymax - self.ymin
				
		def __str__(self):
			return str(self.xmin) + " " + str(self.xmax) + " " + str(self.ymin) + " " + str(self.ymax)
		
		
	def getAABB(self):
		return self.getAABB_sub(self.rootNode, np.matrix(np.eye(3, 3)))
	
	def getAABB_sub(self, node, forwardMat):
		forwardMat = forwardMat * node.getTransform()
		box = Robot.AABB(forwardMat[0, 2], forwardMat[0, 2], forwardMat[1, 2], forwardMat[1, 2])
		for c in node.children:
			box.merge(self.getAABB_sub(c, forwardMat))
		return box
		
	@staticmethod
	def buildNode(description, parent = None):
		factory = {
			'dummy' : DummyNode,
			'fixed' : FixedNode,
			'rotation' : RotationNode, 
			'piston' : PistonNode 
		}
		return factory[description['type']](description, parent)
	
