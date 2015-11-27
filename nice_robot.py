import json
import numpy as np
import cairo
import math
from PIL import Image, ImageTk
import tkinter as tk
import time
import getopt, sys
import argparse

import RobotNodes as rn
import IK as ik
from ikTasks import *
from ikConstraints import *
from common import *

def clickCallback_left(event):
	taskLeftHand.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
	
def clickCallback_right(event):
	taskRightHand.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
		
		
def main():
	global taskLeftHand, taskRightHand
	doCommonStuff("robotDescriptions/doubleArmDescription.json")
	
	root  = cn.root
	robot = cn.robot
	
	root.bind("<Button-1>", clickCallback_left)
	root.bind("<B1-Motion>", clickCallback_left)
	root.bind("<Button-3>", clickCallback_right)
	root.bind("<B3-Motion>", clickCallback_right)
	
	jointLimits = Limits(robot)
	
	path1 = Path(robot.getNodeByName("root"), robot.getNodeByName("rightFinger"))	
	taskRightHand = LocationTask(robot, path1)
	taskRightHandOrient = OrientationTask(robot, path1, 0)
	taskRightHand.setName("taskRightHand")
	taskRightHandOrient.setName("taskRightHandOrient")
	
	path2 = Path(robot.getNodeByName("root"), robot.getNodeByName("leftFinger"))
	taskLeftHand  = LocationTask(robot, path2)
	taskLeftHand2 = LocationTask(robot, path2)
	taskLeftHand2.setMethod(LineTargetMethod(np.matrix([[2],[2]]), np.matrix([[1],[0]])))
	taskLeftHandOrient  = OrientationTask(robot, path2, 1)
	
	taskLeftHand.setName("taskLeftHand")
	taskLeftHand2.setName("taskLeftHand2")
	taskLeftHandOrient.setName("taskLeftHandOrient")
	
	defaultValTasks = []
	for i in range(0, robot.getDOF()):
		task = PreferedAngleTask(robot, robot.getActiveNodeWithIndex(i).name)
		task.setName(robot.getActiveNodeWithIndex(i).name + "Default")
		defaultValTasks.append(task)
	
	cn.taskContainer = [[taskRightHandOrient], [taskRightHand], [ taskLeftHand2], [taskLeftHand], defaultValTasks]

	root.mainloop()


if __name__ == "__main__":
    main()
