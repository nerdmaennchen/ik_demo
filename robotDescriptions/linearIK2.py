import numpy as np
import cairo
import math
import tkinter as tk
import time

import RobotNodes as rn
import IK as ik
from ikTasks import *
from ikConstraints import *
from common import *

def clickCallback_left(event):
	taskFinger.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
	
def clickCallback_right(event):
	taskPiston.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
		
def main():
	cn.solver = ik.solve_simple
	cn.epsilon = 0
	doCommonStuff("robotDescriptions/double_piston_robot.json")
	
	root  = cn.root
	robot = cn.robot
	
	root.bind("<Button-1>", clickCallback_left)
	root.bind("<B1-Motion>", clickCallback_left)
	root.bind("<Button-3>", clickCallback_right)
	root.bind("<B3-Motion>", clickCallback_right)
	
	global taskFinger, taskPiston, taskContainer
	taskFinger  = LocationTask(robot, Path(robot.getNodeByName("root"), robot.getNodeByName("finger")))
	taskPiston  = LocationTask(robot, Path(robot.getNodeByName("root"), robot.getNodeByName("piston2")))
	
	taskFinger.setName("finger")
	taskPiston.setName("piston")
	
	cn.taskContainer = [[taskPiston, taskFinger]]

	root.mainloop()


if __name__ == "__main__":
    main()



