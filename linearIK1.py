import numpy as np
import math

import RobotNodes as rn
import IK as ik
from ikTasks import *
from ikConstraints import *
from common import *

def clickCallback_left(event):
	taskFinger.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
		
def main():
	doCommonStuff("robotDescriptions/piston_robot.json")
	cn.solver = ik.solve_simple
	cn.epsilon = 0
	
	root  = cn.root
	robot = cn.robot
	
	root.bind("<Button-1>", clickCallback_left)
	root.bind("<B1-Motion>", clickCallback_left)
	root.bind("<Key>", keyCallback)
	
	global taskFinger
	taskFinger  = LocationTask(robot, Path(robot.getNodeByName("root"), robot.getNodeByName("finger")))
	
	taskFinger.setName("finger")
	
	cn.taskContainer = [[taskFinger]]
	root.mainloop()


if __name__ == "__main__":
    main()
