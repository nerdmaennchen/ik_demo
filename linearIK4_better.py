import numpy as np
import math

import RobotNodes as rn
import IK as ik
from ikTasks import *
from ikConstraints import *
from common import *

def clickCallback_left(event):
	taskLeftHand.getMethod().setTarget(click2Rootcoordinate(event.x, event.y))
		
def main():
	cn.solver = ik.solve_better
	doCommonStuff("robotDescriptions/armDescription.json")
	
	root  = cn.root
	robot = cn.robot
	
	root.bind("<Button-1>", clickCallback_left)
	root.bind("<B1-Motion>", clickCallback_left)
	
	global taskLeftHand, taskLeftHand2
		
	path = Path(robot.getNodeByName("root"), robot.getNodeByName("finger"))
	taskLeftHand  = LocationTask(robot, path)
	taskLeftHand2 = LocationTask(robot, path)
	taskLeftHand2.setMethod(LineTargetMethod(np.matrix([[2],[2]]), np.matrix([[1],[0]])))
	
	taskLeftHand.setName("taskLeftHand")
	taskLeftHand2.setName("taskLeftHand2")

	cn.taskContainer = [[taskLeftHand2], [taskLeftHand]]

	root.mainloop()


if __name__ == "__main__":
    main()


