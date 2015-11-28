import numpy as np
from numpy.linalg import inv
from copy import copy

def pinv(matrix, epsilon, W=None):
	if type(W) == np.matrixlib.defmatrix.matrix:
		helper = matrix * W * matrix.transpose()
		return W * matrix.transpose() * inv(helper + epsilon * np.matrix(np.eye(helper.shape[0])))
	else:
		helper = matrix * matrix.transpose()
		return matrix.transpose() * inv(helper + epsilon * np.matrix(np.eye(helper.shape[0])))

def solve_simple(robot, taskGroups, epsilon=1, nullspaceEpsilon=1e-10, constraints = []):
	numCols = robot.getDOF()
	dq = np.matrix(np.zeros((numCols, 1)))
	Ny = np.matrix(np.eye(numCols, numCols))
	reallyBigJacobian = np.matrix(np.zeros((0, numCols)))
	
	for taskGroup in taskGroups:
		bigJacobian = np.matrix(np.zeros((0, numCols)))
		bigError    = np.matrix(np.zeros((0, 1)))
		for task in taskGroup:
			bigJacobian = np.concatenate((bigJacobian, task.getJacobian()))
			bigError    = np.concatenate((bigError,    task.getError()))
		
		jacobian_pinv = pinv(bigJacobian, epsilon)
		
		dq  += Ny * jacobian_pinv * bigError                                    # apply the configuration change of the current taskGroup
		
		reallyBigJacobian = np.concatenate((reallyBigJacobian, bigJacobian))
		Ny = np.matrix(np.eye(numCols, numCols)) - pinv(reallyBigJacobian, nullspaceEpsilon) * reallyBigJacobian
	robot.setConfiguration(robot.getConfiguration() + dq)
	return dq
	

def solve_better(robot, taskGroups, epsilon=1, nullspaceEpsilon=1e-10, constraints = []):
	numCols = robot.getDOF()
	dq = np.matrix(np.zeros((numCols, 1)))
	Ny = np.matrix(np.eye(numCols, numCols))
	reallyBigJacobian = np.matrix(np.zeros((0, numCols)))
	curValuesVec = robot.getConfiguration()
	
	for taskGroup in taskGroups:
		bigJacobian = np.matrix(np.zeros((0, numCols)))
		bigError    = np.matrix(np.zeros((0, 1)))
		for task in taskGroup:
			bigJacobian = np.concatenate((bigJacobian, task.getJacobian()))
			bigError    = np.concatenate((bigError,    task.getError()))
			
		jacobian_pinv = pinv(bigJacobian, epsilon, Ny)
		
		dq += Ny * jacobian_pinv * bigError         # change we introduce with this batch
		
		reallyBigJacobian = np.concatenate((reallyBigJacobian, bigJacobian))
		Ny = np.matrix(np.eye(numCols, numCols)) - pinv(reallyBigJacobian, nullspaceEpsilon) * reallyBigJacobian
	robot.setConfiguration(robot.getConfiguration() + dq)
	return dq

def solve_better2(robot, taskGroups, epsilon=1, nullspaceEpsilon=1e-10, constraints = []):
	numCols = robot.getDOF()
	dq = np.matrix(np.zeros((numCols, 1)))
	Ny = np.matrix(np.eye(numCols, numCols))
	reallyBigJacobian = np.matrix(np.zeros((0, numCols)))
	curValuesVec = robot.getConfiguration()
	
	for taskGroup in taskGroups:
		bigJacobian = np.matrix(np.zeros((0, numCols)))
		bigError    = np.matrix(np.zeros((0, 1)))
		for task in taskGroup:
			bigJacobian = np.concatenate((bigJacobian, task.getJacobian()))
			bigError    = np.concatenate((bigError,    task.getError()))
			
		jacobian_pinv = pinv(bigJacobian, epsilon, Ny)
		
		dq += jacobian_pinv * bigError         # change we introduce with this batch
		
		reallyBigJacobian = np.concatenate((reallyBigJacobian, bigJacobian))
		Ny = np.matrix(np.eye(numCols, numCols)) - pinv(reallyBigJacobian, nullspaceEpsilon) * reallyBigJacobian
	robot.setConfiguration(robot.getConfiguration() + dq)
	return dq
	
def solve_best(robot, taskGroups, epsilon=1, nullspaceEpsilon=1e-10, constraints = []):
	numCols = robot.getDOF()
	dq = np.matrix(np.zeros((numCols, 1)))
	
	for i in range(0, len(taskGroups)):
		reallyBigJacobian = np.matrix(np.zeros((0, numCols)))
		for j in range(0, i):
			for task in taskGroups[j]:
				reallyBigJacobian = np.concatenate((reallyBigJacobian, task.getJacobian()))
		Ny = np.matrix(np.eye(numCols, numCols)) - pinv(reallyBigJacobian, nullspaceEpsilon) * reallyBigJacobian
		
		taskGroup = taskGroups[i]
		bigJacobian = np.matrix(np.zeros((0, numCols)))
		bigError    = np.matrix(np.zeros((0, 1)))
		for task in taskGroup:
			bigJacobian = np.concatenate((bigJacobian, task.getJacobian()))
			bigError    = np.concatenate((bigError,    task.getError()))
		
		dq_ = pinv(bigJacobian, epsilon, Ny) * bigError         # change we introduce with this batch
		dq += dq_
		robot.setConfiguration(robot.getConfiguration() + dq_)
	
	return dq
