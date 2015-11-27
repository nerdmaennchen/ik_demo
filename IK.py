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

def stuff():
	## handle "hard" constraints (like angle limits)
	dq_preliminary = Ny * jacobian_pinv * bigError         # change we introduce with this batch
	q_preliminary  = curValuesVec + dq + dq_preliminary    # current configuration + change from other batches
	q_constraintError = np.matrix(np.zeros((numCols, 1)))  # this is where we hold the error
	dq_corrected = dq_preliminary                          # this is the change we actually apply
	
	for constraint in constraints:
		q_constraintError += constraint.getError(q_preliminary) # += is actually wrong here... something like max and min should be used
	
	## we need the "inverse" of the errors... but we cannot allways divide this is a woraround 
	nyDiag = np.diag(Ny)
	for i in range(0, nyDiag.shape[0]):
		if nyDiag[i] > nullspaceEpsilon:
			q_constraintError[i, 0] = q_constraintError[i, 0] / nyDiag[i]
		else:
			q_constraintError[i, 0] = 0
	dq_corrected += Ny * q_constraintError # remove the constraint error
	dq  += dq_corrected                                    # apply the configuration change of the current taskGroup
	
