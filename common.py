import numpy as np
import getopt, sys
import cairo
import argparse
import IK as ik
import tkinter as tk
from PIL import Image, ImageTk
import json

import tempfile 

import RobotNodes as rn


class cn:
	robot = None
	taskFinger = None
	taskPiston = None
	WIDTH = None
	HEIGHT = None
	SCALE = None
	label_image = None
	surface = None
	root = None
	epsilon = .1
	nullspaceEpsilon = 1e-10
	solver = ik.solve_best
	taskContainer = []
	constraints   = []
	pause = False
	delay = 1
	
def drawAndShow():
	surface = cn.surface
	robot   = cn.robot
	WIDTH, HEIGHT, SCALE = cn.WIDTH, cn.HEIGHT, cn.SCALE
	ctx = cairo.Context (surface)
	ctx.set_source_rgb(1, 1, 1)
	ctx.rectangle(0, 0, WIDTH, HEIGHT)
	ctx.fill()
	ctx.set_line_width(SCALE / (max(WIDTH, HEIGHT)))
	ctx.translate(0, HEIGHT)
	ctx.scale (WIDTH / SCALE, HEIGHT / SCALE)
	ctx.scale(1, -1)
	cn.robot.draw(ctx)
	
	surface.flush()
	
	img = None
	if cairo.version == '1.10.1':
		img = Image.frombuffer("RGBA", (surface.get_width(), surface.get_height()), surface.get_data(),"raw","RGBA",0,1)
	else:
		tmpfile = tempfile.TemporaryFile()
		surface.write_to_png(tmpfile)
		img = Image.open(tmpfile)
	
	tkpi = ImageTk.PhotoImage(img)
	cn.label_image.configure(image = tkpi)
	cn.label_image.image = tkpi

	

def click2Rootcoordinate(x, y):
	WIDTH, HEIGHT, SCALE = cn.WIDTH, cn.HEIGHT, cn.SCALE
	x = x * SCALE / WIDTH
	y = SCALE - y * SCALE / HEIGHT
	return np.matrix([[x],[y]])

def update_tick():
	if not cn.pause:
		cn.solver(cn.robot, cn.taskContainer, epsilon=cn.epsilon, nullspaceEpsilon=cn.nullspaceEpsilon, constraints=cn.constraints)
	drawAndShow()
	cn.root.after(cn.delay, update_tick)
	
def keyCallback(event):
	if ' ' == event.char:
		cn.pause = not cn.pause
		
def doCommonStuff(descriptionPath):
	cn.descriptionPath = descriptionPath
	try:
		opts, args = getopt.getopt(sys.argv[1:], "", ["nullspaceepsilon=", "epsilon=", "robot=", "solve=", "delay="])
	except getopt.GetoptError as err:
		pass
			
	for o, a in opts:
		if o == "--epsilon":
			cn.epsilon = float(a)
		if o == "--nullspaceepsilon":
			cn.nullspaceEpsilon = float(a)
		if o == "--robot":
			cn.descriptionPath = a
		if o == "--solve":
			if a == "simple":
				cn.solver = ik.solve_simple
			if a == "better":
				cn.solver = ik.solve_better
			if a == "better2":
				cn.solver = ik.solve_better2
			if a == "best":
				cn.solver = ik.solve_best
			if a == "delay":
				cn.delay = int(o)

	cn.robot = rn.Robot(json.load(open(cn.descriptionPath)))
	aabb = cn.robot.getAABB()
	cn.WIDTH, cn.HEIGHT = 1024, 1024
	cn.SCALE = max(aabb.getWidth(), aabb.getHeight()) * 2
	cn.surface = cairo.ImageSurface (cairo.FORMAT_ARGB32, cn.WIDTH, cn.HEIGHT)

	cn.root = tk.Tk()
	cn.root.geometry('{}x{}'.format(cn.WIDTH, cn.HEIGHT))
	cn.label_image = tk.Label(cn.root)
	cn.label_image.place(x=0, y=0, width=cn.WIDTH, height=cn.HEIGHT)
	
	cn.root.bind("<Key>", keyCallback)
	
	np.set_printoptions(linewidth=200)
	update_tick()
	
