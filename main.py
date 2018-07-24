#!/usr/local/bin/python

import pybullet
import time
import pybullet_data
import math, random
import sys
import numpy

import OpenGL
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GL import shaders

import render.cubeRender as cubeRender
import render.worldRender as worldRender
import world.worldGen as worldGen

cubes = []
render_vertices = []
colors = []

vertex_array = numpy.array([], numpy.float32)
color_array = numpy.array([], numpy.float32)

worldsize = 13
world = worldGen.worldGen(worldsize)

# Temporary line to test world rendering.
world[1][1] = 5
display = (1200, 720)

def init_libs():
	"""Initialize Pybullet and Pygame. Turn on GL's depth test and make the sky blue."""
	physicsClient = pybullet.connect(pybullet.DIRECT)
	pybullet.setGravity(0,0,-40)
	
	pygame.init()
	pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
	pygame.key.set_repeat(1, 2)
	
	glEnable(GL_DEPTH_TEST)
	glClearColor(0.5, 0.6, 1.0, 0.0);
	
def setup_world():
	"""Sets up the basic debug world."""
	plane = pybullet.createCollisionShape(pybullet.GEOM_PLANE)
	pybullet.createMultiBody(0,plane,-1,[0,0,-9])

	# Later on my plan is to just generate a world. For now, we need some debug cubes.
	cubes.append(cubeRender.createCube([0,12,0], 1, [0,0,45]))
	cubes.append(cubeRender.createCube([4,4,6], 1, [0,0,0]))
	cubes.append(cubeRender.createCube([4,5.9,9], 2, [0,0,0]))
	
	boxestodelete = worldGen.resetWorldBoxes(worldsize, -9, world) # We run this once to initiate the first collision boxes.

def reset_camera():
	"""Resets the camera to the start position. Returns Yaw and Camera Position."""
	
	# These numbers have no significance other than just being near where the cubes and terrain are rendered. (At the Origin)
	yaw = 0.0
	camerax = -3
	cameray = 1
	cameraz = -2
	
	# gluLookAt takes 9 arguments, the camera position, the lookat position and the up vector.
	# (Just set the up vector to all zeroes except for a 1 for the axis that is upwards)
	# gluLookAt also multiplies the "current vector" rather than changing the camera vector because PyOpenGL is stupid.
	# Use glLoadIdentity() to stop this.
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+math.cos(yaw),cameray+math.sin(yaw),-4, 0,0,1)
	return yaw, camerax, cameray, cameraz

def create_program():
	VERTEX_SHADER = """ 
		attribute vec3 a_Position;
		attribute vec3 a_Color;

		varying vec4 v_Color;

		void main()
		{
			v_Color = vec4(a_Color, 1.0);
			gl_Position = gl_ModelViewMatrix * vec4(a_Position, 1.0);
		}
		"""

	FRAGMENT_SHADER = """
		varying vec4 v_Color;

		void main()
		{
			gl_FragColor = v_Color;
		}
		"""
	
	vertshader = shaders.compileShader(VERTEX_SHADER, GL_VERTEX_SHADER)
	fragshader = shaders.compileShader(FRAGMENT_SHADER, GL_FRAGMENT_SHADER)

	program = glCreateProgram()
	glAttachShader(program, vertshader)
	glAttachShader(program, fragshader)
	
	glLinkProgram(program)
	
	return program

def update_vertex_arrays(vertex_data, color_data, program):
	global vertex_array
	global color_array
	
	i = 0
	for vertex in vertex_data:
		color_array = numpy.append(color_array, [colors[i][0],colors[i][1],colors[i][2]])
		vertex_array = numpy.append(vertex_array, [vertex[0],vertex[1],vertex[2]])
		i+=1
	
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vertex_array)
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, color_array)

	glEnableVertexAttribArray(0)
	glEnableVertexAttribArray(1)

	glBindAttribLocation(program, 0, "a_Position")
	glBindAttribLocation(program, 1, "a_Color")
	
	return len(vertex_array)/3

def render_loop(program, numtriangles):
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	
	glUseProgram(program)
	glDrawArrays(GL_TRIANGLES, 0, numtriangles)
	

init_libs()
setup_world()
yaw, camerax, cameray, cameraz = reset_camera()
program = create_program()

walkspeed = 0.5
turnspeed = 0.03

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			quit()
		elif event.type == pygame.KEYDOWN:
			pressed_keys = pygame.key.get_pressed()
			
			if pressed_keys[pygame.K_LEFT]:
				yaw += turnspeed
			elif pressed_keys[pygame.K_RIGHT]:
				yaw -= turnspeed
			if pressed_keys[pygame.K_UP]:
				camerax += math.cos(yaw) * walkspeed 
				cameray += math.sin(yaw) * walkspeed
			elif pressed_keys[pygame.K_DOWN]:
				camerax -= math.cos(yaw) * walkspeed
				cameray -= math.sin(yaw) * walkspeed
			if pressed_keys[pygame.K_SPACE]:
				yaw, camerax, cameray, cameraz = reset_camera()
			if pressed_keys[pygame.K_q]:
				boxestodelete = worldGen.resetWorldBoxes(worldsize, -9, world, boxestodelete)
			if pressed_keys[pygame.K_f]:
				for cube in cubes:
					pybullet.applyExternalForce(cube[0], -1, [0,0,100],[0,0,0],pybullet.LINK_FRAME)
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+math.cos(yaw),cameray+math.sin(yaw),cameraz, 0,0,1)
	
	# Step Physics Simulation
	pybullet.stepSimulation()
	
	groundpoints = worldRender.groundVertices(worldsize, -9, world)
	
	for vertex in groundpoints:
		render_vertices.append(vertex)
		if vertex[2] > -5:
			colors.append((0.7,0.5,0.2))
		else:
			colors.append((0.2,0.5,0.2))
	
	# Calculate Vertices to render
	for cube in cubes:
		cubeId = cube[0]
		size = cube[1]
		
		cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
		
		cubepoints = cubeRender.cubeVertices(cubePos, size, cubeOrn)
		
		for vertex in cubepoints:
			render_vertices.append(vertex)
			colors.append((0.5,0.5,0.5))
	
	numtriangles = update_vertex_arrays(render_vertices, colors, program)
	render_loop(program, numtriangles)
	
	# Empty Vertex List
	render_vertices = []
	colors = []
	vertex_array = numpy.array([], numpy.float32)
	color_array = numpy.array([], numpy.float32)
	
	pygame.display.flip()
	pygame.time.wait(10)
