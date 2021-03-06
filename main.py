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
import ctypes
from OpenGL.GL import shaders

import render.cubeRender as cubeRender
import render.worldRender as worldRender
import render.renderLoop as renderLoop
import world.worldGen as worldGen
import gui.textRender as textRender
import gui.invRender as invRender
import gui.inventory as inventory

# TERRAIN VBO ARRAYS

chunksize = 16
basez = -9
world = {}
view_range = 1
chunk_view_adjustment = 4.0 # This is a slight multiplier to the size of the world so that it doesn't look small when the player walks on it.

for x in range(-5, 5):
	for y in range(-5, 5):
		chunk = worldGen.worldGen(chunksize)
		world[(x,y)] = chunk

print(world.keys())

terrain_vbo = numpy.array([], numpy.float32)
color_vbo = numpy.array([], numpy.float32)

stos = [] # Static Terrain Objects, which appear with the terrain

# Cubes, non-terrain object arrays. Using VBOs for moving objects is laggy.

cubes = []

vertex_array = numpy.array([], numpy.float32)
color_array = numpy.array([], numpy.float32)

# Temporary line to test world rendering.
display = (1200, 720)

def init_libs():
	"""Initialize Pybullet and Pygame. Turn on GL's depth test and make the sky blue."""
	physicsClient = pybullet.connect(pybullet.DIRECT)
	pybullet.setGravity(0,0,-40)
	
	pygame.init()
	pygame.display.set_mode(display, HWSURFACE|OPENGL|DOUBLEBUF)
	pygame.key.set_repeat(1, 2)
	
	glEnable(GL_DEPTH_TEST)
	glClearColor(0.5, 0.6, 1.0, 0.0);
	glViewport(0, 0, display[0], display[1])
	
def setup_world(world, player_chunk_position):
	"""Sets up the basic debug world."""
	plane = pybullet.createCollisionShape(pybullet.GEOM_PLANE)
	pybullet.createMultiBody(0,plane,-1,[0,0,-9])

	# Later on my plan is to just generate a world. For now, we need some debug cubes.
	cubes.append(cubeRender.createCube([0,12,0], 1, [45,45,45]))
	cubes.append(cubeRender.createCube([4,-4,6], 1, [0,0,0]))
	cubes.append(cubeRender.createCube([4,5.9,9], 2, [45,30,10]))
	
	addSTO([18,3], 1, [0.6, 0.2, 0.1])

	boxestodelete = worldGen.resetWorldBoxes(chunksize, -9, player_chunk_position, world) # We run this once to initiate the first collision boxes.
	
	return boxestodelete

def reset_camera():
	"""Resets the camera to the start position. Returns Yaw and Camera Position."""
	
	# These numbers have no significance other than just being near where the cubes and terrain are rendered. (At the Origin)
	yaw = 0.0
	pitch = 0.0
	camerax = -3
	cameray = 1
	cameraz = -2
	
	# gluLookAt takes 9 arguments, the camera position, the lookat position and the up vector.
	# (Just set the up vector to all zeroes except for a 1 for the axis that is upwards)
	# gluLookAt also multiplies the "current vector" rather than changing the camera vector because PyOpenGL is stupid.
	# Use glLoadIdentity() to stop this.
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+(math.cos(yaw)*math.cos(pitch)),cameray+(math.sin(yaw)*math.cos(pitch)),(-4)+math.cos(pitch), 0,0,1)
	return yaw, pitch, camerax, cameray, cameraz

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

def create_gui_program():
	VERTEX_SHADER = """ 
		attribute vec3 a_Position;
		attribute vec3 a_Color;

		varying vec4 v_Color;

		void main()
		{
			v_Color = vec4(a_Color, 1.0);
			gl_Position = vec4(a_Position, 1.0);
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

def addVBOVertex(vertex, color):
	global terrain_vbo
	global color_vbo
	
	terrain_vbo = numpy.append(terrain_vbo, [vertex[0],vertex[1],vertex[2]])
	color_vbo = numpy.append(color_vbo, [color[0],color[1],color[2]])
	
def addSTO(position2d, size, color):
	chunk_x = int(math.floor(position2d[0]/chunksize))
	chunk_y = int(math.floor(position2d[1]/chunksize))
	
	chunk_position = (chunk_x, chunk_y)
	
	x = int(position2d[0] - (chunk_x*chunksize))
	y = int(position2d[1] - (chunk_y*chunksize))
	z = len(world[chunk_position][x][y]) + basez + size
	
	
	stos.append([(position2d[0],position2d[1],z), size, color])

def recalculate_vbos(buffers, player_chunk_position, view_range):
	global terrain_vbo
	global color_vbo
	
	terrain_vbo = numpy.array([], numpy.float32)
	color_vbo = numpy.array([], numpy.float32)
	
	groundpoints, topsoil = worldRender.groundVertices(chunksize, basez, world, player_chunk_position, view_range, chunk_view_adjustment)
	for i in range(0,len(groundpoints)):
		if topsoil[i] == 0:
			addVBOVertex(groundpoints[i],(0.7,0.5,0.2))
		elif topsoil[i] == 1:
			addVBOVertex(groundpoints[i],(0.3,0.7,0.3))
		elif topsoil[i] == 2:
			addVBOVertex(groundpoints[i],(0.6,0.6,0.3))
	
	
	glBindBuffer(GL_ARRAY_BUFFER, buffers[0])
	glBufferData(GL_ARRAY_BUFFER, len(terrain_vbo)*4, (ctypes.c_float*len(terrain_vbo))(*terrain_vbo), GL_STATIC_DRAW)

	glBindBuffer(GL_ARRAY_BUFFER, buffers[1])
	glBufferData(GL_ARRAY_BUFFER, len(color_vbo)*4, (ctypes.c_float*len(color_vbo))(*color_vbo), GL_STATIC_DRAW)
	glBindBuffer(GL_ARRAY_BUFFER, 0)
	

init_libs()
player_chunk_position = (round(-3/chunksize), round(1/chunksize)) # -3 and 1 are the default position of the camera but I need reset camera to come after the world is setup.
last_player_chunk_position = player_chunk_position
boxestodelete = setup_world(world, player_chunk_position)
yaw, pitch, camerax, cameray, cameraz = reset_camera()
program = create_program()
gui_program = create_gui_program()

grab_mouse = False
gui_active = False

buffers = glGenBuffers(2)
recalculate_vbos(buffers, player_chunk_position, view_range)

walkspeed = 0.5

sensitivity = 400.0

text_collection = textRender.TextCollection(display, "gui/textures/")
text_collection.add_text("PyOpenGL Sandbox", 30.0, 0.0, 0.8, True)

prev_pressed = pygame.key.get_pressed()
no_key_timer = 0

gui_v, gui_c = invRender.create_inventory(2,4, display, [])
player_inventory = inventory.create_inv(2,4)

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			quit()
		elif event.type == pygame.KEYDOWN:
			pressed_keys = pygame.key.get_pressed()
			
			if pressed_keys[pygame.K_m] and (no_key_timer > 5 or not prev_pressed[pygame.K_m]):
				if grab_mouse:
					grab_mouse = False
					pygame.mouse.set_visible(True)
				else:
					grab_mouse = True
					pygame.mouse.set_visible(False)
			if pressed_keys[pygame.K_w] and not gui_active:
				camerax += math.cos(yaw) * walkspeed 
				cameray += math.sin(yaw) * walkspeed
			elif pressed_keys[pygame.K_s] and not gui_active:
				camerax -= math.cos(yaw) * walkspeed
				cameray -= math.sin(yaw) * walkspeed
			if pressed_keys[pygame.K_a] and not gui_active:
				camerax += math.cos(yaw+(math.pi/2.0)) * walkspeed 
				cameray += math.sin(yaw+(math.pi/2.0)) * walkspeed
			if pressed_keys[pygame.K_d] and not gui_active:
				camerax += math.cos(yaw-(math.pi/2.0)) * walkspeed 
				cameray += math.sin(yaw-(math.pi/2.0)) * walkspeed
			if pressed_keys[pygame.K_SPACE] and not gui_active:
				yaw, pitch, camerax, cameray, cameraz = reset_camera()
			if pressed_keys[pygame.K_q]:
				digx = int(float(camerax)/chunk_view_adjustment)
				digy = int(float(cameray)/chunk_view_adjustment)
				chunk = world[player_chunk_position]
				if digx < len(chunk) -1:
					if digy < len(chunk[digx]) -1:
						if len(world[player_chunk_position][digx][digy]) != 1:
							if world[player_chunk_position][digx][digy][-1] == 1 or world[player_chunk_position][digx][digy][-1] == 0:
								inventory.add_to_inv(player_inventory, "dirt")
							elif world[player_chunk_position][digx][digy][-1] == 2:
								inventory.add_to_inv(player_inventory, "sand")
						
							del world[player_chunk_position][digx][digy][-1]
						else:
							world[player_chunk_position][digx][digy][-1] = 2
				
				boxestodelete = worldGen.resetWorldBoxes(chunksize, basez, player_chunk_position, world, boxestodelete)
				recalculate_vbos(buffers, player_chunk_position, view_range)
			if pressed_keys[pygame.K_e]:
				digx = int(float(camerax)/chunk_view_adjustment) - player_chunk_position[0]*chunksize
				digy = int(float(cameray)/chunk_view_adjustment) - player_chunk_position[1]*chunksize
				chunk = world[player_chunk_position]
				if digx < len(chunk) -1:
					if digy < len(chunk[digx]) -1:
						if inventory.inv_contains(player_inventory, "dirt"):
							world[player_chunk_position][digx][digy].append(0)
							inventory.remove_from_inv(player_inventory, "dirt")
						elif inventory.inv_contains(player_inventory, "sand"):
							world[player_chunk_position][digx][digy].append(2)
							inventory.remove_from_inv(player_inventory, "sand")
				
				boxestodelete = worldGen.resetWorldBoxes(chunksize, basez, player_chunk_position, world, boxestodelete)
				recalculate_vbos(buffers, player_chunk_position, view_range)
			if pressed_keys[pygame.K_f] and not gui_active:
				for cube in cubes:
					pybullet.applyExternalForce(cube[0], -1, [0,0,100],[0,0,0],pybullet.LINK_FRAME)
			if pressed_keys[pygame.K_i] and (no_key_timer > 5 or not prev_pressed[pygame.K_i]):
				gui_v, gui_c = invRender.create_inventory(2,4, display, player_inventory)
				if gui_active:
					gui_active = False
				else:
					gui_active = True
			
			no_key_timer = 0
			prev_pressed = pressed_keys
			
		elif event.type == pygame.MOUSEMOTION and grab_mouse and not gui_active:
			mousemove = pygame.mouse.get_pos()
			dyaw = mousemove[0] - (display[0]/2)
			dpitch = mousemove[1] - (display[1]/2)
			
			newpitch = pitch - dpitch/float(sensitivity)
			
			yaw -= dyaw/float(sensitivity)
			if newpitch > -1.45 and newpitch < 1.45:
				pitch = newpitch
			
			pygame.mouse.set_pos((display[0]/2),(display[1]/2))
				
	
	# Step Physics Simulation
	pybullet.stepSimulation()
	
	
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	
	player_chunk_position = (round(camerax/(chunksize*chunk_view_adjustment)), round(cameray/(chunksize*chunk_view_adjustment)))
	if player_chunk_position != last_player_chunk_position:
		boxestodelete = worldGen.resetWorldBoxes(chunksize, basez, player_chunk_position, world, boxestodelete)
		recalculate_vbos(buffers, player_chunk_position, view_range)
	
	last_player_chunk_position = player_chunk_position
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+(math.cos(yaw)*math.cos(pitch)),cameray+(math.sin(yaw)*math.cos(pitch)),cameraz+math.sin(pitch), 0,0,1)
	
	renderLoop.vbo_render(program, buffers, len(terrain_vbo)/3)
	renderLoop.render_loop(program, cubes)
	for sto in stos:
		renderLoop.static_render_loop(program, sto[0], sto[1], sto[2])
	#text_collection.render() #Laggy and problematic
	if gui_active:
		renderLoop.gui_render(gui_program, gui_v, gui_c)
	
	pygame.display.flip()
	pygame.time.wait(10)
	
	no_key_timer += 1
