import pybullet
import time
import pybullet_data
import math, random

import OpenGL
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

cubes = []
render_vertices = []
colors = []

world = []

for x in range(0,10):
	world.append([])
	for y in range(0,10):
		world[x].append(random.randint(0,2))

world[1][1] = 5

print(world)

def groundVertices(size, basez, world, deleteoldboxes=False):
	vertices = []
	boxes = []
	
	halfsize = int(math.floor(size/2.0))
	
	for x in range(-halfsize, halfsize):
		for y in range(-halfsize, halfsize):
			vertices.append((4*x, 4*y, basez + world[x][y]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
			
			vertices.append((4*x + 4, 4*y + 4, basez + world[x + 1][y + 1]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
			
			if deleteoldboxes:
				shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,halfExtents=[2,2,0.1])
				boxId = pybullet.createMultiBody(0,shape,-1,[(4*x),(4*y),world[x][y]+basez],[0,0,0])
				boxes.append(boxId)
	
	if deleteoldboxes:
		return boxes
	else:
		return vertices

def createCube(pos, size=1, orn=[0,0,0]):
	cubeStartOrientation = pybullet.getQuaternionFromEuler(orn)

	shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,halfExtents=[size,size,size])
	boxId = pybullet.createMultiBody(1,shape,-1,pos,cubeStartOrientation)
	
	cubes.append([boxId,size])

def cubeVertices(pos, size, orn):
	vertices = []
	
	cubex = pos[0]
	cubey = pos[1]
	cubez = pos[2]
	
	# Bottom Face
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	
	# Top Face
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	
	# X+ Face
	vertices.append(((1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	
	# X- Face
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	
	# Y+ Face
	vertices.append(((1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (1*size)+cubey, (-1*size)+cubez))
	
	# Y- Face
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (1*size)+cubez))
	vertices.append(((-1*size)+cubex, (-1*size)+cubey, (-1*size)+cubez))
	
	euler = pybullet.getEulerFromQuaternion(orn)
	
	if euler[2] != 0:
		xrot = euler[0]
		yrot = euler[1]
		zrot = euler[2]
		
		i = 0
		for vertex in vertices:
			x = vertex[0] - cubex
			y = vertex[1] - cubey
			z = vertex[2] - cubez
			
			# Z Rotation
			levelonex = ((math.cos(zrot)*x) - (math.sin(zrot)*y))
			leveloney = ((math.sin(zrot)*x) + (math.cos(zrot)*y))
			levelonez = z # Consistency
			
			# Y Rotation
			leveltwox = ((math.cos(yrot)*levelonex) - (math.sin(yrot)*levelonez))
			leveltwoy = leveloney
			leveltwoz = ((math.sin(yrot)*levelonex) + (math.cos(yrot)*levelonez))
			
			# X Rotation
			levelthreex = leveltwox + cubex
			levelthreey = ((math.cos(xrot)*leveltwoy) - (math.sin(xrot)*leveltwoz)) + cubey
			levelthreez = ((math.sin(xrot)*leveltwoy) + (math.cos(xrot)*leveltwoz)) + cubez
			
			vertices[i] = (levelthreex, levelthreey, levelthreez)
			i += 1
	
	return vertices

# Initialize Physics Engine. We use pybullet.DIRECT since we are not using pybullet's GUI rendering system.
physicsClient = pybullet.connect(pybullet.DIRECT)
pybullet.setGravity(0,0,-10)

pybullet.createCollisionShape(pybullet.GEOM_PLANE)
pybullet.createMultiBody(0,0,-1,[0,0,-9])

# Initialize a Pygame window showing OpenGL
pygame.init()
display = (1200, 720)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
pygame.key.set_repeat(1, 2)

# gluLookAt takes 9 arguments, the camera position, the lookat position and the up vector. (Just set that to all zeroes except for a 1 for the position that is upwards)
def reset_camera():
	yaw = 0.0
	camerax = -3
	cameray = 1
	cameraz = -2
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+math.cos(yaw),cameray+math.sin(yaw),-4, 0,0,1)
	return yaw, camerax, cameray, cameraz

yaw, camerax, cameray, cameraz = reset_camera()
	

glEnable(GL_DEPTH_TEST)

createCube([0,12,0], 1, [0,0,45])
createCube([4,4,6], 1, [0,0,0])
createCube([4,5.9,9], 2, [0,0,0])

glClearColor(0.5, 0.6, 1.0, 0.0);

walkspeed = 0.5
turnspeed = 0.03

boxestodelete = groundVertices(6, -9, world, True) # We run this once to initiate the first collision boxes.

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
			quit()
		elif event.type == pygame.KEYDOWN:
			if pygame.key.get_pressed()[pygame.K_LEFT]:
				yaw += turnspeed
			elif pygame.key.get_pressed()[pygame.K_RIGHT]:
				yaw -= turnspeed
			if pygame.key.get_pressed()[pygame.K_UP]:
				camerax += math.cos(yaw) * walkspeed 
				cameray += math.sin(yaw) * walkspeed
			elif pygame.key.get_pressed()[pygame.K_DOWN]:
				camerax -= math.cos(yaw) * walkspeed
				cameray -= math.sin(yaw) * walkspeed
			if pygame.key.get_pressed()[pygame.K_SPACE]:
				yaw, camerax, cameray, cameraz = reset_camera()
	
	glLoadIdentity()
	gluPerspective(45, (float(display[0])/float(display[1])), 0.1, 100.0)
	gluLookAt(camerax,cameray,cameraz, camerax+math.cos(yaw),cameray+math.sin(yaw),cameraz, 0,0,1)
	
	# Step Physics Simulation
	pybullet.stepSimulation()
	
	groundpoints = groundVertices(6, -9, world)
	
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
		
		cubepoints = cubeVertices(cubePos, size, cubeOrn)
		
		for vertex in cubepoints:
			render_vertices.append(vertex)
			colors.append((0.5,0.5,0.5))
	
		
	
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
	
		
	glBegin(GL_TRIANGLES)
	
	i = 0
	for vertex in render_vertices:
		glColor3fv(colors[i])
		glVertex3fv(vertex)
		i+=1
	
	glEnd()
	
	# Empty Vertex List
	render_vertices = []
	colors = []
	
	pygame.display.flip()
	pygame.time.wait(10)
