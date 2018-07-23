import random
import pybullet
import math

def worldGen(size, amplitude=1):
	"""Temporary worldgen function. Will be replaced later with something
	that actually looks nice. Takes size and noise amplitude and returns
	world vertices for use in other functions."""
	world = []
	
	for x in range(0,size):
		world.append([])
		for y in range(0,size):
			world[x].append(random.randint(0,amplitude))
	
	return world

def resetWorldBoxes(size, basez, world, deleteids=[]):
	"""This is a function which creates or replaces the collisionshape
	of the world."""
	boxes = []
	
	if deleteids != []:
		for box in deleteids:
			pybullet.removeBody(box)
	
	halfsize = int(math.floor(size/2.0))
	
	for x in range(-halfsize, halfsize):
		for y in range(-halfsize, halfsize):
			shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,halfExtents=[2,2,0.1])
			boxId = pybullet.createMultiBody(0,shape,-1,[(4*x),(4*y),world[x][y]+basez],[0,0,0])
			boxes.append(boxId)
	
	return boxes
