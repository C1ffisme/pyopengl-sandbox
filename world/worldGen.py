import random
import pybullet
import math

def worldGen(size, amplitude=4):
	"""Temporary worldgen function. Will be replaced later with something
	that actually looks nice. Takes size and noise amplitude
	and returns chunk vertices for use in other functions."""
	worlda = []
	worldb = []
	world = []
	
	if size > 4:
		for x in range(0,size): # I'd make these smaller heightmaps be... well... smaller, but then I just run into annoying list index out of range errors.
			worlda.append([])
			for y in range(0,size):
				worlda[x].append(random.randint(0,amplitude))
				
		for x in range(0, size):
			worldb.append([])
			for y in range(0,size):
				worldb[x].append(worlda[int(math.floor(x/2.0))][int(math.floor(y/2.0))] + random.randint(-amplitude/3,amplitude/3))
		
		for x in range(0, size):
			world.append([])
			for y in range(0,size):
				z = worldb[int(math.floor(x/2.0))][int(math.floor(y/2.0))] + random.randint(-amplitude/4,amplitude/4)
				blocks = []
				if z < 1:
					z = 1
				
				for i in range(0,z):
					if i == (z-1):
						if z == 1:
							blocks.append(2)
						else:
							blocks.append(1)
					else:
						blocks.append(0)
				
				world[x].append(blocks)
	elif size < 2:
		print("WARNING: The worldsize entered is lower than the normal generator can accept! Using the legacy generator.")
		for x in range(0, size):
			world.append([])
			for y in range(0,size):
				
				blocks = []
				for i in range(0,random.randint(-amplitude,amplitude)):
					if i == (z-1):
						blocks.append(1)
					else:
						blocks.append(0)
				
				world[x].append(blocks)
	
	return world

def resetWorldBoxes(size, basez, player_chunk_position, world, deleteids=[]):
	"""This is a function which creates or replaces the collisionshape
	of the world."""
	boxes = []
	
	if deleteids != []:
		for box in deleteids:
			pybullet.removeBody(box)
	
	pybullet.resetSimulation()
	if player_chunk_position in world:
		for x in range(0, size):
			for y in range(0, size):
				shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,halfExtents=[2,2,0.1])
				boxId = pybullet.createMultiBody(0,shape,-1,[(4*x),(4*y),len(world[player_chunk_position][x][y])+basez],[0,0,0])
				boxes.append(boxId)
	
	return boxes
