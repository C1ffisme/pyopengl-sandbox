import pybullet
import math

def groundVertices(size, basez, world):
	"""This is a function which takes a set of vertices, the size of the
	 desired square and base z and returns the triangles for OpenGL to render.
	 
	 It also creates the collision shape for the terrain."""
	vertices = []
	
	for x in range(0, size-1): # Size - 1 because annoying out-of-range errors
		for y in range(0, size-1):
			vertices.append((4*x, 4*y, basez + world[x][y]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
			
			vertices.append((4*x + 4, 4*y + 4, basez + world[x + 1][y + 1]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
	
	
	return vertices
