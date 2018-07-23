import pybullet
import math

def groundVertices(size, basez, world):
	"""This is a function which takes a set of vertices, the size of the
	 desired square and base z and returns the triangles for OpenGL to render.
	 
	 It also creates the collision shape for the terrain."""
	vertices = []
	
	halfsize = int(math.floor(size/2.0))
	
	for x in range(-halfsize, halfsize):
		for y in range(-halfsize, halfsize):
			vertices.append((4*x, 4*y, basez + world[x][y]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
			
			vertices.append((4*x + 4, 4*y + 4, basez + world[x + 1][y + 1]))
			vertices.append((4*x + 4, 4*y, basez + world[x + 1][y]))
			vertices.append((4*x, 4*y + 4, basez + world[x][y + 1]))
	
	
	return vertices
