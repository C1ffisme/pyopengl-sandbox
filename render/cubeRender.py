import pybullet
import math

def createCube(pos, size=1, orn=[0,0,0]):
	"""This function creates a cube using the size of the cube, 
	the position and the orientation in euler rotation."""
	cubeStartOrientation = pybullet.getQuaternionFromEuler(orn)

	shape = pybullet.createCollisionShape(pybullet.GEOM_BOX,halfExtents=[size,size,size])
	boxId = pybullet.createMultiBody(1,shape,-1,pos,cubeStartOrientation)
	
	return boxId, size

# This list describes the triangles of the most basic cube.
basic_cube = [
	(-1,1,-1), # Bottom Face
	(1,-1,-1),
	(1,1,-1),
	(-1,1,-1),
	(1,-1,-1),
	(-1,-1,1),
	
	(-1,1,1), # Top Face
	(1,-1,1),
	(1,1,1),
	(-1,1,1),
	(1,-1,1),
	(-1,-1,1),
	
	(1,1,-1), # X+ Face
	(1,-1,1),
	(1,1,1),
	(1,1,-1),
	(1,-1,1),
	(1,-1,-1),
	
	(-1,1,-1), # X- Face
	(-1,-1,1),
	(-1,1,1),
	(-1,1,-1),
	(-1,-1,1),
	(-1,-1,-1),
	
	(1,1,-1), # Y+ Face
	(-1,1,1),
	(1,1,1),
	(1,1,-1),
	(-1,1,1),
	(-1,1,-1),
	
	(1,-1,-1), # Y- Face
	(-1,-1,1),
	(1,-1,1),
	(1,-1,-1),
	(-1,-1,1),
	(-1,-1,-1)
]

def cubeVertices(pos, size, orn):
	"""This function gives the triangles for OpenGL to render a cube given
	the position, size of the cube and euler rotation."""
	vertices = []
	
	cubex = pos[0]
	cubey = pos[1]
	cubez = pos[2]
	
	for vertex in basic_cube:
		v_x = (vertex[0] * size) + cubex
		v_y = (vertex[1] * size) + cubey
		v_z = (vertex[2] * size) + cubez
		
		vertices.append((v_x, v_y, v_z))
	
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
