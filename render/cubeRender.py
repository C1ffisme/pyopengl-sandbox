import pybullet
import math
import objImport

def createCube(pos, size=1, orn=[0,0,0]):
	"""This function creates a cube using the size of the cube, 
	the position and the orientation in euler rotation."""
	for axes in orn:
		radorn = (math.radians(orn[0]), math.radians(orn[1]), math.radians(orn[2]))
		
	
	cubeStartOrientation = pybullet.getQuaternionFromEuler(radorn)

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

basic_round = objImport.importObj("Round.obj")

def cubeVertices(size):
	"""Legacy function, soon to be removed."""
	vertices = []
	
	for vertex in basic_round:
		v_x = (vertex[0] * size)
		v_y = (vertex[1] * size)
		v_z = (vertex[2] * size)
		
		vertices.append((v_x, v_y, v_z))
	
	return vertices
