import pybullet
import math

def groundVertices(size, basez, world, base_position, view_range, chunk_view_adjustment):
	"""This is a function which takes a set of vertices, the size of the
	 desired square and base z and returns the triangles for OpenGL to render.
	 
	 It also creates the collision shape for the terrain."""
	vertices = []
	
	for view_x in range(-view_range,view_range):
		for view_y in range(-view_range, view_range):
			chunk_loc = (base_position[0] + view_x, base_position[1] + view_y)
			chunk_loc_xplus = (base_position[0] + view_x + 1, base_position[1] + view_y)
			chunk_loc_yplus = (base_position[0] + view_x, base_position[1] + view_y + 1)
			chunk_loc_xyplus = (base_position[0] + view_x + 1, base_position[1] + view_y + 1)
			if chunk_loc in world:
				current_chunk = world[chunk_loc]
				vertices = vertices + render_chunk(size, chunk_loc, current_chunk, basez, chunk_view_adjustment)
				if chunk_loc_xplus in world:
					current_chunk_xplus = world[chunk_loc_xplus]
					vertices = vertices + render_x_border(size, chunk_loc, current_chunk, current_chunk_xplus, basez, chunk_view_adjustment)
				
				if chunk_loc_yplus in world:
					current_chunk_yplus = world[chunk_loc_yplus]
					vertices = vertices + render_y_border(size, chunk_loc, current_chunk, current_chunk_yplus, basez, chunk_view_adjustment)
				
				if chunk_loc_xyplus in world:
					current_chunk_xyplus = world[chunk_loc_xyplus]
					corner_x = chunk_view_adjustment*(size-1) + (chunk_loc[0]*chunk_view_adjustment*size)
					corner_y = chunk_view_adjustment*(size-1) + (chunk_loc[1]*chunk_view_adjustment*size)
					vertices = vertices + render_square(chunk_view_adjustment, corner_x, corner_y, basez + current_chunk[size-1][size-1], basez + current_chunk_xplus[0][size-1], basez + current_chunk_yplus[size-1][0], basez + current_chunk_xyplus[0][0])
				
	
	return vertices

def render_chunk(size, base_position, world, basez, chunk_adjust):
	vertices = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for x in range(0, size-1): # Size - 1 because annoying out-of-range errors
		for y in range(0, size-1):
			square_x = chunk_adjust*x + (basex*size)
			square_y = chunk_adjust*y + (basey*size)
			vertices = vertices + render_square(chunk_adjust, square_x, square_y, basez + world[x][y], basez+world[x+1][y], basez+world[x][y+1], basez+world[x+1][y+1])
	
	return vertices

def render_x_border(size, base_position, world1, world2, basez, chunk_adjust):
	vertices = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for y in range(0, size-1):
		square_y = chunk_adjust*y + (basey*size)
		vertices = vertices + render_square(chunk_adjust, chunk_adjust*(size-1) + (basex*size), square_y, basez + world1[size-1][y], basez+world2[0][y], basez+world1[size-1][y+1], basez+world2[0][y+1])
	
	return vertices

def render_y_border(size, base_position, world1, world2, basez, chunk_adjust):
	vertices = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for x in range(0, size-1):
		square_x = chunk_adjust*x + (basex*size)
		vertices = vertices + render_square(chunk_adjust, square_x, chunk_adjust*(size-1) + (basey*size), basez + world1[x][size-1], basez+world1[x+1][size-1], basez+world2[x][0], basez+world2[x+1][0])
	
	return vertices

def render_square(chunk_adjust, base_x, base_y, z1, z2, z3, z4):
	vertices = []
	
	# z1 : (0,0)
	# z2 : (1,0)
	# z3 : (0,1)
	# z4 : (1,1)
	
	vertices.append((base_x, base_y, z1))
	vertices.append((base_x + chunk_adjust, base_y, z2))
	vertices.append((base_x, base_y + chunk_adjust, z3))
			
	vertices.append((base_x + chunk_adjust, base_y + chunk_adjust, z4))
	vertices.append((base_x + chunk_adjust, base_y, z2))
	vertices.append((base_x, base_y + chunk_adjust, z3))
	
	return vertices
