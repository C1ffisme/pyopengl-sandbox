import pybullet
import math

def groundVertices(size, basez, world, base_position, view_range, chunk_view_adjustment):
	"""This is a function which takes a set of vertices, the size of the
	 desired square and base z and returns the triangles for OpenGL to render.
	 
	 It also creates the collision shape for the terrain."""
	vertices = []
	topsoil = []
	
	for view_x in range(-view_range,view_range):
		for view_y in range(-view_range, view_range):
			chunk_loc = (base_position[0] + view_x, base_position[1] + view_y)
			chunk_loc_xplus = (base_position[0] + view_x + 1, base_position[1] + view_y)
			chunk_loc_yplus = (base_position[0] + view_x, base_position[1] + view_y + 1)
			chunk_loc_xyplus = (base_position[0] + view_x + 1, base_position[1] + view_y + 1)
			if chunk_loc in world:
				current_chunk = world[chunk_loc]
				v,t = render_chunk(size, chunk_loc, current_chunk, basez, chunk_view_adjustment)
				vertices = vertices + v
				topsoil = topsoil + t
				if chunk_loc_xplus in world:
					current_chunk_xplus = world[chunk_loc_xplus]
					v,t = render_x_border(size, chunk_loc, current_chunk, current_chunk_xplus, basez, chunk_view_adjustment)
					vertices = vertices + v
					topsoil = topsoil + t
				
				if chunk_loc_yplus in world:
					current_chunk_yplus = world[chunk_loc_yplus]
					v,t = render_y_border(size, chunk_loc, current_chunk, current_chunk_yplus, basez, chunk_view_adjustment)
					vertices = vertices + v
					topsoil = topsoil + t
				
				if chunk_loc_xyplus in world:
					current_chunk_xyplus = world[chunk_loc_xyplus]
					corner_x = chunk_view_adjustment*(size-1) + (chunk_loc[0]*chunk_view_adjustment*size)
					corner_y = chunk_view_adjustment*(size-1) + (chunk_loc[1]*chunk_view_adjustment*size)
					vertices = vertices + render_square(chunk_view_adjustment, corner_x, corner_y, basez + len(current_chunk[size-1][size-1]), basez + len(current_chunk_xplus[0][size-1]), basez + len(current_chunk_yplus[size-1][0]), basez + len(current_chunk_xyplus[0][0]))
					topsoil = topsoil + color_square(current_chunk[size-1][size-1][-1], current_chunk_xplus[0][size-1][-1], current_chunk_yplus[size-1][0][-1], current_chunk_xyplus[0][0][-1])
				
	
	return vertices, topsoil

def render_chunk(size, base_position, world, basez, chunk_adjust):
	vertices = []
	topsoil = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for x in range(0, size-1): # Size - 1 because annoying out-of-range errors
		for y in range(0, size-1):
			square_x = chunk_adjust*x + (basex*size)
			square_y = chunk_adjust*y + (basey*size)
			vertices = vertices + render_square(chunk_adjust, square_x, square_y, basez+len(world[x][y]), basez+len(world[x+1][y]), basez+len(world[x][y+1]), basez+len(world[x+1][y+1]))
			topsoil = topsoil + color_square(world[x][y][-1], world[x+1][y][-1], world[x][y+1][-1], world[x+1][y+1][-1])
	
	return vertices, topsoil

def render_x_border(size, base_position, world1, world2, basez, chunk_adjust):
	vertices = []
	topsoil = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for y in range(0, size-1):
		square_y = chunk_adjust*y + (basey*size)
		vertices = vertices + render_square(chunk_adjust, chunk_adjust*(size-1) + (basex*size), square_y, basez+len(world1[size-1][y]), basez+len(world2[0][y]), basez+len(world1[size-1][y+1]), basez+len(world2[0][y+1]))
		topsoil = topsoil + color_square(world1[size-1][y][-1], world2[0][y][-1], world1[size-1][y+1][-1], world2[0][y+1][-1])
	
	return vertices, topsoil

def render_y_border(size, base_position, world1, world2, basez, chunk_adjust):
	vertices = []
	topsoil = []
	
	basex, basey = base_position
	basex *= chunk_adjust
	basey *= chunk_adjust
	
	for x in range(0, size-1):
		square_x = chunk_adjust*x + (basex*size)
		vertices = vertices + render_square(chunk_adjust, square_x, chunk_adjust*(size-1) + (basey*size), basez+len(world1[x][size-1]), basez+len(world1[x+1][size-1]), basez+len(world2[x][0]), basez+len(world2[x+1][0]))
		topsoil = topsoil + color_square(world1[x][size-1][-1], world1[x+1][size-1][-1], world2[x][0][-1], world2[x+1][0][-1])
	
	return vertices, topsoil

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

def color_square(c1, c2, c3, c4):
	topsoil = []
	
	# z1 : (0,0)
	# z2 : (1,0)
	# z3 : (0,1)
	# z4 : (1,1)
	
	topsoil.append(c1)
	topsoil.append(c2)
	topsoil.append(c3)
			
	topsoil.append(c4)
	topsoil.append(c2)
	topsoil.append(c3)
	
	return topsoil
