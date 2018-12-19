from OpenGL import *
from OpenGL.GL import *
import render.cubeRender as cubeRender
import pybullet
import numpy
import math

def render_loop(program, cubes):
	"""This is basically the render loop if you have a color and vertex numpy array.
	This does not work with VBOs, use vbo_render() instead."""
	
	glDepthRange(0.01, 1.0) # This is important so that text is not rendered over.
	
	glEnableVertexAttribArray(0)
	glEnableVertexAttribArray(1)

	glBindAttribLocation(program, 0, "a_Position")
	glBindAttribLocation(program, 1, "a_Color")
	
	for cube in cubes:
		cubeId = cube[0]
		size = cube[1]
		
		cubePos, cubeOrn = pybullet.getBasePositionAndOrientation(cubeId)
		
		cubepoints = cubeRender.cubeVertices(size)
		
		vertex_array = numpy.array([], numpy.float32)
		color_array = numpy.array([], numpy.float32)
		
		for vertex in cubepoints:
			vertex_array = numpy.append(vertex_array, [vertex[0],vertex[1],vertex[2]])
			color_array = numpy.append(color_array, [0.7,0.7,0.7])
		
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vertex_array)
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, color_array)
	
		glUseProgram(program)
		
		glMatrixMode(GL_MODELVIEW)
		
		glPushMatrix()
		
		glTranslatef(cubePos[0], cubePos[1], cubePos[2])
		
		euler = pybullet.getEulerFromQuaternion(cubeOrn)
		degeuler = (math.degrees(euler[0]), math.degrees(euler[1]), math.degrees(euler[2]))
		glRotatef(degeuler[2], 0.0, 0.0, 1.0)
		glRotatef(degeuler[1], 0.0, 1.0, 0.0)
		glRotatef(degeuler[0], 1.0, 0.0, 0.0)
		
		glDrawArrays(GL_TRIANGLES, 0, len(cubepoints))
		
		glPopMatrix()
	
	glDisableVertexAttribArray(0)
	glDisableVertexAttribArray(1)
	glUseProgram(0)
	
	glDepthRange(0.0, 1.0)
	
def vbo_render(program, buffers, num_vertices):
	"""This renders the VBOs."""
	glDepthRange(0.01, 1.0)
	
	glBindBuffer(GL_ARRAY_BUFFER, buffers[0])
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
	
	glBindBuffer(GL_ARRAY_BUFFER, buffers[1])
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
	glBindBuffer(GL_ARRAY_BUFFER, 0)
	
	glEnableVertexAttribArray(0)
	glEnableVertexAttribArray(1)
	
	glBindAttribLocation(program, 0, "a_Position")
	glBindAttribLocation(program, 1, "a_Color")
	
	glUseProgram(program)
	
	glDrawArrays(GL_TRIANGLES, 0, num_vertices)
	
	glDisableVertexAttribArray(1)
	glDisableVertexAttribArray(0)
	
	glUseProgram(0)
	glDepthRange(0.0, 1.0)
