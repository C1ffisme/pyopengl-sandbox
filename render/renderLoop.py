from OpenGL import *
from OpenGL.GL import *
from OpenGL.GLU import *

def render_loop(program, vertex_array, color_array):
	"""This is basically the render loop if you have a color and vertex numpy array.
	This does not work with VBOs, use vbo_render() instead."""
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vertex_array)
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, color_array)
	
	glEnableVertexAttribArray(0)
	glEnableVertexAttribArray(1)

	glBindAttribLocation(program, 0, "a_Position")
	glBindAttribLocation(program, 1, "a_Color")
	
	glUseProgram(program)
	glDrawArrays(GL_TRIANGLES, 0, len(vertex_array)/3)
	
	glDisableVertexAttribArray(0)
	glDisableVertexAttribArray(1)
	glUseProgram(0)
	
def vbo_render(program, buffers, num_vertices):
	"""This renders the VBOs."""
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
