import numpy
import pygame
from OpenGL import *
from OpenGL.GL import *
from OpenGL.GL.shaders import *

class TextCollection():
	"""A Text Collection is a collection of all the text items to be rendered on
	the window. You could easily just use a list, but this is a matter of convenience and
	easy code reading."""
	def __init__(self, displaysize, font_location):
		self.all_text = []
		self.vertices = numpy.array([], numpy.float32)
		self.texcoords = numpy.array([], numpy.float32)
		self.display = displaysize
		self.font_location = font_location
		vertex_shader_code = """
			attribute vec3 a_Position;
			attribute vec2 a_texCoord;

			varying vec2 v_texCoord;

				void main()
				{
					gl_Position = vec4(a_Position, 1.0);
					v_texCoord = a_texCoord;
				}
		"""
		fragment_shader_code = """
			varying vec2 v_texCoord;

			uniform sampler2D s_texture;

			void main()
			{
				gl_FragColor = texture2D(s_texture, v_texCoord);
			}
		"""
		
		vertexShader = compileShader(vertex_shader_code, GL_VERTEX_SHADER)
		fragmentShader = compileShader(fragment_shader_code, GL_FRAGMENT_SHADER)

		self.program = glCreateProgram()
		glAttachShader(self.program, vertexShader)
		glAttachShader(self.program, fragmentShader)

		glLinkProgram(self.program)
		
		self.stextLoc = glGetUniformLocation(self.program, "s_texture")
	def add_text(self, text, fontwidth, text_x, text_y, center=False):
		fontheight = float(fontwidth)/0.6 # If you're screaming at me for using a magic number, you're justified. This is the aspect ratio of the letter textures.

		inputwidth = float(fontwidth)/float(self.display[0])
		inputheight = float(fontheight)/float(self.display[1])
		
		text_vertices = numpy.array([], numpy.float32)
		text_texcoords = numpy.array([], numpy.float32)
		
		if center:
			text_x -= (inputwidth * len(text) * 1.1)/2
		
	
		for character in text:
			if character != " ":
				text_vertices = numpy.append(text_vertices, [0.0 + text_x, 0.0 + text_y, 0.0,
						inputwidth + text_x, 0.0 + text_y, 0.0,
						inputwidth + text_x, inputheight + text_y, 0.0,
						
						0.0 + text_x, 0.0 + text_y, 0.0,
						0.0 + text_x, inputheight + text_y, 0.0,
						inputwidth + text_x, inputheight + text_y, 0.0
						])
		
				text_texcoords = numpy.append(text_texcoords, [0.0, 0.0,
						1.0, 0.0,
						1.0, 1.0,
						0.0, 0.0,
						0.0, 1.0,
						 1.0, 1.0])
		
			text_x += inputwidth*1.1
		
		self.all_text.append(text)
		self.vertices = numpy.append(self.vertices, text_vertices)
		self.texcoords = numpy.append(self.texcoords, text_texcoords)
	def get_text(self):
		total_text_string = ""
		
		for text_string in self.all_text:
			total_text_string += text_string
		
		return total_text_string
	def get_vertices(self):
		return self.vertices
	def get_texcoords(self):
		return self.texcoords
	def empty(self):
		self.all_text = []
		self.vertices = []
		self.texcoords = []
	def render(self): # I probably should put this in the render module, but I don't want this code to be complicated.
		glDepthRange(0.0, 0.01)
		
		glEnable(GL_TEXTURE_2D)
		glEnable(GL_BLEND)
		
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, self.vertices)
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, self.texcoords)

		glEnableVertexAttribArray(0)
		glEnableVertexAttribArray(1)

		glBindAttribLocation(self.program, 0, "a_Position")
		glBindAttribLocation(self.program, 1, "a_texCoord")
	
		glUseProgram(self.program)
		
		offset = 0
	
		text = self.get_text()
		nospacestext = ""
		for char in text:
			if char != " ":
				nospacestext += char
	
		for char in nospacestext:
			glActiveTexture(GL_TEXTURE0)
			glBindTexture(GL_TEXTURE_2D, set_texture(char, self.font_location))
			glUniform1i(self.stextLoc, 0)

			glDrawArrays(GL_TRIANGLES, offset, 6)
			offset += 6
		
		glUseProgram(0)
	
		glDisable(GL_TEXTURE_2D)
		glDisable(GL_BLEND)
		
		glDepthRange(0.0, 1.0)

def set_texture(char, font_location):
	img = pygame.image.load(font_location+char+".png")
	imgdata = pygame.image.tostring(img, "RGBA", 1)
	imgwidth, imgheight = img.get_size()

	texid = glGenTextures(1)
	glBindTexture(GL_TEXTURE_2D, texid)
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, imgwidth, imgheight, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, imgdata)
        
	glGenerateMipmap(GL_TEXTURE_2D)

	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	return texid
