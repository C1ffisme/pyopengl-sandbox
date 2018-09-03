# Render Loop
_(renderLoop.py)_

### Description

Render Loop contains the isolated functions required to render VBOs or arrays of
vertices.

## Functions

### render_loop(program, vertex_array, color_array)

This function serves two purposes: To create the world collisionbox and to return the vertices to make the triangles to render the world.

*Arguments:*
- `program` is the program you would like to use to render these arrays.
- `vertex_array` is a numpy array of vertices.
- `color_array` is a numpy array of RGB colors for each corresponding vertex.

### vbo_render(program, buffers, num_vertices)

This function serves two purposes: To create the world collisionbox and to return the vertices to make the triangles to render the world.

*Arguments:*
- `program` is the program you would like to use to render these arrays.
- `buffers` is a list containing the buffer locations returned by glGenBuffers().
- `num_vertices` is an integer representing the number of vertices.
