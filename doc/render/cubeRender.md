# Cube Render
_(cubeRender.py)_

### Description

Cube Render contains the functions required to create and calculate the vertices
of a cube.

## Functions

### createCube(pos, size, orn)

This function creates a cube in the current PyBullet simulation.

*Arguments:*
- `pos` is a tuple containing 3 numbers which describes the position the cube will be placed at.
- `size` is a number that describes the size of the cube.
- `orn` is a list containing 3 numbers which describes the euler rotation to be applied to the cube when placed.

*Returns:*
`boxId`, `size`

- `boxId` is the ID given to the new cube by PyBullet
- `size` is the size of the cube. (TODO: Figure out the size of the cube in cubeVertices instead.)

### cubeVertices(pos, size, orn)

This function takes the data of a cube and returns the triangles for rendering.

*Arguments:*
- `pos` is a tuple (though a list may work) of 3 numbers which describe the current position of the cube.
- `size` is the size of the cube. If this cube was created using createCube, you can use the same size argument.
- `orn` is the euler rotation to apply to the cube.

*Returns:*
`vertices`

- `vertices` is a list filled with 3 value tuples, or vertices. They describe the triangles of a cube which can be given to OpenGL.
