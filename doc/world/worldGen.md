# World Generation
_(worldGen.py)_

### Description

World Generation contains the functions required that render the terrain and create
the Terrain collision box.

## Functions

### worldGen(size, amplitude)

This function serves two purposes: To create the world collisionbox and to return the vertices to make the triangles to render the world.

*Arguments:*
- `size` is a number describing the size of the world to generate.
- `amplitude` is a number that describes the max Z value of the points in the generated world.

*Returns:*
`world`

- `world` is a list of lists of numbers. The location of the second list in the first list is the X value of the point, the location of the number in the second list is the Y value of the point, and the value of the number is the Z.

### resetWorldBoxes(size, basez, world, deleteids)

This function serves two purposes: To create the world collisionbox and to return the vertices to make the triangles to render the world.

*Arguments:*
- `size` is a number describing the size of the world to generate boxes for.
- `basez` is a number that describes the base Z value the world generates at.
- `world` is the world to generate boxes for.
- `deleteids` is a list of ids for the boxes of an older world collisionshape to delete before making new boxes.

*Returns:*
`boxes`

- `boxes` is a list of ids of boxes that make up the world collisionshape.

