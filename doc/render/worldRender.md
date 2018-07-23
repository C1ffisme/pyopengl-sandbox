# World Render
_(worldRender.py)_

### Description

World Render contains the functions required to render the point-based terrain.

## Functions

### groundVertices(size, basez, world)

This function serves two purposes: To create the world collisionbox and to return the vertices to make the triangles to render the world.

*Arguments:*
- `size` is a number describing the size of the area to generate centered around the origin.
- `basez` is a number that describes the base Z value the world generates at.
- `world` is a list of lists of numbers. The location of the second list in the first list is the X value of the point, the location of the number in the second list is the Y value of the point, and the value of the number is the Z.

*Returns:*
`vertices`

- `vertices` is a list of vertices describing the triangles needed to render the world.
