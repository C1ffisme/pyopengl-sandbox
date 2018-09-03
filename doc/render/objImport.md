# OBJ Import
_(objImport.py)_

### Description

OBJ Import contains a function for loading OBJ files and returning them in a way
that can be read by other functions.

## Functions

### importObj(directory)

This function creates a cube in the current PyBullet simulation.

*Arguments:*
- `directory` is a string leading from the working directory to the OBJ file.

*Returns:*
`objectvertexlist`

- `objectvertexlist` is a list of three-integer tuples. Each set of three tuples represents a triangle.
