# Text Render
_(textRender.py)_

### Description

Text Render contains the functions required to render text on the screen as a GUI element.

## Functions

### set_texture(char, font_location)

Set the texture for the text renderer. (DO NOT USE THIS. USE THE TEXT COLLECTION CLASS INSTEAD.)

*Arguments:*
- `char` is a character describing the text character you want to set the texture to.
- `font_location` is a directory describing where the png files for the font renderer can be found from the working directory.

*Returns:*
`texid`

- `texid` is an output from glGenTextures(). It needs to be fed directly into glBindTexture.

## Classes

### TextCollection(displaysize, font_location)

A text collection is a special object that holds all of the text to be rendered onto the screen, and it also has special functions to render that text too.

*Arguments:*
- `displaysize` is the size of the display to render text onto.
- `font_location` is a directory describing where the png files for the font renderer can be found from the working directory.

#### add_text(text, fontwidth, text_x, text_y, center)

Add a piece of text to the renderer.

*Arguments:*
- `text` is a string of characters you want to render.
- `fontwidth` is the size of the text to render.
- `text_x` describes where to put the text on the screen. -1.0 is all the way on the left, 1.0 is all the way on the right.
- `text_y` describes where to put the text on the screen. -1.0 is all the way on the bottom, 1.0 is all the way on the top.
- `center` says whether or not to center the text on the selected location, rather than starting at text_x and text_y. Defaults to False.

#### get_text()

Get the text to be rendered as one string. This is for rendering purposes and isn't intended for much else.

*Returns:*

- `totaltextstring` is a string containing all the strings to be rendered by the renderer.

#### get_vertices()

Gets the vertices to be rendered on the screen.

*Returns:*

- `vertices` is a numpy array of the vertices to be rendered.

#### empty()

Removes all text in the TextCollection. Currently the only way to remove text.

#### render()

This is a rendering function, much like those in `render/renderLoop.py`. Use in exactly the same places as those functions. (Preferably after your 3D rendering.)


