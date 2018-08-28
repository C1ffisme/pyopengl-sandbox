def importObj(directory):
	vertices = {}
	faces = []
	objectvertexlist = []
	
	vertex_number = 1
	
	for line in open(directory, "r"):
		if line[0] == "v" and line[1] != "n" and line[1] != "t":
			x, y, z = line[1:].split()
			vertices[vertex_number] = (float(x), float(y), float(z))
			vertex_number += 1
		elif line[0] == "f":
			face = line[1:].split()
			a = int(face[0].split("/")[0])
			b = int(face[1].split("/")[0])
			c = int(face[2].split("/")[0])
			faces.append((a, b, c))
	
	for face in faces:
		objectvertexlist.append(vertices[face[0]])
		objectvertexlist.append(vertices[face[1]])
		objectvertexlist.append(vertices[face[2]])
	
	return objectvertexlist
	
		
