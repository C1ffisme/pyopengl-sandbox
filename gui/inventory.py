 
def create_inv(width, height):
	inv = []
	for row in range(0,height):
		inv.append([])
		for column in range(0,width):
			inv[row].append(["none",0])
	
	return inv

def add_to_inv(inventory, item):
	for row in inventory:
		for column in row:
			if column[0] == "none":
				column[0] = item
				return inventory
	
	print("Failure: Inventory full")
	return inventory

def remove_from_inv(inventory, item):
	for row in inventory:
		for column in row:
			if column[0] == item:
				column[0] = "none"
				return inventory
	
	print("Failure: Inventory full")
	return inventory

def get_inv_image(item):
	colors = {"none":[0.7,0.7,0.7],"dirt":[0.5,0.3,0.1]}
	
	return colors[item]

def inv_contains(inventory, item):
	for row in inventory:
		for column in row:
			if column[0] == item:
				return True
	
	return False
	
			
