import numpy
import math
import inventory

def create_inventory(width, height, display, inv=[]):
	gui_height = 0.7
	gui_width = 0.7
	
	gui_v = numpy.array([-gui_width,-gui_height,0.1, -gui_width, gui_height,0.1, gui_width,-gui_height,0.1,  -gui_width,gui_height,0.1, gui_width,-gui_height,0.1, gui_width,gui_height,0.1], numpy.float32)
	gui_c = numpy.array([0.4,0.4,0.4, 0.4,0.4,0.4, 0.4,0.4,0.4, 0.4,0.4,0.4, 0.4,0.4,0.4, 0.4,0.4,0.4], numpy.float32)
	
	slot_size = 0.2
	margin_size = 0.1
	half_height = float(height)/2.0
	half_width = float(width)/2.0
	
	flo_h = math.floor(half_height)
	flo_w = math.floor(half_width)
	
	if flo_h == 0:
		flo_h = 1
	if flo_w == 0:
		flo_w = 1
	
	inv_x = 0
	
	for row in range(-int(math.floor(half_height)), int(math.ceil(half_height))):
		inv_y = 0
		
		for column in range(-int(math.floor(half_width)), int(math.ceil(half_width))):
			
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w,(gui_height-margin_size)*(row)/flo_h,0])
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w + slot_size,(gui_height-margin_size)*(row)/flo_h,0])
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w,(gui_height-margin_size)*(row)/flo_h + slot_size,0])
			
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w + slot_size,(gui_height-margin_size)*(row)/flo_h + slot_size,0])
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w + slot_size,(gui_height-margin_size)*(row)/flo_h,0])
			gui_v = numpy.append(gui_v, [(gui_width-margin_size)*(column)/flo_w,(gui_height-margin_size)*(row)/flo_h + slot_size,0])
			
			if inv == []:
				gui_c = numpy.append(gui_c, [0.7,0.7,0.7, 0.7,0.7,0.7, 0.7,0.7,0.7,  0.7,0.7,0.7, 0.7,0.7,0.7, 0.7,0.7,0.7])
			else:
				for i in range(0,6):
					gui_c = numpy.append(gui_c, [inventory.get_inv_image(inv[inv_x][inv_y][0])])
			
			inv_y += 1
		
		inv_x += 1
				
			
	return gui_v, gui_c
