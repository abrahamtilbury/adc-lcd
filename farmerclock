// PIC16F887
// 4MHz external crystal oscillator
// MikroC PRO for PIC
// 5V supply

// Segment mapping: RD0=A, RD1=B, RD2=C, RD3=D, RD4=E, RD5=F, RD6=G, RD7=DP. 
// A logic 1 illuminates the corresponding segment on the assumed common-cathode display
const unsigned char digitPatterns[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

position_offsets = [
	(-2, 0),
	(2, 0),
	(6, 0),
	(10, 0)
]

def go_to(x, y):
	while get_pos_x() != x:
		if get_pos_x() < x:
			move(East)
		else:
			move(West)
	while get_pos_y() != y:
		if get_pos_y() < y:
			move(North)
		else:
			move(South)

digits = {
	0: [a, b, c, d, e, f],
	1: [b, c],
	2: [a, b, d, e, g],
	3: [a, b, c, d, g],
	4: [b, c, f, g],
	5: [a, c, d, f, g],
	6: [a, c, d, e, f, g],
	7: [a, b, c],
	8: [a, b, c, d, e, f, g],
	9: [a, b, c, d, f, g]
}

print(get_world_size())
while True:
	# Read time once per display refresh
	time = get_time() // 1
	hours = (time // 3600) % 24
	minutes = (time // 60) % 60
	clock_digits = [
		hours // 10,
		hours % 10,
		minutes // 10,
		minutes % 10
	]
	clock_digits = [8, 8, 8, 8]
	quick_print(clock_digits)
	# Construct the coordinates that should be ON
	active_tiles = {}
	for i in range(4):
		dx, dy = position_offsets[i]
		for segment in digits[clock_digits[i]]:
			for coord in segment:
				x, y = coord
				active_tiles[(x + dx, y + dy)] = True
	quick_print("Active tiles:", len(active_tiles))
	quick_print("Top-left:", (0,6) in active_tiles)
	# Visit the display, column by column
	go_to(0,2)
	for column in range(15):
		for row in range(5):
			x = get_pos_x()
			y = get_pos_y()
			should_be_soil = (x, y) in active_tiles
			is_soil = get_ground_type() == Grounds.Soil
			if should_be_soil != is_soil:
				till()
			if (x, y) == (0, 6):
				quick_print("Required:", should_be_soil)
				quick_print("Ground:", get_ground_type())
				quick_print("Entity:", get_entity_type())
			# Alternate direction each column
			if row < 4:
				if column % 2 == 0:
					move(North)
				else:
					move(South)
		if column < 14:
			move(East)