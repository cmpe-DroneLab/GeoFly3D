from node import Node

class scan_area:
	def __init__(self,edges=None):
		if edges is None: 
			edges = []
			self.nodes = {}
		else:
			self.check_edges(edges)		

	def add_edge(self,edge):
		if (len(edge) != 2):
			return False
		if (edge[0] != edge[1]):
			self.edges.append(edge)
			if edge[0] not in self.nodes:
				self.nodes[edge[0]] = [edge[1]]
			else:
				self.nodes[edge[0]].append(edge[1])
			if edge[1] not in self.nodes:
				self.nodes[edge[1]] = [edge[0]]
			else:
				self.nodes[edge[1]].append(edge[0])
				return True
		else:
			return False
		
	def check_edges(self,edges):
		for edge in edges:
			self.add_edge(edge)
			
	def is_closed(self):
		for node in self.nodes:
			if (len(self.nodes[node]) != 2):
				return False
		return True
	

	def is_convex(self):
		if self.is_closed() == False:
			return False
		first_node    = list(self.nodes.keys())[0]
		visited_nodes = []
		queue         = [first_node]
		while len(queue) > 0:
			node = queue.pop()
			visited_nodes.append(node)
			for neighbor in self.nodes[node]:
				if neighbor not in visited_nodes and neighbor not in queue:
				    queue.append(neighbor)
		current_direction = None
		for i in range(len(visited_nodes)):    
			p1 = visited_nodes[i]
			p2 = visited_nodes[(i + 1) % n]
			p3 = visited_nodes[(i + 2) % n]
			cross_product = (p2.coordinates[0] - p1.coordinates[0]) * (p3.coordinates[1] - p2.coordinates[1]) - (p2.coordinates[1] - p1.coordinates[1]) * (p3.coordinates[0] - p2.coordinates[0])	
			if cross_product == 0:
				continue  
			elif cross_product > 0:
				current_direction = 1  
			else:
				current_direction = -1  
			# Check if the direction changes
			if direction is None:
				direction = current_direction
			elif direction != current_direction:
				return False  
		return True

	def get_polygon_center(self):
		point_count         = len(list(self.nodes.keys()))
		coordinates_center  = []
		coordinate_count    = len(list(self.nodes.keys())[0].coordinates)
		for i in range(coordinate_count):
			sub_sum = 0
			for j in range(point_count):
				sub_sum = list(self.nodes.keys())[j][i]
			coordinates_center.append(sub_sum/point_count)
		return coordinates_center


	def create_route(self,start_point):
		if self.is_convex() == False:
			print("Please Check the Polygon")
			return []
		route = []
		longest_edge   = None
		biggest_length = 0
		for edge in self.edges:
			distance = edge[0].calculate_distance(edge[1])
			if distance > biggest_length:
				biggest_length = distance
				longest_edge = edge.copy()
		p1            = longest_edge[0]
		p2            = longest_edge[1]
		convex_center = self.get_polygon_center()

		vector_1      = []
		vector_2      = []
		dist_1        = start_point.calculate_distance(p1)
		dist_2        = start_point.calculate_distance(p2)
		if dist_1 < dist_2:
			route.append(p1)
			for i in range(len(p1.coordinates)-1):
				vector_1.append(p2.coordinates[i] - p1.coordinates[i])
				vector_2.append(convex_center.coordinates[i] - p1.coordinates[i])
		else:
			route.append(p2)
			for i in range(len(p1.coordinates)-1):
				vector_1.append(p1.coordinates[i] - p2.coordinates[i])
				vector_2.append(convex_center.coordinates[i] - p2.coordinates[i])
		cross_multiple = (vector_1[0]*vector_2[1])-(vector_1[1]*vector_2[0])
		if cross_multiple = 
		

	 
			
		
				
				