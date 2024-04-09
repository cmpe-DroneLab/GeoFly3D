from node import Node
import math
class Scan_area:
	def __init__(self,edges=None):
		self.edges = []
		self.nodes = {}
		if edges is not None:
			self.check_edges(edges)

	# Calculate the increment value 
	def calculate_increment(self,height,intersection_ratio):
		HFOV_degree   = 75.5                            # Check https://www.parrot.com/en/drones/anafi/technical-specifications
		HFOV_rad      = (HFOV_degree*math.pi)/180
		increment     = math.tan(HFOV_rad/2) * height * 2 * (1-intersection_ratio)  # Increment by variable intersection rates of the height of the taken picture
		return abs(increment)

	def add_edge(self,edge):
		if (len(edge) != 2):
			return False
		if (edge[0] != edge[1]):
			self.edges.append(edge)
			if edge[0] not in list(self.nodes.keys()):
				self.nodes[edge[0]] = [edge[1]]
			else:
				self.nodes[edge[0]].append(edge[1])
			if edge[1] not in list(self.nodes.keys()):
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
		n             = 3
		while len(queue) > 0:
			node = queue.pop()
			visited_nodes.append(node)
			for neighbor in self.nodes[node]:
				if neighbor not in visited_nodes and neighbor not in queue:
				    queue.append(neighbor)
		current_direction = None
		direction         = None
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
				sub_sum = list(self.nodes.keys())[j].coordinates[i]
			coordinates_center.append(sub_sum/point_count)
		return coordinates_center


	def create_route(self,start_point,height,intersection_ratio):
		if self.is_convex() == False:
			print("Please Check the Polygon")
			return []
		route = []
		longest_edge   = None
		biggest_length = 0
		increment = self.calculate_increment(height,intersection_ratio)
		for edge in self.edges:
			distance = edge[0].calculate_distance(edge[1])
			if distance > biggest_length:
				biggest_length = distance
				longest_edge = edge.copy()
		p1            = longest_edge[0]
		p2            = longest_edge[1]
		convex_center = self.get_polygon_center()
		vector_1      = []
		angle         = None
		vector_2      = []
		dist_1        = start_point.calculate_distance(p1)
		dist_2        = start_point.calculate_distance(p2)
		if dist_1 < dist_2:
			#route.append(p1)
			for i in range(len(p1.coordinates)-1):
				vector_1.append(p2.coordinates[i] - p1.coordinates[i])
				vector_2.append(convex_center[i] - p1.coordinates[i])
		else:
			route.append(p2)
			for i in range(len(p1.coordinates)-1):
				vector_1.append(p1.coordinates[i] - p2.coordinates[i])
				vector_2.append(convex_center[i] - p2.coordinates[i])
		angle            = math.atan2(vector_1[1],vector_1[0])
		angle_orthogonal = None
		cross_multiple = (vector_1[0]*vector_2[1])-(vector_1[1]*vector_2[0])
		if cross_multiple > 0:
			angle_orthogonal = angle + (math.pi/2)
		else:
			angle_orthogonal = angle - (math.pi/2)
		#print("Angle of the longest edge:",180*angle/math.pi)
		#print("Angle of the normal edge:",180*angle_orthogonal/math.pi)
		direction_vector = [math.cos(angle_orthogonal),math.sin(angle_orthogonal)] 
		edge_point       = []
		for edge in self.edges:
			edge_point.append([edge,[]])
			true_angle          = None
			true_vector         = None
			start_vertex        = None
			end_vertex          = None
			vertex_1            = edge[0]
			vertex_2            = edge[1]
			direction_1         = [vertex_1.coordinates[0]-vertex_2.coordinates[0],vertex_1.coordinates[1]-vertex_2.coordinates[1]] 
			direction_2         = [vertex_2.coordinates[0]-vertex_1.coordinates[0],vertex_2.coordinates[1]-vertex_1.coordinates[1]]
			angle_1             = math.atan2(direction_1[1],direction_1[0])
			angle_2             = math.atan2(direction_2[1],direction_2[0])
			direction_vector_1  = [math.cos(angle_1),math.sin(angle_1)]
			direction_vector_2  = [math.cos(angle_2),math.sin(angle_2)]
			dot_1               = (direction_vector_1[0]*direction_vector[0])+(direction_vector_1[1]*direction_vector[1])
			dot_2               = (direction_vector_2[0]*direction_vector[0])+(direction_vector_2[1]*direction_vector[1])
			is_orthogonal       = False
			#print("Dot_1:",dot_1)
			#print("Dot_2:",dot_2) 
			if dot_1 > 0.00001:
				true_angle   = angle_1
				true_vector  = direction_1.copy()
				start_vertex = vertex_2
				end_vertex   = vertex_1
				true_vector.append(end_vertex.coordinates[2]-start_vertex.coordinates[2])
			elif dot_2 > 0.00001:
				true_angle   = angle_2
				true_vector  = direction_2.copy()
				start_vertex = vertex_1
				end_vertex   = vertex_2
				true_vector.append(end_vertex.coordinates[2]-start_vertex.coordinates[2])
			else:
				start_vertex = vertex_1
				end_vertex   = vertex_2
				is_orthogonal = True
			if (is_orthogonal == True):
				#edge_point[edge] = [start_vertex,end_vertex]
				print(start_vertex.coordinates[0],start_vertex.coordinates[1])
				node_start = Node(start_vertex.coordinates[0],start_vertex.coordinates[1],start_vertex.coordinates[2])
				node_end   = Node(end_vertex.coordinates[0],end_vertex.coordinates[1],end_vertex.coordinates[2])
				edge_point[-1][1].append(node_start)
				edge_point[-1][1].append(node_end)
				#print(len(edge_point[-1][1]))
			else:
				angle_edge  = true_angle - angle_orthogonal
				edge_inc    = abs(increment / math.cos(angle_edge))
				edge_length = vertex_1.calculate_distance(vertex_2)
				#print("Edge Length:",edge_length)
				limit       = int((edge_length//edge_inc) + 1)
				for i in range(limit):
					coordinate_1 = start_vertex.coordinates[0]+(edge_inc*i*math.cos(true_angle))
					coordinate_2 = start_vertex.coordinates[1]+(edge_inc*i*math.sin(true_angle))
					coordinate_3 = ((i/(limit)) * true_vector[2]) + start_vertex.coordinates[2]
					new_node     = Node(coordinate_1,coordinate_2,coordinate_3)       
					edge_point[-1][1].append(new_node)
		return edge_point
		

				
		

	 
			
		
				
				