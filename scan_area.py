from node import Node
import math
import matplotlib.pyplot as plt
import numpy as np
class Scan_area:
	def __init__(self,edges=None):
		self.edges = []
		self.nodes = {}
		if edges is not None:
			self.check_edges(edges)

	# Calculate the base increment value. The increment is adapted to fit the slope of every edge 
	def calculate_increment(self,height,intersection_ratio):
		HFOV_degree   = 75.5                            # Check https://www.parrot.com/en/drones/anafi/technical-specifications
		HFOV_rad      = (HFOV_degree*math.pi)/180
		increment     = math.tan(HFOV_rad/2) * height * 2 * (1-intersection_ratio)  # Increment by variable intersection rates of the height of the taken picture
		return abs(increment)
	# Add a single edge to the shape
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

	# Add a list of edges
	def check_edges(self,edges):
		for edge in edges:
			self.add_edge(edge)

	# Check if the edges form a closed polygon. Every unique node should have a degree of 2		
	def is_closed(self):
		for node in self.nodes:
			if (len(self.nodes[node]) != 2):
				return False
		return True
	
	# We support convex shapes. Concav shapes are out of question
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
		direction         = None
		n                 = len(visited_nodes)	
		for i in range(len(visited_nodes)):    
			p1 = visited_nodes[i]
			p2 = visited_nodes[(i + 1) % n]
			p3 = visited_nodes[(i + 2) % n]
			cross_product = ((p2.coordinates[0] - p1.coordinates[0]) * (p3.coordinates[1] - p1.coordinates[1])) - ((p2.coordinates[1] - p1.coordinates[1]) * (p3.coordinates[0] - p1.coordinates[0]))
			if cross_product == 0:
				continue  
			elif cross_product > 0.0001:
				current_direction = 1  
			else:
				current_direction = -1
			# Check if the direction changes
			if direction is None:
				direction = current_direction
			elif direction != current_direction:
				return False  
		return True

	# It's necessary to determine the direction of incrementing
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

	# A Route is created given that the polygon is closed and convex
	def create_route(self,start_point,height,intersection_ratio,enable_rotate,angle_rotation,n1,n2):
		if self.is_convex() == False:
			print("Please Check the Polygon")
			return []
		route = []
		increment     = self.calculate_increment(height,intersection_ratio)
		first_point   = None
		second_point  = None
		angle         = None
		convex_center = self.get_polygon_center()
		vector_1      = []
		vector_2      = []
		transformed_list    = []
		if enable_rotate:
			longest_edge   = None
			biggest_length = 0
			for edge in self.edges:
				distance = edge[0].calculate_distance(edge[1])
				if distance > biggest_length:
					biggest_length = distance
					longest_edge = edge.copy()
			p1            = longest_edge[0]
			p2            = longest_edge[1]	
			dist_1        = start_point.calculate_distance(p1)
			dist_2        = start_point.calculate_distance(p2)				
			if dist_1 < dist_2:
				first_point  = p1
				second_point = p2
				for i in range(len(p1.coordinates)-1):
					vector_1.append(p2.coordinates[i] - p1.coordinates[i])
					vector_2.append(convex_center[i] - p1.coordinates[i])
			else:
				first_point  = p2
				second_point = p1
				for i in range(len(p1.coordinates)-1):
					vector_1.append(p1.coordinates[i] - p2.coordinates[i])
					vector_2.append(convex_center[i] - p2.coordinates[i])
		else:
			first_point = n1
			second_point= n2
			for i in range(len(n1.coordinates)-1):
				vector_1.append(n2.coordinates[i] - n1.coordinates[i])
				vector_2.append(convex_center[i] - n1.coordinates[i])
		
		angle            = math.atan2(vector_1[1],vector_1[0])
		angle_orthogonal = None
		cross_multiple = (vector_1[0]*vector_2[1])-(vector_1[1]*vector_2[0])
		if cross_multiple > 0:
			angle_orthogonal = angle + (math.pi/2)
		else:
			angle_orthogonal = angle - (math.pi/2)		
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
				node_start = Node(start_vertex.coordinates[0],start_vertex.coordinates[1],start_vertex.coordinates[2])
				node_end   = Node(end_vertex.coordinates[0],end_vertex.coordinates[1],end_vertex.coordinates[2])
				edge_point[-1][1].append(node_start)
				edge_point[-1][1].append(node_end)
				edge_point[-1].append(math.pi/2)
			else:
				edge_point[-1].append(true_angle)
				angle_edge  = true_angle - angle_orthogonal
				edge_inc    = abs(increment / math.cos(angle_edge))
				edge_length = vertex_1.calculate_distance(vertex_2)
				limit       = int((edge_length//edge_inc) + 1)
				for i in range(limit):
					coordinate_1 = start_vertex.coordinates[0]+(edge_inc*i*math.cos(true_angle))
					coordinate_2 = start_vertex.coordinates[1]+(edge_inc*i*math.sin(true_angle))
					coordinate_3 = ((i/(limit)) * true_vector[2]) + start_vertex.coordinates[2]
					new_node     = Node(coordinate_1,coordinate_2,coordinate_3)       
					edge_point[-1][1].append(new_node)
		
		#sorted_nodes = self.sort_points(edge_point,angle_orthogonal,angle,first_point,second_point)
		sorted_nodes      = self.sort_points_alt(edge_point,first_point,second_point)
		if enable_rotate:
			transformed_list = self.route_transformation(first_point,second_point,angle_rotation,start_point,height,intersection_ratio)
		return sorted_nodes,transformed_list

		

	def sort_points_alt(self,edge_point,first_point,second_point):
		initial_direction   = None   # 1 right  -1 left relative to the orthogonal vector cutting mid point of the longest edge
		sorted_list         = []
		points              = []
		index               = 0
		for i in range(len(edge_point)):		
			for k in range(len(edge_point[i][1])):
				points.append(edge_point[i][1][k])
		for i in range(len(points)):
			if points[i] == first_point:
				index = i
				break
		sorted_list.append(points[index])
		points.pop(index)
		index = 0
		for i in range(len(points)):
			if points[i] == second_point:
				index = i
				break
		sorted_list.append(points[index])
		points.pop(index)
		current_side = 1	
		counter = 1
		points_two = []
		for point in points:
			if point not in self.nodes:
				points_two.append(point)
		while True:
			if (len(points_two) == 0):
				break
			min_dist  = float("inf")
			min_index = 0
			if current_side == 1:
				for i in range(len(points_two)):
					dist = second_point.calculate_distance(points_two[i])
					if  dist < min_dist:
						min_dist  = dist
						min_index = i
				second_point = points_two[min_index]
			elif current_side == -1:
				for i in range(len(points_two)):
					dist = first_point.calculate_distance(points_two[i])
					if  dist < min_dist:
						min_dist  = dist
						min_index = i
				first_point = points_two[min_index]
			else:
				pass
			sorted_list.append(points_two[min_index])
			points_two.pop(min_index)
			if counter % 2 != 0:
				current_side = current_side * -1		
			counter = counter + 1
		return sorted_list
		
	def route_transformation(self,first_node,second_node,angle_rotation,start_point,height,intersection_ratio):
		e1,d1,e2,d2          =self.divide_shape(first_node,second_node,angle_rotation)
		self.edges.clear()
		self.nodes.clear()
		self.edges           = e1
		self.nodes           = d1
		new_route_1,_        = self.create_route(start_point,height,intersection_ratio,False,angle_rotation,e1[0][0],e1[0][1])
		self.edges.clear()
		self.nodes.clear()
		self.edges           = e2
		self.nodes           = d2
		new_route_2,_        = self.create_route(start_point,height,intersection_ratio,False,angle_rotation,e2[0][0],e2[0][1])
		back_up_route        = []
		for node in new_route_2:
			if node not in new_route_1:
				back_up_route.append(node)	
		back_up_route.reverse()
		new_route = back_up_route + new_route_1[1:]
		return new_route

	def divide_shape(self,first_node,second_node,angle_rotation):
		visited_nodes = []
		queue         = [first_node]
		sorted_queue  = [] 
		while len(queue) > 0:
			node = queue.pop()
			visited_nodes.append(node)
			if node == first_node:
					queue.append(second_node)
			else:
				for neighbor in self.nodes[node]:
					if neighbor not in visited_nodes and neighbor not in queue:
						queue.append(neighbor)
		selected_index = 0
		is_found       = False
		#for node in visited_nodes:
		#	print(node.coordinates)
		for i in range(len(visited_nodes)-2):
			first_coordinates  = visited_nodes[i+1].coordinates[0:2]
			second_coordinates = visited_nodes[i+2].coordinates[0:2]
			sampled_points = self.interpolate(first_coordinates,second_coordinates)
			for point in sampled_points:
				temp  = Node(point[0],point[1],visited_nodes[0].coordinates[2])
				angle = self.get_angle(visited_nodes[0],visited_nodes[1],temp)
				if -0.0001 <= angle - angle_rotation <=0.0001:
					visited_nodes.insert(i+2,Node(point[0],point[1],visited_nodes[i+1].coordinates[2]))
					selected_index = i + 3
					is_found = True
					break
			if is_found:
				break
		node_split_1     = visited_nodes[:selected_index]
		node_split_2     = [first_node]+visited_nodes[selected_index:]
		first_edge_set   = []
		second_edge_set  = []
		first_node_dict  = {}
		second_node_dict = {}
		for i in range(len(node_split_1)):
			first_edge_set.append([node_split_1[i],node_split_1[(i+1)%len(node_split_1)]])
			first_node_dict[node_split_1[i]] = [node_split_1[i-1],node_split_1[(i+1)%len(node_split_1)]]
		for i in range(len(node_split_2)):
			second_edge_set.append([node_split_2[i],node_split_2[(i+1)%len(node_split_2)]])
			second_node_dict[node_split_2[i]] = [node_split_2[i-1],node_split_2[(i+1)%len(node_split_2)]]
		return first_edge_set,first_node_dict,second_edge_set,second_node_dict
	
	def get_angle(self,first_node,second_node,third_node):
		#plt.scatter([first_node.coordinates[0],second_node.coordinates[0],third_node.coordinates[0]],[first_node.coordinates[1],second_node.coordinates[1],third_node.coordinates[1]])
		#plt.show()
		length_A   = first_node.calculate_distance(third_node)
		length_1   = first_node.calculate_distance(second_node)
		length_2   = second_node.calculate_distance(third_node)
		vector_1   = [first_node.coordinates[0]-second_node.coordinates[0],first_node.coordinates[1]-second_node.coordinates[1]]
		vector_2   = [third_node.coordinates[0]-second_node.coordinates[0],third_node.coordinates[1]-second_node.coordinates[1]]
		dot_pro    = (vector_1[0]*vector_2[0])+(vector_1[1]*vector_2[1])
		angle_A    = math.acos(dot_pro/(length_1*length_2+0.0001))
		fin_ang    = math.asin(length_2*math.sin(angle_A)/length_A)
		return fin_ang
	
	def interpolate(self,fc,sc):
		x1, y1 = fc
		x2, y2 = sc
		slope = (y2 - y1) / (x2 - x1)
		x_values = np.linspace(min(x1,x2), max(x1,x2), 100)
		y_values = y1 + slope * (x_values - x1)
		sampled_points = np.column_stack((x_values[1:-1], y_values[1:-1]))
		return sampled_points


	 
			
		
				
				
