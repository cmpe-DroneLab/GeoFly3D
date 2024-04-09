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
	 
			
		
				
				