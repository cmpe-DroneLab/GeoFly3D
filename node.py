import math
import haversine as hs
from haversine import Unit, Direction

class Node:
	def __init__(self, coordinate_1,coordinate_2,coordinate_3):
		self.coordinates = [float(coordinate_1),float(coordinate_2),float(coordinate_3)]  # (x,y,z) currently
		self.geoloc = (self.coordinates[0], self.coordinates[1])

	def calculate_distance(self,other):
		sum_square = 0
		for i in range(len(self.coordinates)):
			sum_square += (self.coordinates[i]-other.coordinates[i])**2
		return math.sqrt(sum_square)

	def calculate_transposed_distance(self,other):
		sum_square = 0
		for i in range(2):
			sum_square += (self.coordinates[i]-other.coordinates[i])**2
		return math.sqrt(sum_square)

	def calculate_geographic_distance(self, other):
		loc_other = (other.coordinates[0], other.coordinates[1])
		return hs.haversine(self.geoloc, loc_other,unit=Unit.METERS)

	def calculate_next_node(self, displacement, direction):
		return Node(hs.inverse_haversine(self.geoloc, displacement, direction), 0)
	## eq and hash are necessary for the functionalities of the 
	def __eq__(self,other):
		for i in range(len(self.coordinates)):
			if self.coordinates[i] != other.coordinates[i]:
				return False
		return True
	def __hash__(self):
		return hash((self.coordinates[0], self.coordinates[1], self.coordinates[2]))
