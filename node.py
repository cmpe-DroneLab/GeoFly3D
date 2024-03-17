import math

class Node:
    def __init__(self,coordinate_1,coordinate_2,coordinate_3):
        self.coordinates = [coordinate_1,coordinate_2,coordinate_3]  # (x,y,z) currently

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