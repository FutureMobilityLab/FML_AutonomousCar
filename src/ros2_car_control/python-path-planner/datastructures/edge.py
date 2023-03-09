"""File: edge.py

Description: Weight Edge class for instantiation of edge objects in Tree and Graph based datastructures.
"""
class WeightedEdge:
    def __init__(self, weight):
        self._weight = weight
        
    def get_weight(self):
        return self._weight