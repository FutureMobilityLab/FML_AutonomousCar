"""File: node.py

Description: Node class for instantiation of node objects in Tree and Graph based datastructures.
"""

class Node():
    def __init__(self, index=(None, None), position=(None, None)):
        self._position = position
        self._index = index
        self._x = self._position[0]
        self._y = self._position[1]

    def get_position(self):
        return self._position

    def get_index(self):
        return self._index