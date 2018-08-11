from typing import Union

import numpy as np

_subIndexMultipliers = np.array([1, 2, 4, 8], np.int32)


def _calcIndex(subIndices):
    return int(np.dot(subIndices, _subIndexMultipliers))


def checkCoincident(p1, p2):
    # type: (np.ndarray,np.ndarray)->bool
    """
    Determines if each component of p1 is the same as p2.
    :param p1:
    :param p2:
    :return:
    """
    return (p1 == p2).sum() == 4

# Not clear what the best data structure is for this. Numpy supports kd-trees and Voronoi diagrams, but:
# kd-trees are not considered good in dynamic situations.
# Voronoi uses n^2 space, not clear if this is good for dynamic situations.
# R-trees are supposed to be good for dynamics, but they don't appear to be supported out of the box (there is some GIS version on the web).
# In a recent test, insertion into this tree accounted for ~1.2% of the computation time, so this is NOT the place to optimize.


class UniqueTree:
    """
    A 4-d quadtree-ish data structure used to determine uniqueness value.  The uniqueness of a node is a function of its depth.

    For efficiency this does not check for illegal values outside the range of the cell.  DON'T INSERT THESE VALUES!
    """

    def __init__(self, x, y, width, height, maximumSpeed, levelUniquenessFactor=0.5):
        minPosition = np.array([x, y, -maximumSpeed, -maximumSpeed], np.double)
        dims = np.array([width, height, 2.0 * maximumSpeed, 2.0 * maximumSpeed], np.double)

        self._root = UniqueNode(minPosition, dims, uniqueness=levelUniquenessFactor)
        self._levelUniquenessFactor = levelUniquenessFactor
        self._empty = True

    def insert(self, vertex):
        """
        Insert the vertex and return a uniqueness score.
        :param vertex:
        :return:
        """
        position = np.array([vertex.position[0], vertex.position[1], vertex.speed * vertex.unitVelocity[0], vertex.speed * vertex.unitVelocity[1]])

        if self._empty:
            self._empty = False
            self._insertRecurse(self._root, position)
            return 1.0

        return self._insertRecurse(self._root, position)

    def _insertRecurse(self, node, position):
        """
        Inserts the position into one of the 16 available children of node.
        This will recurse if the child is not empty.
        :param position:
        :return:
        """
        subIndices = node._calcSubIndices(position)
        index = _calcIndex(subIndices)

        # If the slot is empty just insert the position there
        if node._children[index] is None:
            node._children[index] = position
            return node._uniqueness

        # If there is an exising position (not a sub-tree), at the index where we want to insert, then 
        # 1. Replace the exising position with a subtree 
        # 2. Insert the replaced exising position into the subtree
        # 3. Insert the new position into the sub tree
        elif not isinstance(node._children[index], UniqueNode):
            existingPosition = node._children[index]

            # This point already exists.  Don't insert it and return a uniqueness of 0.0!
            if checkCoincident(position, existingPosition):
                return 0.0
            subTree = node._createSubTree(subIndices, node._uniqueness * self._levelUniquenessFactor)
            node._children[index] = subTree

            # Insert the existing position into the subtree 1st
            self._insertRecurse(subTree, existingPosition)

        return self._insertRecurse(node._children[index], position)

        
class UniqueNode:
    """
    Handles the work of actually inserting positions into sub-trees.
    """

    def __init__(self, minPosition, dims, uniqueness):
        """
        :param minPosition: The "upper-left" corner of the 4-d cell.
        :param dims: The dimensions of the 4-d cell
        :param uniqueness: The uniqueness value of this node.  Anything point directly inserted into the child array has this uniqueness value.
        """
        self._minPosition = np.array(minPosition, np.double)  # type: np.ndarray
        self._dims = np.array(dims, np.double)  # type: np.ndarray
        self._halfDims = self._dims / 2.0  # type: np.ndarray

        # Each slot contains a position OR an entire sub-tree
        self._children = [None] * 16  # type: Union[np.ndarray, UniqueNode]
        self._uniqueness = uniqueness  # type: float

    def _createSubTree(self, subIndices, uniqueness):
        """
        Create a new sub-tree root based on this cell and the given sub-indices.
        :param subIndices:
        :param uniqueness:
        :return:
        """
        subCellPosition = self._minPosition + subIndices * self._halfDims
        return UniqueNode(subCellPosition, self._halfDims, uniqueness)

    def _calcSubIndices(self, position):
        """
        Given a position, determine its "sub-indices".  This is a 4-element array of 0|1 determining if this goes
        "left" or "right" for each dimension.

        :param position:
        :return:
        """
        ratios = (position - self._minPosition)
        ratios /= self._halfDims
        subIndices = ratios.astype(np.int32)
        return subIndices
