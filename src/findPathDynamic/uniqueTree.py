import numpy as np

from findPathDynamic.interfaces import VertexUniquenessGroup
from findPathDynamic.vertex import Vertex

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


class UniqueTree(VertexUniquenessGroup):
    """
    A 4-d quadtree-ish data structure used to determine uniqueness value.  Every node in the tree has a uniqueness
    value proportional to the size of its edge.  The root has a uniqueness of 1.0 and each level down the
    tree has 1/2 the uniqueness of the previous level.

    For efficiency this does not check for illegal values outside the range of the cell.  DON'T INSERT THESE VALUES!
    """

    def __init__(self, x, y, width, height, maximumSpeed):
        minPosition = np.array([x, y, -maximumSpeed / np.math.sqrt(2.0), -maximumSpeed / np.math.sqrt(2.0)],
                               np.double)
        dims = np.array([width, height,
                         2.0 * maximumSpeed / np.math.sqrt(2.0),
                         2.0 * maximumSpeed / np.math.sqrt(2.0)],
                        np.double)

        self._root = UniqueNode(minPosition, dims, uniqueness=0.5)
        self._empty = True

    def insert(self, vertex):
        position = np.array([vertex.position[0], vertex.position[1], vertex.velocity[0], vertex.velocity[1]])
        if self._empty:
            self._empty = False
            self._root.insert(position)
            return 1.0

        return self._root.insert(position)


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
        self._minPosition = np.array(minPosition, np.double)
        self._dims = np.array(dims, np.double)
        self._halfDims = self._dims / 2.0
        self._children = [None] * 16
        self._uniqueness = uniqueness

    def insert(self, position):

        subIndices = self.getSubIndices(position)
        index = _calcIndex(subIndices)
        # If the slot is empty just insert the position there
        if self._children[index] is None:
            self._children[index] = position
            return self._uniqueness

        # If there is already a sub-tree at the location, then insert into the sub-tree
        elif not type(self._children[index]) is UniqueNode:
            existingPosition = self._children[index]
            # This point already exists.  Don't insert it and return a uniqueness of 0.0!
            if checkCoincident(position, existingPosition):
                return 0.0
            subTree = self.createSubTree(subIndices)
            self._children[index] = subTree

            # Insert the existing position into the subtree 1st
            subTree.insert(existingPosition)

        return self._children[index].insert(position)

    def getSubIndices(self, position):
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

    def createSubTree(self, subIndices):
        subCellPosition = self._minPosition + subIndices * self._halfDims
        return UniqueNode(subCellPosition, self._halfDims, self.calcSubUniqueness())

    def calcSubUniqueness(self):
        """
        Determines a sub-tree's uniqueness based on this tree's uniqueness.
        :return:
        """
        return self._uniqueness / 2.0


if __name__ == "__main__":
    tree = UniqueTree(0, 0, 64, 64, 64 * np.math.sqrt(2.0))

    print tree.insert(Vertex(np.array([24, 24], np.double), np.array([48, 48], np.double), 0, 0))
    print tree.insert(Vertex(np.array([48, 48], np.double), np.array([-48, -48], np.double), 0, 0))
    print tree.insert(Vertex(np.array([48, 48], np.double), np.array([48, 48], np.double), 0, 0))
    print tree.insert(Vertex(np.array([20, 20], np.double), np.array([52, 52], np.double), 0, 0))
