import numpy as np

_subIndexMultipliers = np.array([1, 2, 4, 8], np.int32)


def _calcIndex(subIndices):
    return int(np.dot(subIndices, _subIndexMultipliers))

# Not clear what the best data structure is for this. Numpy supports kd-trees and Voronoi diagrams, but:
# kd-trees are not considered good in dynamic situations.
# Voronoi uses n^2 space, not clear if this is good for dynamic situations.
# R-trees are supposed to be good for dynamics, but they don't appear to be supported out of the box (there is some GIS version on the web).
# In a recent test, insertion into this tree accounted for ~1.2% of the computation time, so this is NOT the place to optimize.


class UniqueTree:
    """
    A 4-d quadtree-ish data structure used to determine uniqueness of a 4-d value.  The uniqueness of a value is 
    more or less a measure of its distance from its closest neighbor.

    For efficiency this does not check for illegal values outside the range of the cell.  DON'T INSERT THESE VALUES!
    """

    def __init__(self, x, y, width, height, maximumSpeed):
        self._minPosition = np.array([x, y, -maximumSpeed, -maximumSpeed], np.double)
        self._dims = np.array([width, height, 2.0 * maximumSpeed, 2.0 * maximumSpeed], np.double)
        self._root = None

    def insert(self, position):
        """
        Insert the position and return a uniqueness score.
        :param position: A 4D sequence of values representing what is being inserted
        :return: uniqueness
        """

        if self._root is None:
            self._root = createRoot(self._minPosition, self._dims, 1.0, position)
            return 1.0

        return self._root.insert(position)


def createRoot(minPosition, dims, uniqueness, position):
    root = UniqueNode(minPosition, dims, uniqueness)
    root._children = [None] * 16
    root._leafPosition = position
    return root


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
        
        # TODO: Do not need to contantly remake arrays
        self._minPosition = np.array(minPosition, np.double)  # type: np.ndarray
        self._dims = np.array(dims, np.double)  # type: np.ndarray
        self._halfDims = self._dims / 2.0  # type: np.ndarray
        self.uniqueness = uniqueness

    def insert(self, position):
        subIndices = self.calcSubIndices(position)
        index = _calcIndex(subIndices)
        child = self._children[index]
        # If the slot is empty just insert the position there
        if child is None:
            return self.createLeaf(subIndices, index, position).uniqueness
        
        if child.isLeaf():
            if child.isCoincident(position):
                return 0.0
            child.convertToNonLeaf()

        return child.insert(position)

    def createLeaf(self, subIndices, index, position):
        subCellPosition = self._minPosition + subIndices * self._halfDims
        childUniqueness = self.uniqueness * 0.5
        leafNode = UniqueNode(subCellPosition, self._halfDims, childUniqueness)
        leafNode._children = None
        leafNode._leafPosition = position
        self._children[index] = leafNode
        return leafNode

    def convertToNonLeaf(self):
        self._children = [None] * 16
        subIndices = self.calcSubIndices(self._leafPosition)
        index = _calcIndex(subIndices)
        self._children[index] = self.createLeaf(subIndices, index, self._leafPosition)

    def isLeaf(self):
        return self._children is None

    # TODO: Add cutoff value for coincidence
    def isCoincident(self, position):
        """
        Determines if value stored in this leaf matches the given position
        :return:
        """
        return (self._leafPosition == position).sum() == 4

    def calcSubIndices(self, position):
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
