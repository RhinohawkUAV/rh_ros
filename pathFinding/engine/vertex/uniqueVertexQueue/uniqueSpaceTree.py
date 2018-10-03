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

    def __init__(self, minPosition, dims, coincidentDistance, borderPadding=0.001):
        # Pad all dimensions so that no valid point will violate boundaries due to float round off error
        minPosition = np.array(minPosition, np.double)
        dims = np.array(dims, np.double)

        self._minPosition = minPosition - dims * borderPadding
        self._dims = dims * (1.0 + borderPadding * 2.0)
        
        self._root = None
        self._coincidentDistanceSquared = coincidentDistance * coincidentDistance

    def insert(self, position):
        """
        Insert the position and return a uniqueness score.
        :param position: A 4D sequence of values representing what is being inserted
        :return: uniqueness
        """

        if self._root is None:
            self._root = createRoot(self._minPosition, self._dims, 1.0, position)
            return 1.0

        return self.insertRecurse(self._root, position)

    def insertRecurse(self, node, position):
        subIndices = node.calcSubIndices(position)
        index = _calcIndex(subIndices)
        child = node._children[index]
        # If the slot is empty just insert the position there
        if child is None:
            return node.createLeaf(subIndices, index, position).uniqueness
        
        if child.isLeaf():
            if self.isCoincident(child._leafPosition, position):
                return 0.0
            child.convertToNonLeaf()

        return self.insertRecurse(child, position)

    def isCoincident(self, position, position2):
        """
        Determines if value stored in this leaf matches the given position
        :return:
        """
        diff = position2 - position
        diff /= self._dims
        return np.dot(diff, diff) < self._coincidentDistanceSquared

        
#         return (self._leafPosition == position).sum() == 4
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
