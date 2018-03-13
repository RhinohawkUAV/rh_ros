import heapq
import math


class GridHeap:
    """
    A heap-ish data structure for storing data which is associated with a cost and an (x,y) coordinate.

    Push operation, will lookup the minimum cost of the (x,y) coordinate in a MinCostGrid.  If within acceptance threshold,
    the min cost will be updated and this block of data will be added to the heap.

    Pop operation, will return the cheapest AND "acceptable" item in the heap (or None).  To be acceptable, the item
    needs to be within acceptance threshold of the minimum cost for the associate (x,y) coordinate in the MinCostGrid.
    """

    def __init__(self, *minCostGridArgs):
        self._minCostGrid = MinCostGrid(*minCostGridArgs)
        self._heap = []

    def push(self, x, y, cost, data):
        # Update minimum cost at (x,y) bin if appropriate
        if self._minCostGrid.submitCost(x, y, cost):
            # Add data to heap, IF this was accepted
            heapq.heappush(self._heap, (cost, (x, y, data)))

    def pop(self):
        """
        Find the lowest cost item in the heap, whose cost is within acceptance, of minimum cost for the bin
        corresponding to its (x,y) coordinate.
        :return: Lowest cost, acceptable item OR None if there are no acceptable items left
        """
        while len(self._heap) > 0:
            # Get the lowest cost item in the heap
            (cost, (x, y, data)) = self._popNext()

            # While this is the lowest cost overall, it may no longer be within the acceptance threshold of the
            # corresponding (x,y) bin
            if self._minCostGrid.isCostAcceptable(x, y, cost):
                return data

        return None

    def _popNext(self):
        return heapq.heappop(self._heap)


class MinCostGrid:
    """
    Maintains a GridBin of minimum costs.
    You can query if an (x,y) coordinate and associated estimated cost is within the _acceptanceThreshold of the
    known minimum cost of the corresponding bin.

    You can also update the minimum cost for a given (x,y) coordinate, by submitting new data.
    """

    def __init__(self, acceptanceThreshold, numBins, *gridBinArgs):
        # Used to determine if a cost is close enough to minimum, found so far, to be accepted
        self._acceptanceThreshold = acceptanceThreshold

        # A grid bin for looking up minimum costs
        self._gridBin = GridBin(numBins, *gridBinArgs)

        # Holds the minimum cost data.  Initialize minimum cost to infinity everywhere.
        self._minCosts = [float('inf')] * (numBins * numBins)

    def submitCost(self, x, y, cost):
        """
        Submit an (x,y) coordinate and associate cost of route to get there.  This will update the minimum cost at the
        corresponding bin, if appropriate and will tell you if this cost is within the acceptanceThreshold, of the
        minimum cost for this bin.
        :param x:
        :param y:
        :param cost:
        :return: Was this cost acceptable? cost <= minCost + acceptanceThreshold
        """
        index = self._gridBin.getBinIndex(x, y)
        minCost = self._minCosts[index]
        if cost < minCost:
            self._minCosts[index] = cost
            return True
        elif cost <= minCost + self._acceptanceThreshold:
            return True
        else:
            return False

    def isCostAcceptable(self, x, y, cost):
        """
        Query if the given cost is withing the acceptance threshold of the minimum cost as the bin associated with (x,y)
        What a great name for a method!
        :param x:
        :param y:
        :param cost:
        :return:
        """
        return cost <= self.getMin(x, y) + self._acceptanceThreshold

    def getMin(self, x, y):
        index = self._gridBin.getBinIndex(x, y)
        return self._minCosts[index]


class GridBin:
    """
    Does calculations for dividing up an area into evenly spaced bins for storage in an array of size numBins^2.
    For a given (x,y) it calculates the corresponding array index for lookup.
    """

    def __init__(self, numBins, x, y, width, height):
        self._x = x
        self._y = y
        self._width = width
        self._height = height
        self._numBins = numBins

    def getBinIndex(self, x, y):
        xBin = math.floor(self._numBins * (x - self._x) / self._width)
        if xBin < 0 or xBin >= self._numBins:
            return None
        yBin = math.floor(self._numBins * (y - self._y) / self._height)
        if yBin < 0 or yBin >= self._numBins:
            return None
        return xBin + self._numBins * yBin
