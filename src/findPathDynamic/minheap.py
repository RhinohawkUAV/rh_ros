import heapq


class MinHeap:
    """
    What you'd expect.  There is probably a better one, but this is simple and meets our needs.
    """

    def __init__(self):
        self._heap = []
        self._inc = 0

    def isEmpty(self):
        return len(self._heap) == 0

    def __len__(self):
        return len(self._heap)

    def push(self, priority, data):
        # This is given as a 2nd argument, after priority, to break ties.  Unique incrementing value
        heapq.heappush(self._heap, (priority, self._inc, data))
        self._inc += 1

    def getTop(self):
        (priority, dontCare, data) = self._heap[0]
        return data

    def pop(self):
        (cost, dontCare, data) = heapq.heappop(self._heap)
        return data

    def getTopWithPriority(self):
        (priority, dontCare, data) = self._heap[0]
        return (priority, data)

    def popWithPriority(self):
        (priority, dontCare, data) = heapq.heappop(self._heap)
        return (priority, data)
