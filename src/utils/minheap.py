import heapq


class MinHeap:
    """
    What you'd expect.  There is probably a better one, but this is simple and meets our needs.
    """

    def __init__(self):
        self._heapList = []
        self._inc = 0

    def isEmpty(self):
        return len(self._heapList) == 0

    def __len__(self):
        return len(self._heapList)

    def push(self, priority, data):
        # This is given as a 2nd argument, after priority, to break ties.  Unique incrementing value
        heapq.heappush(self._heapList, (priority, self._inc, data))
        self._inc += 1

    def getTop(self):
        (priority, dontCare, data) = self._heapList[0]
        return data

    def pop(self):
        (cost, dontCare, data) = heapq.heappop(self._heapList)
        return data

    def getTopWithPriority(self):
        (priority, dontCare, data) = self._heapList[0]
        return (priority, data)

    def popWithPriority(self):
        (priority, dontCare, data) = heapq.heappop(self._heapList)
        return (priority, data)

    def __iter__(self):
        return _heapIter(self)

    def __getitem__(self, index):
        return self._heapList[index]


class _heapIter:
    def __init__(self, heap):
        self._heap = heap
        self._index = 0

    def next(self):
        if self._index < len(self._heap):
            (priority, dontCare, data) = self._heap[self._index]
            self._index += 1
            return data
        else:
            raise StopIteration()
