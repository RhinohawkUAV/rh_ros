from engine.vertex.vertex import Vertex


class VertexPriorityQueue:
    """
    A PriorityQueue which operates on Vertex objects.
    """

    def push(self, vertex):
        # type: (Vertex) -> None
        """
        Puts the given vertex into the queue.
        :param vertex:
        """
        pass

    def pop(self):
        # type: () -> Vertex
        """
        Returns the vertex to visit next or None if the queue is empty.
        :return:
        """
        pass

    def isEmpty(self):
        pass

    def __iter__(self):
        """Should allow iteration over vertices"""
        pass
