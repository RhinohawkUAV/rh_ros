from findPathDynamic.vertex import Vertex


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
        """
        Returns the vertex to visit next or None if the queue is empty.
        :return:
        """
        pass


class VertexUniquenessGroup:
    """
    Stores a set of vertices.  Supports the operation of adding a new vertex and in the process determining its
    uniqueness.

    Uniqueness is a number in the range [0,1].

    1 - this is the only vertex
    0 - this exactly matches an existing vertex

    """

    def insert(self, vertex):
        # type: (Vertex) -> float
        """
        Insert this vertex and return its uniqueness value.
        :param vertex:
        :return:
        """
        pass
