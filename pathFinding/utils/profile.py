"""
Lets you accumulate time for profiling, by name.
"""
from functools import wraps
from threading import Condition, RLock
import time


class TimeDict():

    def __init__(self):
        self._children = {}
        self.totalTime = 0.0
        self.numCalls = 0
        self._startTime = None

    def startAccumulate(self, startTime):
        self._startTime = startTime
                
    def endAccumulate(self, endTime):
        self.totalTime += endTime - self._startTime
        self.numCalls += 1

    def getChild(self, name):
        if not name in self._children:
            self._children[name] = TimeDict()
        return self._children[name]

    def result(self, name, totalTime, indent):
        string = ""
        string += indent + "Total time for {0}: {1:.2f}s\n".format(name, self.totalTime)
        
        if self.numCalls > 0:
            avg = self.totalTime / self.numCalls
        else:
            avg = 0.0
        string += indent + "Avg. time for {0}: {1:.2f}s\n".format(name, avg)
        if totalTime > 0:
            percent = 100.0 * self.totalTime / totalTime
        else:
            percent = 0.0
        string += indent + "Rel. time for {0}: {1:.2f}%\n".format(name, percent)
        
        subIndent = indent + "  "
        for name, child in self._children.iteritems():
            string += child.result(name, self.totalTime, subIndent)
        return string

    def aggregate(self, aggDict):
        
        for name, child in self._children.iteritems():
            if not name in aggDict:
                aggDict[name] = 0.0
            aggDict[name] += child.totalTime
            child.aggregate(aggDict)


class Profiler():

    def __init__(self):
        self.timeStack = [TimeDict()]

    def startAccumulate(self, name, startTime):
        parent = self.timeStack[-1]
        timeDict = parent.getChild(name)
        timeDict.startAccumulate(startTime)
        self.timeStack.append(timeDict)
                
    def endAccumulate(self, endTime):
        timeDict = self.timeStack.pop()
        timeDict.endAccumulate(endTime)
    
    def result(self):
        timeDict = self.timeStack[-1]
        totalTime = 0.0
        for child in timeDict._children.values():
            totalTime += child.totalTime
        timeDict.totalTime = totalTime
        timeDict.numCalls = 1
        return timeDict.result("All", totalTime, "")

    def aggregate(self):
        timeDict = self.timeStack[-1]
        aggDict = {}
        timeDict.aggregate(aggDict)
        return aggDict


_lock = Condition(RLock())
_profiler = Profiler()


# Times the function and accumulates it time cost to the given name.
# As suggested by Kalan I should have a decorator for timing!
def accumulate(name):

    def decorator(func):
    
        @wraps(func)
        def wrapper(*args, **kwargs):
            with _lock:
                _profiler.startAccumulate(name, time.time())
            try:
                return func(*args, **kwargs)
            finally:
                with _lock:
                    _profiler.endAccumulate(time.time())

        return wrapper

    return decorator


def start(name):
    with _lock:
        _profiler.startAccumulate(name, time.time())

    
def end():
    with _lock:
        _profiler.endAccumulate(time.time())


def result():
    with _lock:
        return _profiler.result()


def aggregate():
    with _lock:
        return _profiler.aggregate()


def printAggregate():
    with _lock:
        times = _profiler.aggregate()
    
    csv = ""
    for name in times:
        csv += name + " , "
    csv += "\n"
    for name, time in times.iteritems():
        csv += str(time) + " , "
    print csv


def reset():
    global _profiler
    with _lock:
        _profiler = Profiler()

