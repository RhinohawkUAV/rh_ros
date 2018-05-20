"""
Lets you accumulate time for profiling, by name.
"""
from functools import wraps
from threading import Condition, RLock
import time

_timeDict = {}
_numDict = {}
_lock = Condition(RLock())


# Times the function and accumulates it time cost to the given name.
# As suggested by Kalan I should have a decorator for timing!
def accumulate(name):

    def decorator(func):
    
        @wraps(func)
        def wrapper(*args, **kwargs):
            with _lock:
                if not name in _numDict:
                    _numDict[name] = 0
                    _timeDict[name] = 0.0
            start = time.clock()
            try:
                return func(*args, **kwargs)
            finally:
                elapsedTime = time.clock() - start
                with _lock:
                    if not name in _numDict:
                        _numDict[name] = 0
                        _timeDict[name] = 0.0
    
                _timeDict[name] += elapsedTime
                _numDict[name] += 1

        return wrapper

    return decorator


def start(name):
    start = time.clock()
    with _lock:
        _timeDict[name] -= start

    
def end(name):
    end = time.clock()
    with _lock:
        _timeDict[name] += start
        _numDict[name] += 1


def result():
    with _lock:
        string = ""
        totalTime = 0.0
        for key, keyTotalTime in _timeDict.iteritems():
            string += "Total time for {0}: {1:.2f}s\n".format(key, keyTotalTime)
            totalTime += keyTotalTime
        for key, keyTotalTime in _timeDict.iteritems():
            string += "Avg. time for {0}: {1:.2f}s\n".format(key, keyTotalTime / _numDict[key])
    
        for key, keyTotalTime in _timeDict.iteritems():
            string += "Rel. time for {0}: {1:.2f}%\n".format(key, 100 * keyTotalTime / totalTime)
        
        return string
        
