#!/usr/bin/env python
import os
from ros import pathFinderGui
if __name__ == '__main__':
    root = os.path.dirname(__file__)
    loadPath = os.path.normpath(os.path.join(root, "../scenarios/test1.json"))
    pathFinderGui.main(loadPath)