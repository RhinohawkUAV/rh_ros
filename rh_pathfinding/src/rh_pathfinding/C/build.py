from distutils.core import setup, Extension
import fastPathIntersect

# Must be module's name
fastPathIntersectModule = Extension('fastPathIntersect', ['fastPathIntersect.c', 'geometry.c'])

# Must be module's name
setup (name='fastPathIntersect',
       version='1.0',
       description='Module for testing intersections between paths and obstacles.',
       ext_modules=[fastPathIntersectModule])
