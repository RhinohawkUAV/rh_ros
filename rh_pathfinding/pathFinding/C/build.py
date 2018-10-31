from distutils.core import setup, Extension

# Must be module's name
testModule = Extension('fastPathIntersect', ['fastPathIntersect.c', 'geometry.c'])

# Must be module's name
setup (name='fastPathIntersect',
       version='1.0',
       description='Module for testing intersections between paths and obstacles.',
       ext_modules=[testModule])
