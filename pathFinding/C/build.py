from distutils.core import setup, Extension

# Must be module's name
testModule = Extension('fastPathIntersect', ['fastPathIntersect.c', 'geometry.c'])

# Must be module's name
setup (name='fastPathIntersect',
       version='1.0',
       description='This is a test',
       ext_modules=[testModule])
