from distutils.core import setup, Extension

#Must be module's name
testModule = Extension('testmod', ['test.c'])

#Must be module's name
setup (name = 'testmod',
       version = '1.0',
       description = 'This is a test',
       ext_modules = [testModule])
