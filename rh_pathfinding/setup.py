from setuptools import setup, find_packages, Extension

fastPathIntersectModule = Extension('fastPathIntersect',
                ['src/rh_pathfinding/C/fastPathIntersect.c', 'src/rh_pathfinding/C/geometry.c'])

setup(name='rhpathfinder',
      version='0.0.1',
      description='Path finding code for Rhinohawk project',
      author='Stephen Pheiffer',
      install_requires=[
          'numpy', 'typing',
      ],
      packages=find_packages(),
      ext_modules=[fastPathIntersectModule])

