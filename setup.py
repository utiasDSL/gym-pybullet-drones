import setuptools

'''
All imports and meta data is added to package from pyproject.toml. setup.py is 
kept as it enables install in editable mode which is not supported in poetry builds.
'''
setuptools.setup()