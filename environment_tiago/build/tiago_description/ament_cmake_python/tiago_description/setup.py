from setuptools import find_packages
from setuptools import setup

setup(
    name='tiago_description',
    version='4.7.1',
    packages=find_packages(
        include=('tiago_description', 'tiago_description.*')),
)
