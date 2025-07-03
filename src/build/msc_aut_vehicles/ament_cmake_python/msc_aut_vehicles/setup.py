from setuptools import find_packages
from setuptools import setup

setup(
    name='msc_aut_vehicles',
    version='0.0.0',
    packages=find_packages(
        include=('msc_aut_vehicles', 'msc_aut_vehicles.*')),
)
