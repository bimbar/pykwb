# -*- coding: utf-8 -*-
from setuptools import setup, find_packages

setup(
    name = 'pykwb',
    version = '0.0.7',
    packages = ['pykwb'],
    install_requires = ['pyserial>=3.0.1'],
    description = 'KWB Easyfire serial library, for inclusion into homeassistant',
    author = 'Markus Peter',
    author_email = 'mpeter@emdev.de',
    url = 'https://github.com/bimbar/pykwb.git',
    license ="MIT"
)
