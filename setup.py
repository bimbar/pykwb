# -*- coding: utf-8 -*-
from setuptools import setup

setup(
    name = 'pykwb',
    version = '0.1.0',
    packages = ['pykwb'],
    package_data = {'pykwb': ['messages.csv']},
    python_requires = '>=3.9',
    install_requires = ['pyserial-asyncio-fast>=0.16'],
    description = 'KWB Easyfire serial library, for inclusion into homeassistant',
    author = 'Markus Peter',
    author_email = 'mpeter@emdev.de',
    url = 'https://github.com/bimbar/pykwb.git',
    license ="MIT"
)
