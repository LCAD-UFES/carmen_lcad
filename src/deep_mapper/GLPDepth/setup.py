#!/usr/bin/env python
from setuptools import setup, find_packages


setup(
    author="Marcos Thiago Piumbini",
    author_email='marcos.piumbini@lcad.inf.ufes.br',
    python_requires='>=3.7',
    description=("Instalação das dependências GLPDepth"),
    install_requires=[
        "torch>=1.6.0",
        "h5py>=3.6.0",
        "scipy>=1.7.3",
        "opencv-python>=4.5.5",
        "mmcv>=1.4.3",
        "timm>=0.5.4",
        "albumentations>=1.1.0",
        "tensorboardX>=2.4.1",
        "gdown>=4.2.1"
    ],
    include_package_data=True,
    keywords='GLPDepth',
    name='GLPDepth',
    packages=find_packages(),
    setup_requires=[],
    test_suite='tests',
    tests_require=[],
    url='https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/deep_mapper',
    version='0.0.1',
)
