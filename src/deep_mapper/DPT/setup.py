#!/usr/bin/env python
from setuptools import setup, find_packages


setup(
    author="Marcos Thiago Piumbini",
    author_email='marcos.piumbini@lcad.inf.ufes.br',
    python_requires='>=3.6',
    description=("Instalação das dependências DPT"),
    install_requires=[
        "torch==1.10.1",
        "torchvision",
        "opencv-python==4.5.5.62",
        "timm",
        "Pillow",
    ],
    include_package_data=True,
    keywords='DPT',
    name='DPT',
    packages=find_packages(),
    setup_requires=[],
    test_suite='tests',
    tests_require=[],
    url='https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/deep_mapper',
    version='0.0.1',
)

