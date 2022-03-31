#!/usr/bin/env python
from setuptools import setup, find_packages


setup(
    author="Marcos Thiago Piumbini",
    author_email='marcos.piumbini@lcad.inf.ufes.br',
    python_requires='>=3.6',
    description=("Instalação das dependências AdaBins"),
    install_requires=[
        "torch==1.6.0",
        "torchvision==0.7.0",
        "pytorch3d",
        "Pillow",
        "wandb",
        "tqdm",
        "matplotlib",
        "scikit-learn",
        "scikit-image",
        "opencv-python==4.2.0.32"
    ],
    include_package_data=True,
    keywords='AdaBins',
    name='AdaBins',
    packages=find_packages(),
    setup_requires=[],
    test_suite='tests',
    tests_require=[],
    url='https://github.com/LCAD-UFES/carmen_lcad/tree/master/src/deep_mapper',
    version='0.0.1',
)

