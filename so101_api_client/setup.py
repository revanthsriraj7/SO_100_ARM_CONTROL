from setuptools import setup, find_packages

setup(
    name='so101-api',
    version='1.0.0',
    description='Python client SDK for the SO-101 dashboard REST API.',
    packages=find_packages(exclude=['tests']),
    install_requires=['requests>=2.25'],
    python_requires='>=3.8',
)
