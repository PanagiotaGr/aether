from setuptools import find_packages, setup

setup(
    name="aether-sim",
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=["numpy>=1.24", "pyyaml>=6.0", "matplotlib>=3.7"],
)
