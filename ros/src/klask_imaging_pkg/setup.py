"""Setup script for the klask_imaging_pkg package."""

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "klask_imaging_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # Include config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    package_data={
        package_name: ["data/*.npz"],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="tobimeier@student.ethz.ch",
    description="The imaging package for Klask robot",
    license="AGPL-3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "camera_node = klask_imaging_pkg.camera_node:main",
            "image_viewer = klask_imaging_pkg.image_viewer:main",
        ],
    },
)
