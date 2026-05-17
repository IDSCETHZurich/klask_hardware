"""Setup file for the klask_system_id_pkg package."""

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "klask_system_id_pkg"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tobias Meier",
    maintainer_email="tobimeier@student.ethz.ch",
    description="The system identification package for Klask robot",
    license="AGPL-3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "acceleration_test_node = klask_system_id_pkg.acceleration_test_node:main",
            "analyze_acceleration_test = klask_system_id_pkg.acceleration_test_analyzer:main",
            "velocity_profile_node = klask_system_id_pkg.velocity_profile_node:main",
            "analyze_velocity_profile = klask_system_id_pkg.analyze_velocity_profile:main",
        ],
    },
)
