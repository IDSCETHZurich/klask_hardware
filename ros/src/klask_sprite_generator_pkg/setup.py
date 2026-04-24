"""Setup file for the klask_sprite_generator_pkg package."""

from setuptools import find_packages, setup
from glob import glob
import os

package_name = "klask_sprite_generator_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tobias Meier",
    maintainer_email="tobimeier@ethz.ch",
    description="The sprite generator package used for the renderer",
    license="AGPL-3.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "klask_sprite_generator_node = klask_sprite_generator_pkg.klask_sprite_generator_node:main"
        ],
    },
)
