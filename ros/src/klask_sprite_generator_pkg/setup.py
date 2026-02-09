"""Setup file for the klask_sprite_generator_pkg package."""

from setuptools import find_packages, setup

package_name = "klask_sprite_generator_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="nisohm@ethz.ch",
    description="TODO: Package description",
    license="Apache-2.0",
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
