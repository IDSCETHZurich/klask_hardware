from setuptools import setup, find_packages

package_name = 'klask_interfaces_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tobimeier@ethz.ch',
    description='Python utilities for klask_interfaces',
    license='Apache-2.0',
    tests_require=['pytest'],
)
