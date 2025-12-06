from setuptools import find_packages, setup

package_name = 'klask_state_estimation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tobimeier@ethz.ch',
    description='This package contains the state estimation node for the KLASK robotic platform.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'klask_state_estimation = klask_state_estimation_pkg.klask_state_estimation:main'
        ],
    },
)
