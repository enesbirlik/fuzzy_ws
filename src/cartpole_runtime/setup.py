from setuptools import setup
import os
from glob import glob

package_name = 'cartpole_runtime'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='you@example.com',
    description='Cartpole runtime nodes and utilities',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartpole_gui = cartpole_runtime.cartpole_gui:main',
            'pid_controller = cartpole_runtime.pid_controller:main',
            'lqr_controller = cartpole_runtime.lqr_controller:main',
            'fuzzy_controller = cartpole_runtime.fuzzy_controller:main',
        ],
    },
)
