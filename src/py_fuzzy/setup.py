from setuptools import find_packages, setup

package_name = 'py_fuzzy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enesb',
    maintainer_email='enesbirlik0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cartpole_pid_controller = py_fuzzy.cartpole_pid_controller:main',
            'cartpole_fuzzy_controller = py_fuzzy.cartpole_fuzzy_controller:main',
            'cartpole_data_plotter = py_fuzzy.cartpole_data_plotter:main',
            'cartpole_lookup_controller = py_fuzzy.cartpole_lookup_controller:main'

        ],
    },
)
