from setuptools import find_packages, setup

package_name = 'stella_arduino_py'

setup(
 name=package_name,
 version='0.1.0',
 packages=find_packages(exclude=['test']),
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools', 'pyserial'],
 zip_safe=True,
 maintainer='NTREX LAB',
    maintainer_email='lab@ntrex.co.kr',
 description='STELLA Arduino ROS2 Example package',
 license='Apache License 2.0',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'stella_arduino_serial = stella_arduino_py.stella_arduino_serial:main',
             'stella_arduino_node = stella_arduino_py.stella_arduino_node:main'
     ],
   },
)
