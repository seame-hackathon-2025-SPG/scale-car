from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('integration/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spg1234',
    maintainer_email='spg1234@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
