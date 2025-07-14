from setuptools import setup

package_name = 'crosswalk_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jingyu',
    maintainer_email='your@email.com',
    description='Crosswalk detector node that stops vehicle for 5 seconds when triggered',
    license='MIT',
    entry_points={
        'console_scripts': [
            'crosswalk_detector_node = crosswalk_detector.publisher.crosswalk_detector_node:main',
        ],
    },
)
