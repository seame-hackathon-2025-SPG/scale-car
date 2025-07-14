from setuptools import setup

package_name = 'stop_line_detector'

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
    description='Stop line detector node that stops vehicle for 5 seconds when triggered',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stop_line_detector_node = stop_line_detector.publisher.stop_line_detector_node:main',
        ],
    },
)
