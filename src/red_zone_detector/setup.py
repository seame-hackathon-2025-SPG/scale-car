from setuptools import setup

package_name = 'red_zone_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='진규',
    maintainer_email='jin@example.com',
    description='Red zone detection node that slows down when red area is detected.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'red_zone_detector_node = red_zone_detector.red_zone_detector_node:main'
        ],
    },
)
