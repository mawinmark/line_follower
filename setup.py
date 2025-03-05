from setuptools import setup

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mawin',
    maintainer_email='mawin.mrk@example.com',
    description='Line follower robot using OpenCV in ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = line_follower.line_follower:main',
        ],
    },
)
