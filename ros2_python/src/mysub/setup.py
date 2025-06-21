from setuptools import find_packages, setup

import os
# This file is part of the mysub package.


package_name = 'mysub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/mysub_launch_file.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jose',
    maintainer_email='jose.pagan@terex.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            ## Node will use "main" function from simple_subscriber.py script:
            'simpleSubscriber = mysub.mysubscriber:main',
        ],
    },
)
