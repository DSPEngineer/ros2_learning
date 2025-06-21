from setuptools import find_packages, setup

# Additional imports
import os
from glob import glob

package_name = 'mypub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py'))
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
            ## Node will use "main" function from simple_publisher.py script:
            'simplePublisher = mypub.simple_publisher:main',
        ],
    },
)
