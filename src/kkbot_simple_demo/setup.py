from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'kkbot_simple_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quicktron',
    maintainer_email='niezhongbiao@flashhold.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "example_nav_to_pose = kkbot_simple_demo.example_nav_to_pose:main"
        ],
    },
)
