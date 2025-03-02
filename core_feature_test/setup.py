from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'core_feature_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'srv'), glob('core/srv/*.srv')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Test package for features implementation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_factory = core_feature_test.test_factory:main'
            # 'test_service_factory = core_feature_test.test_services:main',
        ],
    },
)
