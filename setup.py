from setuptools import setup
import os
from glob import glob

package_name = 'mike_av_stack'
point_cloud_stacker = 'mike_av_stack/point_cloud_stacker'
traffic_manager = 'mike_av_stack/traffic_manager'
visualizations = 'mike_av_stack/visualizations'

setup(
    name=package_name,
    version='2.0.3',
    packages=[package_name, 
              point_cloud_stacker,
              traffic_manager,
              visualizations],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob('mike_av_stack/configs/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mike',
    maintainer_email='mgmike1023@aol.com',
    description='Michael\'s autonomous vehicle stack built on top of carla ros bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_cloud_stacker = mike_av_stack.point_cloud_stacker.point_cloud_stacker:main',
            'visualizations = mike_av_stack.visualizations.visualizations:main',
        ],
    },
)
