from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_assist_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azwad',
    maintainer_email='wakifrajin@gmail.com',
    description='A voice-controlled TurtleBot3 in Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speaker = voice_assist_bot.speaker:main',
            'control = voice_assist_bot.control:main',
        ],
    },
)
