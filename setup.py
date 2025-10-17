from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'llm_action_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cstar',
    maintainer_email='kuldeeplakhansons@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'e_llm_ros = llm_action_generator.llm_ros:main',
            'e_llm_client_with_vlm = llm_action_generator.llm_client_with_vlm:main',
            'e_llm_sub = llm_action_generator.llm_sub:main',
            'e_llm_client = llm_action_generator.llm_client:main',

        ],
    },
)
