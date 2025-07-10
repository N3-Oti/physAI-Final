from setuptools import setup, find_packages
import os

package_name = 'physics_ai'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PhysAI Team',
    maintainer_email='dev@physai.example.com',
    description='Natural Language Robot Control with Gemini AI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gemini_json_publisher = physics_ai.gemini_json_publisher:main',
            'json_command_interpreter = physics_ai.json_command_interpreter:main',
        ],
    },
)