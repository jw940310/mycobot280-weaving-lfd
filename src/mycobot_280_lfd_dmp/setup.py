from glob import glob

from setuptools import setup

package_name = 'mycobot_280_lfd_dmp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwkim',
    maintainer_email='jw940310@gmail.com',
    description='LfD 위빙 스킬 DMP 계층 (movement_primitives 연동)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'check_dmp_env = mycobot_280_lfd_dmp.env_check:main',
            'dmp_reproduce = mycobot_280_lfd_dmp.reproduce_cli:main',
        ],
    },
)
