from setuptools import setup

package_name = 'mycobot_280_lfd_data'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/samples', ['samples/line_sample.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwkim',
    maintainer_email='jw940310@gmail.com',
    description='LfD 궤적 CSV 스키마 v1 참조 구현 (load/save/validate)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'validate_trajectory = mycobot_280_lfd_data.schema:main',
            'generate_trajectories = mycobot_280_lfd_data.synth:main',
            'trajectory_tf_publisher = mycobot_280_lfd_data.tf_publisher:main',
        ],
    },
)
