from setuptools import find_packages, setup

package_name = 'grupo03_robot_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fernando',
    maintainer_email='fernando.aramayo.s@ucb.edu.bo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fk = grupo03_robot_kinematics.fk_node:main',
            'ik = grupo03_robot_kinematics.ik_node:main',
        ],
    },
)
