from setuptools import find_packages, setup

package_name = 'dishwashing_robot'

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
    maintainer='k-chan-l',
    maintainer_email='kings0625@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main_node = dishwashing_robot.main_node:main',
            'plate_test = dishwashing_robot.nodes.plate_test_node:main',
        ],
    },
)
