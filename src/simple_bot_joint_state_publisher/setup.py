from setuptools import find_packages, setup

package_name = 'simple_bot_joint_state_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiktorbajor1@gmail.com',
    maintainer_email='wiktorbajor1@gmail.com',
    description='Simple one joint joint_states publisher.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_bot_joint_state_publisher = simple_bot_joint_state_publisher.simple_bot_joint_state_publisher:main'
        ],
    },
)
