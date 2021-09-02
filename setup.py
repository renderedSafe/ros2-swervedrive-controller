from setuptools import setup

package_name = 'swervedrive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Controls swerve drive robot with /cmd_vel topic',
    license='GPL v3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swervedrive-controller = swervedrive_control.swervedrive_control_function:main',
        ],
    },
)
