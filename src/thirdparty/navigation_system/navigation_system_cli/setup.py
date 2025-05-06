from setuptools import find_packages
from setuptools import setup

package_name = 'navigation_system_cli'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Juan Carlos Manzanares Serrano',
    author_email='jc.manzanares.serrano@gmail.com',
    maintainer='Juan Carlos Manzanares Serrano',
    maintainer_email='jc.manzanares.serrano@gmail.com',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Command line tools for Navigation System.',
    long_description="""\
The package provides the navigation_system command as
a plugin for Navigation System.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'robocup = navigation_system_cli.command.robocup:RobocupCommand',
        ],
        'ros2cli.extension_point': [
            'robocup.verb = navigation_system_cli.verb:VerbExtension',
        ],
        'robocup.verb': [
            'set_map = navigation_system_cli.verb.set_map:SetMapVerb',
            'save_map = navigation_system_cli.verb.save_map:SaveMapVerb',
            'set_pose = navigation_system_cli.verb.set_pose:SetPoseVerb',
            'set_mode = navigation_system_cli.verb.set_mode:SetModeVerb',
        ],
    }
)
