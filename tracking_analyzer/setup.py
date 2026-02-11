from setuptools import find_packages, setup

package_name = 'tracking_analyzer'

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
    maintainer='rushil',
    maintainer_email='kapoorrushil11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'analyzer_node = tracking_analyzer.analyzer_node:main',
            'instant_error_node = tracking_analyzer.instant_error_node:main',
            'xy_tracking_node = tracking_analyzer.xy_tracking_node:main',
        ],
    },
)
