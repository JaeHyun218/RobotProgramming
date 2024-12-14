from setuptools import setup

package_name = 'turtlebot_run'

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
    maintainer='jaehyun',
    maintainer_email='2019100726@khu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'yolo = turtlebot_run.yolo:main',
        	'mapping = turtlebot_run.map:main',
        	'navigation = turtlebot_run.navi:main',
        ],
    },
)
