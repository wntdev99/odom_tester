from setuptools import find_packages, setup

package_name = 'odom_compare'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='james@example.com',
    description='방법 ① — swerve vs EKF 오도메트리 드리프트 비교.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_compare_node = odom_compare.odom_compare_node:main',
        ],
    },
)
