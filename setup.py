from setuptools import find_packages, setup

package_name = 'offboard_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),  # ← resource/offboard_control 파일이 실제로 있어야 함
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minseo',
    maintainer_email='minseo@example.com',
    description='Offboard WGS84 flight controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_node = offboard_control.offboard_node:main',
            'offboard_mili = offboard_control.offboard_mili:main',
        ],
    },
)

