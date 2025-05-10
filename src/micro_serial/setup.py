from setuptools import find_packages, setup

package_name = 'micro_serial'

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
    maintainer='blakasutha',
    maintainer_email='blakasutha@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "STM_interface = micro_serial.stm_serial:main",
	        "PICO_interface = micro_serial.pico_serial:main",
            "DISP_interface = micro_serial.display_data:main",
            "FUSION_SENSOR = micro_serial.sensor_fusion:main",
        ],
    },
)
