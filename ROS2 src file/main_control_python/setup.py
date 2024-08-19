from setuptools import find_packages, setup

package_name = 'main_control_python'

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
            "main_program = main_control_python.main_control_py:main",
	        "image_rec = main_control_python.image_receive:main",
        ],
    },
)
