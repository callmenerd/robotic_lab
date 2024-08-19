from setuptools import find_packages, setup

package_name = 'move_command'

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
            "goto = move_command.go_to:main",
            "move_to = move_command.move_plan:main",
            "move_plan = move_command.move_planning:main",
        ],
    },
)
