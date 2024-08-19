from setuptools import find_packages, setup

package_name = 'image_process'

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
    maintainer_email='blakasutha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_publisher = image_process.image_pub:main',
            'img_subscriber = image_process.image_sub:main',
            'img_subscriber_process = image_process.image_sub_process:main',
            'img_subscriber_process_coral = image_process.image_sub_coral:main',
            'img_sub_colorseg = image_process.image_sub_seg:main',
            'line_follow = image_process.line_follower:main',
        ],
    },
)
