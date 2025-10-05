from setuptools import find_packages, setup

package_name = 'tactile'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',  
        'numpy',
        'qrcode[pil]',   
    ],
    zip_safe=True,
    maintainer='yolandazhu',
    maintainer_email='xz3013@columbia.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'tactile_viewer = tactile.tactile_viewer:main',
            'tactile_sensor = tactile.tactile_sensor:main',
            'test_viewer = tactile.test_viewer:main',
            'test_sensor = tactile.test_sensor:main',
            'ffmpeg_v4l2_publisher = tactile.ffmpeg_v4l2_publisher:main',
            'ffmpeg_v4l2_subscriber = tactile.ffmpeg_v4l2_subscriber:main',
            'combined_subscriber = tactile.combined_subscriber:main',  # NEW ENTRY
            'image_subscriber = tactile.image_subscriber:main',
            'image_local_time = tactile.image_local_time:main',
            'qr = tactile.qr:main',  # NEW ENTRY for the QR code generator node
            'episode_marker_node = tactile.episode_marker_node:main',
            'tactile_subscriber = tactile.tactile_subscriber:main',
            'ros_subscriber_child = tactile.ros_subscriber_child:main',


        ],
    },
)
