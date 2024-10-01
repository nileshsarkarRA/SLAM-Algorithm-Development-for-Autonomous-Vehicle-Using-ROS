from setuptools import setup, find_packages

package_name = 'autonomous_vehicle'

setup(
    name=package_name,
    version='0.1.0',  # Updated version
    packages=find_packages(),  # Automatically find all packages in the directory
    install_requires=[
        'setuptools',
        'gpiozero',  # Include gpiozero as a runtime dependency
        # 'opencv-python',  # Uncomment if using OpenCV
    ],
    zip_safe=True,
    maintainer='R.U.N.S',
    maintainer_email='eng23ra0065@dsu.edu.in',
    description='An autonomous vehicle project using Raspberry Pi and ROS 2.',
    license='MIT',
    tests_require=['pytest'],  # Testing requirements
    entry_points={  # Console scripts for running nodes
        'console_scripts': [
            'motor_control = autonomous_vehicle.motor_control:main',
            'lidar_node = autonomous_vehicle.lidar_node:main',
            'camera_node = autonomous_vehicle.camera_node:main',
        ],
    },
    extras_require={  # Optional dependencies
        'opencv': ['opencv-python'],  # Optional: OpenCV support
    },
)

