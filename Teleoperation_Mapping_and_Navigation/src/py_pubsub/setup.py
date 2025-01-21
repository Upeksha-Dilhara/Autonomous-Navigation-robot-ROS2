from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='dinoj',
    maintainer_email='dinoj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
            'cmdVel_to_pwm_node = py_pubsub.motor_pwm_test:main',
            'basic = py_pubsub.basic:main',
            'diff_inverse = py_pubsub.diff_inverse:main',
            'imu_sensor = py_pubsub.imu:main',
            'kalman_filter = py_pubsub.kalman_filter:main',
            'velocity_smoother = py_pubsub.velocity_smoother:main'
        ],
    },
)
