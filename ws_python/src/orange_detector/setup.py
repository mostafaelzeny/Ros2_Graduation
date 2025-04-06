from setuptools import setup

package_name = 'orange_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mostafa',
    maintainer_email='mostafaelzeny27@gmail.com',
    description='Orange detection and servo control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_orange = orange_detector.detect_orange:main',
            'control_servo = orange_detector.control_servo:main',
        ],
    },
)

