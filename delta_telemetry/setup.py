from setuptools import find_packages, setup

package_name = 'delta_telemetry'

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
    maintainer='rikisu',
    maintainer_email='likhiacharya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plotter3d = delta_telemetry.plotter3d:main',
            'tf_error_plotter = delta_telemetry.tf_error_plotter:main',
            'joint_state_fk_broadcaster = delta_telemetry.joint_state_fk_broadcaster:main',
        ],
    },
)
