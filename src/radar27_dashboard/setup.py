from glob import glob
from setuptools import find_packages, setup

setup(
    name='radar27_dashboard', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/radar27_dashboard']),
        ('share/radar27_dashboard', ['package.xml', 'README.md']),
        ('share/radar27_dashboard/web', glob('web/*')),
        ('share/radar27_dashboard/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Radar27', maintainer_email='you@example.com', license='MIT',
    description='Read-only Radar27 telemetry dashboard',
    entry_points={'console_scripts': [
        'dashboard_node = radar27_dashboard.dashboard_node:main',
    ]},
)
