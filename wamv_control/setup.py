import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wamv_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         # Instalación de archivos de launch
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.xml'))),  # por si añades XML en el futuro
        
        # Instalación de archivos de configuración (yaml, params, etc.)
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.params.yaml'))),  # opcional
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danisanch118',
    maintainer_email='dansancas@alum.us.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit = wamv_control.pure_pursuit:main',
            'control = wamv_control.control:main',
            'rrt_planner = wamv_control.rrt_planner:main'
        ],
    },
)
