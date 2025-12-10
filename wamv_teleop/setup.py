from setuptools import find_packages, setup

package_name = 'wamv_teleop'

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
    maintainer='Your Name', # Cámbialo si quieres
    maintainer_email='your.email@example.com', # Cámbialo si quieres
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # AQUÍ ES DONDE SE DECLARA EL EJECUTABLE
            'keydrive = wamv_teleop.keydrive_node:main',
        ],
    },
)
