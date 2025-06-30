from setuptools import find_packages, setup

package_name = 'nebupkg'

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
    maintainer='julian',
    maintainer_email='julian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	'emociones = nebupkg.emociones:main',
	'model_node = nebupkg.model_node:main',
	'voz_node = nebupkg.voz_node:main',
	'todo = nebupkg.todo:main',
	'rostros = nebupkg.rostros:main',
	'arduino = nebupkg.arduino:main',
    'nodotts = nebupkg.nodotts:main',
        ],
    },
)
