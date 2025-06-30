from setuptools import find_packages, setup

package_name = 'github'

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
    maintainer='geovani',
    maintainer_email='geovani@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'git = github.git:main'
            'wakeword = github.wakeword:main'
            'emociones = github.emociones:main',
	        'model_node = github.model_node:main',
	        'voz_node = github.voz_node:main',
	        'todo = github.todo:main',
	        'rostros = github.rostros:main',
	        'arduino = github.arduino:main',
            'nodotts = github.nodotts:main',
        ],
    },
)
