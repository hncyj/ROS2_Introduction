from setuptools import find_packages, setup

package_name = 'py_demo_pkg'

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
    maintainer='chenyinjie',
    maintainer_email='chenyinjie666@foxmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_node = py_demo_pkg.py_node:main',
            'person_node = py_demo_pkg.person_node:main',
            'writer_node = py_demo_pkg.writer_node:main',
            'thread_node = py_demo_pkg.learn_thread:main'
        ],
    },
)
