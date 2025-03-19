from setuptools import find_packages, setup

package_name = 'py_demo_tf'

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
            'static_tf_broadcaster=py_demo_tf.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster=py_demo_tf.dynamic_tf_broadcaster:main',
            'tf_listener=py_demo_tf.tf_listener:main'
        ],
    },
)
