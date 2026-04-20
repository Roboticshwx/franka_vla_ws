from setuptools import find_packages, setup

package_name = 'vla_inference'

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
    maintainer='hwx',
    maintainer_email='2557095375@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vla_talker = vla_inference.publisher_member_function:main',
            'vla_listener = vla_inference.subscriber_member_function:main',
            'vla_fake_publisher = vla_inference.vla_inference_action:main',
        ],
    },
)




