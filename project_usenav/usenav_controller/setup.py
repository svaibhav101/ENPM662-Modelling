from setuptools import find_packages, setup

package_name = 'usenav_controller'

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
    maintainer='vibi',
    maintainer_email='vaibhavshende1903@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_controller=usenav_controller.simple_controller:main',
            'feedback_controller=usenav_controller.feedback_controller:main'
            

        ],
    },
)
