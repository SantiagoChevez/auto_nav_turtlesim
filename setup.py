from setuptools import find_packages, setup

package_name = 'auto_nav_turtlesim'

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
    maintainer='santiago',
    maintainer_email='santiago@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "nav_controller= auto_nav_turtlesim.nav_controller:main",
        "web= auto_nav_turtlesim.web:main"
        ],
    },
)
