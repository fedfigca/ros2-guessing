from setuptools import setup

package_name = 'guessing_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede',
    maintainer_email='fede@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guessing_dealer = guessing_game.guessing_dealer:main',
            'guessing_player = guessing_game.guessing_player:main'
        ],
    },
)
