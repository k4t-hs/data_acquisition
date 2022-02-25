from setuptools import setup

package_name = 'data_acquisition'

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
    maintainer='Kathrin Alba',
    maintainer_email='alka1020@h-ka.de',
    description='Extract and handle data from bag files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag2df = data_acquisition.bag2df:main',
            'imprTests = data_acquisition.imprTests:main',
            'key_input = data_acquisition.key_input:main',
        ],
    },
)
