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
    description='Extract and handle data from bag files',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag2df = data_acquisition.bag2df:main',
            'imprTests = data_acquisition.imprTests:main',
            'topics2df = data_acquisition.topics2df:main',
            'topics2df_v2 = data_acquisition.topics2df_v2:main',
        ],
    },
)
