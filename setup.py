from setuptools import setup

package_name = 'suas24_localization_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ascend',
    maintainer_email='dronyboi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'suas24_localization_visualization = suas24_localization_visualization.suas24_localization_visualization'
        ],
    },
)
