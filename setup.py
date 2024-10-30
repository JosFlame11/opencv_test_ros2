from setuptools import find_packages, setup

package_name = 'opencv_test'

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
    maintainer='josflame11',
    maintainer_email='jolapa20@gmail.com',
    description='This package is meant to test image conversion with cv_bridge',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_cv_bridge = opencv_test.cv_bridge_test:main',
            'show_compressed_image = opencv_test.compressed_img_2_opencv:main',
            'test_hough = opencv_test.hough_tf:main',
        ],
    },
)
