from setuptools import setup

package_name = 'send_images'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ri',
    maintainer_email='ri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        f"sending_img = {package_name}.sending_img:main",
        f"camera_img_sending = {package_name}.camera_image_publisher:main",
        ],
    },
)
