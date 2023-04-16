from setuptools import setup

package_name = 'pc_det'

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
    maintainer='wangqiankai',
    maintainer_email='kkwang@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker=pc_det.pub_bin:main',
            'listener=pc_det.bin2pcd:main',
            'pcl_client=pc_det.pcl_client:main',
            'infer_node=pc_det.infer_node:main'
        ],
    },
)
