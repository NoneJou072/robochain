import os
from glob import glob
from setuptools import setup

package_name = 'gpt_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name:['prompts/*', 'agents/*', 'commons/*', 'examples/*']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhouhr',
    maintainer_email='jou072@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gpt_client = gpt_client.examples.client_retrieval_codellama:main",
        ],
    },
)
