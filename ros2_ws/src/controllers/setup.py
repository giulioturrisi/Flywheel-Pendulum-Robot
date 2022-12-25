from setuptools import setup

package_name = 'controllers'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['run_lqr = controllers.run_lqr:main',
                            'run_adaptive_lqr = controllers.run_adaptive_lqr:main',
                            'run_ilqr = controllers.run_ilqr:main',
                            'run_feedback_lin = controllers.run_feedback_lin:main',
                            'run_acados_nmpc = controllers.run_acados_nmpc:main'
        ],
    },
)
