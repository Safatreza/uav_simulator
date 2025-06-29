from setuptools import setup, find_packages

setup(
    name='uav_simulator',
    version='0.1.0',
    description='Modular UAV Simulator with high-fidelity physics and control',
    author='Your Name',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'numpy', 'matplotlib', 'PyYAML', 'scipy', 'streamlit', 'filterpy'
    ],
    entry_points={
        'console_scripts': [
            'uavsim-run = main:main',
            'uavsim-export = main:export_logs',
        ],
    },
    include_package_data=True,
    python_requires='>=3.8',
) 