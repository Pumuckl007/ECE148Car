from setuptools import setup, Extension, find_packages
import platform

LIBS = ['rcmpu']
# LIBRARY_DIRS = ['./rc_mpu/bin']
# INCLUDE_DIRS = ['./rc_mpu/include']
LIBRARY_DIRS = []
INCLUDE_DIRS = []
if platform.system().lower() == 'linux':
    LIBS.append('rt')

_mpu9250 = Extension("rcmpupy._mpu9250",
                     sources = ["src/_mpu9250.c"],
                     libraries = LIBS,
                     library_dirs = LIBRARY_DIRS,
                     include_dirs = INCLUDE_DIRS)

def readme():
    with open('README.rst') as f:
        return f.read()

setup(

    name="rcmpu",
    version="0.1.0",
    packages=find_packages(),
    python_requires='>=3.4',

    # extensions
    ext_modules=[_mpu9250],

    # metadata
    author = "Mauricio C. de Oliveira",
    author_email = "mauricio@ucsd.edu",

    description = "Python Library for MPU-925X on Beaglebone Black and Beaglebone Blue",
    long_description=readme(),
    license = "MIT",

    keywords= ["MPU-925X", "Beaglebone Black", "Beaglebone Blue"],

    url = "https://github.com/mcdeoliveira/rcmpupy",
    download_url = "https://github.com/mcdeoliveira/rcmpupy/archive/0.1.tar.gz",

    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Other Audience',
        'Topic :: Scientific/Engineering',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3.4',
    ],

)

