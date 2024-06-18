import os.path
from subprocess import check_output

from setuptools import setup
from setuptools.command.build_py import build_py as _build_py
from wheel.bdist_wheel import bdist_wheel as _bdist_wheel

# The shared library name
LIB_NAME = 'libkin.so'


class build_py(_build_py):
    """Subclass of build_py command to also copy the compiled shared library"""

    def run(self):
        _build_py.run(self)
        out_name = os.path.join(self.build_lib, LIB_NAME)
        if not os.path.isfile(LIB_NAME):
            raise IOError('{} not found, run `make` first.'.format(LIB_NAME))
        self.copy_file(LIB_NAME, out_name)


class bdist_wheel(_bdist_wheel):
    """Subclass of bdist_wheel command to always build a platform wheel."""

    def finalize_options(self):
        _bdist_wheel.finalize_options(self)
        self.root_is_pure = False


# Read description from README
with open('README.md') as f:
    long_description = f.read()

# Try to get tag at current commit
version = check_output(['git', 'describe', '--tags']).strip().decode()
parts = version.split('-')
if parts[-1].startswith('g') and len(parts[-1]) == 8:
    # If there was no tag at current commit, find latest reachable tag
    # and combine it with git hash to form a local version (as
    # specified by PEP 440).
    version = check_output(['git', 'describe', '--abbrev=0', '--tags'])
    git_hash = check_output(['git', 'rev-parse', '--short', 'HEAD'])
    version = '{}+{}'.format(
        version.strip().decode(),
        git_hash.strip().decode(),
    )
version = version.lstrip('v')


setup(
    name='ckin',
    version=version,
    author='Cognibotics Developers',
    author_email='support@cognibotics.org',
    description='DH and LPOE inverse kinematics.',
    long_description=long_description,
    license='Other/Proprietary License',
    url='https://gitlab.cognibotics.net/cb/ckin',
    py_modules=['kin_wrapper'],
    cmdclass={
        'build_py': build_py,
        'bdist_wheel': bdist_wheel,
    },
    install_requires=['lpoe~=3.0'],
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: Other/Proprietary License',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Topic :: Software Development',
        'Operating System :: MacOS :: MacOS X',
        'Operating System :: POSIX :: Linux',
    ],
)
