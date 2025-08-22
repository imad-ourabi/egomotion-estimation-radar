# This file is part of project link.developers/new-node-hugin.
# It is copyrighted by the contributors recorded in the version control history of the file,
# available from its original location git@github.com:imad-ourabi/new-node-hugin.git.
#
# SPDX-License-Identifier: MPL-2.0

"""Packaging settings."""
from setuptools import find_packages, setup
import os

setup(
    name = 'new-node-hugin',
    version = str(os.environ.get("BUILD_VERSION")),
    description = "experimental node for hugin",
    url = '',
    author = 'imad',
    author_email = '',
    license = 'MPLv2',
    keywords = 'cli',
    packages = find_packages(exclude=['docs', 'tests*']),
    include_package_data=True,
    entry_points = {
        'console_scripts': [
            'new-node-hugin=hugin.main:main',
        ]
    },
)
