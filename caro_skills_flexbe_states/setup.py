#!/usr/bin/env python
# # Copyright 2026 Carologistics
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
from glob import glob

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = "caro_skills_flexbe_states"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/tests", glob("tests/*.test")),
        ("share/" + PACKAGE_NAME + "/launch", glob("tests/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@TODO.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "example_action_state = caro_skills_flexbe_states.example_action_state",
            "example_state = caro_skills_flexbe_states.example_state",
        ],
    },
)
