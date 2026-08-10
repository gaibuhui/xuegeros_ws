from glob import glob
import os
import sys
import unittest

from setuptools import Command, find_packages, setup


package_name = "xuegecar_obstacle_avoidance"


class TestCommand(Command):
    """Run the package's standard-library unit tests for colcon."""

    user_options = []

    def initialize_options(self):
        pass

    def finalize_options(self):
        pass

    def run(self):
        sys.path.insert(0, os.path.dirname(__file__))
        suite = unittest.defaultTestLoader.discover("test", top_level_dir=".")
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        if not result.wasSuccessful():
            raise SystemExit(1)


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xuegeros",
    maintainer_email="xuegeros@todo.todo",
    description="Conservative LaserScan-based autonomous obstacle avoidance for xuegecar.",
    license="MIT",
    cmdclass={"test": TestCommand},
    entry_points={
        "console_scripts": [
            "obstacle_avoidance = xuegecar_obstacle_avoidance.obstacle_avoidance:main",
        ],
    },
)
