from glob import glob
from setuptools import find_packages, setup


package_name = "vlm_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["pytest.ini"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + package_name + "/config",
            glob("config/*.yaml") + glob("config/*.rviz"),
        ),
        (
            "share/" + package_name + "/scripts",
            glob("scripts/*.sh") + glob("scripts/*.py"),
        ),
        (
            "share/" + package_name + "/behavior_trees",
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@example.com",
    description="VLM grounded RGB-D navigation for ROS 2 and Nav2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fastlio_odom_adapter = vlm_nav.fastlio_odom_adapter:main",
            "vlm_navigator = vlm_nav.vlm_navigator:main",
        ]
    },
)
