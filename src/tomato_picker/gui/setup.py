from glob import glob
from setuptools import find_packages, setup

package_name = "tomato_picker_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Kaede Rei",
    maintainer_email="kaerei@foxmail.com",
    description="Optional wrist RGB-D manual target selection GUI for Tomato-Picker",
    license="MIT",
    entry_points={
        "console_scripts": [
            "wrist_target_gui = tomato_picker_gui.app:main",
        ],
    },
)
