from setuptools import setup

package_name = "asrs_pickplace"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="Pick-and-place application for the ASRS robotic arm",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pick_place_node = asrs_pickplace.pick_place_node:main",
        ],
    },
)
