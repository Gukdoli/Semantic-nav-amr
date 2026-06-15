from setuptools import find_packages, setup

package_name = "object_detector"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="tjddnwkdiy@gmail.com",
    description="Open-vocabulary object detection + 2D->3D projection.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detector_node = object_detector.object_detector_node:main",
        ],
    },
)
