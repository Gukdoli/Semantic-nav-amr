from setuptools import find_packages, setup

package_name = "semantic_map"

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
    description="Semantic object database, lookup service and RViz markers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "semantic_map_node = semantic_map.semantic_map_node:main",
        ],
    },
)
