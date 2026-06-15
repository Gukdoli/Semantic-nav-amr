from setuptools import find_packages, setup

package_name = "language_goal"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # google-genai powers the optional LLM command parser (M5); the node falls
    # back to keyword matching when it / the API key is absent.
    install_requires=["setuptools", "google-genai"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="tjddnwkdiy@gmail.com",
    description="Natural-language command parsing + Nav2 goal generation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "goal_commander_node = language_goal.goal_commander_node:main",
            "web_command_node = language_goal.web_command_node:main",
        ],
    },
)
