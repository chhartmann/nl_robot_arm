from setuptools import find_packages, setup

package_name = "nlra_skills"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/skills.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="christian",
    maintainer_email="christian@example.com",
    description="Skill layer: ROS 2 action servers wrapping arm/gripper capabilities.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "skill_servers = nlra_skills.skill_servers:main",
        ],
    },
)
