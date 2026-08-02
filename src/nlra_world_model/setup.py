from setuptools import find_packages, setup

package_name = "nlra_world_model"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="christian",
    maintainer_email="christian@example.com",
    description="World model: ground-truth object poses + query services.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_model = nlra_world_model.world_model:main",
        ],
    },
)
