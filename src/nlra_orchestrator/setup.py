from setuptools import find_packages, setup

package_name = "nlra_orchestrator"

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
    description="Orchestrator: grounds tasks via world model, sequences skills.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "orchestrator = nlra_orchestrator.orchestrator:main",
        ],
    },
)
