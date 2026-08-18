import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nlra_web_ui"

web_share = os.path.join("share", package_name, "web")


def _web_data_files():
    """Install web/ preserving its subdirectory structure (css, js, models)."""
    entries = []
    for root, dirs, files in os.walk("web"):
        if not files:
            continue
        rel = os.path.relpath(root, "web")
        dest = web_share if rel == "." else os.path.join(web_share, rel)
        entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ] + _web_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="christian",
    maintainer_email="christian@example.com",
    description="Web UI: manual control, NL chat, diagnostics for nlra robot arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_server = nlra_web_ui.web_server:main",
        ],
    },
)
