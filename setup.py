import os
from glob import glob
from setuptools import setup

package_name = "ros2_diag2influxdb"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        # ("share/" + package_name + "/config", ["config/antares.yaml"])
    ],
    install_requires=["setuptools", "influxdb_client"],
    zip_safe=True,
    maintainer="tfoldi",
    maintainer_email="tfoldi@xsi.hu",
    description="Send data from diagnostics topic to InfluxDB",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "diag2influxdb_node = ros2_diag2influxdb.diag2influxdb_node:main"
        ],
    },
)
