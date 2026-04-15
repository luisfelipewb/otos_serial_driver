from glob import glob
import os

from setuptools import setup


package_name = "otos_serial_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Luis",
    maintainer_email="luis@example.com",
    description="ROS 2 serial bridge driver for SparkFun Qwiic OTOS via Arduino.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "otos_serial_node = otos_serial_driver.otos_serial_node:main",
        ],
    },
)
