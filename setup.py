from setuptools import find_packages, setup

package_name = "simple_mover"

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
    maintainer="Project Team",
    maintainer_email="team@example.com",
    description="Global coordinate waypoint control for a differential-drive robot in ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "position_tracker = simple_mover.position_tracker:main",
            "global_coordinate = simple_mover.global_coordinate:main",
            "inverse_kin = simple_mover.inverse_kin:main",
        ],
    },
)
