from setuptools import find_packages, setup

package_name = "zero_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/bringup_fake.launch.py",
                "launch/control_chain.launch.py",
                "launch/gazebo_control.launch.py",
                "launch/gazebo_mapping.launch.py",
                "launch/gazebo_navigation.launch.py",
            ],
        ),
        (f"share/{package_name}/config", ["config/v1_sim.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cnxwzy",
    maintainer_email="cnxwzy@gmail.com",
    description="Launch orchestration for the Zero unmanned surface vessel.",
    license="CC-BY-NC-SA-4.0",
    tests_require=["pytest"],
)
