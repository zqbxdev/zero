from setuptools import find_packages, setup

package_name = "zero_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cnxwzy",
    maintainer_email="cnxwzy@gmail.com",
    description="Control conversion nodes for the Zero unmanned surface vessel.",
    license="CC-BY-NC-SA-4.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist_to_motor_command = zero_control.twist_to_motor_command:main",
        ],
    },
)
