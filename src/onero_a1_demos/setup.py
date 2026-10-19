from setuptools import find_packages, setup

setup(
    name="onero_a1_demos", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/onero_a1_demos"]),
        ("share/onero_a1_demos", ["package.xml", "LICENSE"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Onero Robotics", maintainer_email="dev@onerobot.com",
    description="MoveIt examples for the Onero A1 arm", license="Apache-2.0",
    entry_points={"console_scripts": [
        "run = onero_a1_demos.runner:main",
        "compare_trials = onero_a1_demos.compare_trials:main",
        "sdk_compare = onero_a1_demos.sdk_compare:main",
    ]},
)
