from setuptools import find_packages, setup

setup(
    name="onero_a1_common", version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/onero_a1_common"]),
        ("share/onero_a1_common", ["package.xml", "LICENSE", "README.md"]),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="Onero Robotics", maintainer_email="dev@onerobot.com",
    description="Shared MoveIt ROS clients for Onero A1", license="Apache-2.0",
)
