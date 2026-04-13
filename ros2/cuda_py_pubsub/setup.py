from setuptools import setup

package_name = "cuda_py_pubsub"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dora-benchmark",
    maintainer_email="noreply@example.com",
    description="Python ROS 2 CUDA IPC GPU-to-GPU latency benchmark.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "cuda_talker = cuda_py_pubsub.cuda_talker:main",
            "cuda_listener = cuda_py_pubsub.cuda_listener:main",
        ],
    },
)
