import os
import toml
from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(
    os.path.join(EXTENSION_PATH, "config", "extension.toml")
)

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "pyyaml",
]

setup(
    name="quadrupedrobot-simulator",
    install_requires=INSTALL_REQUIRES,
    # additional metadata about your package
    author=EXTENSION_TOML_DATA["package"]["author"],
    author_email=EXTENSION_TOML_DATA["package"]["author_email"],
    maintainer=EXTENSION_TOML_DATA["package"]["author"],
    maintainer_email=EXTENSION_TOML_DATA["package"]["author_email"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    url=EXTENSION_TOML_DATA["package"]["repository"],
    python_requires=">=3.7.*",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Linux",
    ],
    zip_safe=False,
)
