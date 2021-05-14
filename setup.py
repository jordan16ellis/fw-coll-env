from pathlib import Path
from setuptools import setup

# Available at setup time due to pyproject.toml
from pybind11.setup_helpers import Pybind11Extension, build_ext

__version__ = "0.0.1"

ext_modules = [
    Pybind11Extension(
        "fw_coll_env_c",
        ["src/main.cpp", "src/Utils.cpp", "src/FwAvailActions.cpp",
         "src/Uhat.cpp", "src/FwCollisionEnv.cpp",
         "src/BarrierGammaTurn.cpp", "src/BarrierGammaStraight.cpp",
         "src/FwActionIndex.cpp"],
        include_dirs=[Path(__file__).parent / 'include'],
        # Example: passing in the version to the compiled code
        define_macros=[('VERSION_INFO', __version__)],
        ),
]

setup(
    name="fw-coll-env",
    version=__version__,
    author="Eric Squires",
    author_email="eric.squires@gmail.com",
    description="fw-coll-env",
    long_description="",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    install_requires=[
        "gym",
        "pytest",
        "flake8",
        "pylint",
        "pydocstyle",
        "mypy",
    ],
)
