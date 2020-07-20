# Robowflex CI Scripts

Robowflex CI is done through [Github Actions](https://github.com/KavrakiLab/robowflex/actions).
CI does the following:
- Checks formatting of code base with `clang-format` (see `check-format.sh` and `.github/workflows/linting.yml`).
- Builds and deploys documentation to Github Pages (see `documentation.sh` and `.github/workflows/documentation.yml`)
- Builds code to check compilation (see `.github/workflows/build.yml`).
