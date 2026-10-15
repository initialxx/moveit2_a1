#!/usr/bin/env bash
set -eo pipefail
sdk_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv --system-site-packages "${sdk_workspace}/.venv-sdk"
"${sdk_workspace}/.venv-sdk/bin/python" -m pip install -r "${sdk_workspace}/tools/requirements-sdk.txt"
"${sdk_workspace}/.venv-sdk/bin/python" -c 'import pinocchio; print("SDK dynamics ready:", pinocchio.__version__)'
