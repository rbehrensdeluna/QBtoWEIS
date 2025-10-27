#!/usr/bin/env bash
# ===============================================================
#  QBtoWEIS Installer for Linux / macOS (Shell version)
#  -----------------------------------------------------------
#  Assumes:
#    - Conda (Miniforge/Miniconda/Anaconda) installed & on PATH
#    - Git installed & on PATH
#    - No admin privileges required
#
#  What it does:
#    1) Clone or update QBtoWEIS (smart detection of repo location)
#    2) Create or update the conda env from environment.yml
#    3) Install petsc4py, mpi4py, pyoptsparse
#    4) Install QBtoWEIS in editable mode (-e)
# ===============================================================

set -euo pipefail

# --- Config ---
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_OWNER="rbehrensdeluna"
REPO_NAME="QBtoWEIS"
REPO_URL="https://github.com/${REPO_OWNER}/${REPO_NAME}.git"

# Detect repo root if the script is inside the repo (root or /install)
if [ -d "${SCRIPT_DIR}/.git" ]; then
  REPO_DIR="${SCRIPT_DIR}"
elif [ -d "${SCRIPT_DIR}/../.git" ]; then
  REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
else
  # Fall back: clone next to this script
  REPO_DIR="${SCRIPT_DIR}/${REPO_NAME}"
fi

ENV_NAME="qbweis-env"

echo
echo "==============================================================="
echo "  QBtoWEIS Installer (Linux/macOS)"
echo "  Target folder: ${REPO_DIR}"
echo "  Conda env:     ${ENV_NAME}"
echo "==============================================================="
echo

# --- Check dependencies ---
command -v conda >/dev/null 2>&1 || { echo "[ERROR] conda not found. Please install Miniforge or Conda."; exit 1; }
command -v git   >/dev/null 2>&1 || { echo "[ERROR] git not found. Please install Git."; exit 1; }

# --- Clone or update repository ---
if [ -d "${REPO_DIR}/.git" ]; then
  echo "==> Repository found at '${REPO_DIR}'. Pulling latest changes..."
  (cd "${REPO_DIR}" && git pull --rebase --autostash)
elif [ -d "${REPO_DIR}" ]; then
  echo "[ERROR] Folder '${REPO_DIR}' exists but is not a git repository."
  exit 1
else
  echo "==> Cloning repository to '${REPO_DIR}' ..."
  git clone "${REPO_URL}" "${REPO_DIR}"
fi

echo

# --- Conda channel setup (best effort) ---
conda config --add channels conda-forge || true

# --- Create or update environment from environment.yml ---
if conda env list | grep -E "^\s*${ENV_NAME}\s" >/dev/null 2>&1; then
  echo "==> Updating environment from environment.yml ..."
  conda env update --name "${ENV_NAME}" -f "${REPO_DIR}/environment.yml" --prune || echo "[WARN] Update had issues; continuing..."
else
  echo "==> Creating environment from environment.yml ..."
  conda env create --name "${ENV_NAME}" -f "${REPO_DIR}/environment.yml"
fi

echo
echo "==> Installing Linux/macOS-specific dependencies ..."
conda install -n "${ENV_NAME}" -y petsc4py mpi4py pyoptsparse

echo
echo "==> Installing QBtoWEIS (-e) ..."
conda run -n "${ENV_NAME}" python -m pip install -e "${REPO_DIR}"

echo
echo "==============================================================="
echo "  QBtoWEIS installation complete!"
echo
echo "  Next steps:"
echo "    1) Activate the environment:"
echo "         conda activate ${ENV_NAME}"
echo
echo "    2) Download QBladeCE (v2.0.9+):"
echo "         https://qblade.org/downloads/"
echo
echo "    3) Set your QBlade shared object path (e.g., libQBladeEE_2.0.9.so.1.0) in:"
echo "         QBtoWEIS/weis/inputs/modeling_schema.yaml  at 'path2qb_dll'"
echo
echo "    4) Try the example:"
echo "         cd ${REPO_DIR}/qb_examples/00_run_test"
echo "         python weis_driver_oc3.py"
echo "==============================================================="
echo
