#!/bin/bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_AFC_EXTRAS_DIR="${REPO_ROOT}/AFC/extras"
TARGET_AFC_REPO="${HOME}/AFC-Klipper-Add-On"
TARGET_AFC_EXTRAS_DIR="${TARGET_AFC_REPO}/extras"

usage() {
    echo "Usage: $0 [-t <afc_addon_path>]"
    echo "  -t  Path to AFC-Klipper-Add-On repository (default: ~/AFC-Klipper-Add-On)"
}

while getopts ":t:h" opt; do
    case "${opt}" in
        t)
            TARGET_AFC_REPO="${OPTARG}"
            TARGET_AFC_EXTRAS_DIR="${TARGET_AFC_REPO}/extras"
            ;;
        h)
            usage
            exit 0
            ;;
        :)
            echo "Error: Option -${OPTARG} requires an argument."
            usage
            exit 1
            ;;
        \?)
            echo "Error: Invalid option -${OPTARG}"
            usage
            exit 1
            ;;
    esac
done

if [ ! -d "${SOURCE_AFC_EXTRAS_DIR}" ]; then
    echo "[ERROR] Source AFC extras directory not found: ${SOURCE_AFC_EXTRAS_DIR}"
    exit 1
fi

if [ ! -d "${TARGET_AFC_EXTRAS_DIR}" ]; then
    echo "[ERROR] Target AFC extras directory not found: ${TARGET_AFC_EXTRAS_DIR}"
    echo "        Install AFC-Klipper-Add-On first, or pass -t with the correct path."
    exit 1
fi

echo "Copying AFC extras from ${SOURCE_AFC_EXTRAS_DIR} to ${TARGET_AFC_EXTRAS_DIR}..."
cp -v "${SOURCE_AFC_EXTRAS_DIR}"/* "${TARGET_AFC_EXTRAS_DIR}/"

echo "Copying OpenAMS AFC integration files..."
cp -v "${REPO_ROOT}/AFC_OpenAMS.py" "${TARGET_AFC_EXTRAS_DIR}/"
cp -v "${REPO_ROOT}/openams_integration.py" "${TARGET_AFC_EXTRAS_DIR}/"

echo "[OK] Post-install copy complete."
