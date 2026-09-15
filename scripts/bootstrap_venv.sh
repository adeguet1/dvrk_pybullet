#!/usr/bin/env bash
# Bootstrap the dVRK workspace Python environment used by dvrk_pybullet.
#
# The venv intentionally inherits ROS packages installed for the system Python.
# A local colcon wrapper prevents bare `colcon build` from silently invoking
# /usr/bin/python3 and generating console scripts that cannot import PyBullet.

set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "$script_path")"

find_workspace_root() {
    local cursor="$script_dir"
    while [[ "$cursor" != "/" ]]; do
        case "$(basename "$cursor")" in
            src|install)
                dirname "$cursor"
                return 0
                ;;
        esac
        cursor="$(dirname "$cursor")"
    done
    return 1
}

usage() {
    echo "usage: $0 [--python PATH]"
}

bootstrap_python="/usr/bin/python3"

while (($#)); do
    case "$1" in
        --python)
            if (($# < 2)); then
                usage >&2
                exit 2
            fi
            bootstrap_python="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

workspace_root="$(find_workspace_root || true)"
if [[ -z "$workspace_root" || ! -d "$workspace_root" ]]; then
    echo "error: could not determine the colcon workspace root from $script_path" >&2
    exit 2
fi
workspace_root="$(readlink -f "$workspace_root")"

if [[ ! -x "$bootstrap_python" ]]; then
    echo "error: bootstrap Python is not executable: $bootstrap_python" >&2
    exit 2
fi

if [[ -e "$workspace_root/.venv" && ! -f "$workspace_root/.venv/pyvenv.cfg" ]]; then
    echo "error: refusing to use an existing non-venv path: $workspace_root/.venv" >&2
    exit 2
fi

if [[ ! -f "$workspace_root/.venv/pyvenv.cfg" ]]; then
    echo "Creating $workspace_root/.venv with system ROS packages"
    "$bootstrap_python" -m venv --system-site-packages "$workspace_root/.venv"
else
    echo "Reusing $workspace_root/.venv"
fi

venv_python="$workspace_root/.venv/bin/python"
if [[ ! -x "$venv_python" ]]; then
    echo "error: venv Python is missing: $venv_python" >&2
    exit 2
fi

if ! "$venv_python" -c "import colcon_core"; then
    echo "error: the venv cannot import system colcon packages" >&2
    echo "Install python3-colcon-common-extensions and recreate the venv with --system-site-packages." >&2
    exit 2
fi

install -m 0755 "$script_dir/colcon-venv" "$workspace_root/.venv/bin/colcon"
"$venv_python" -m pip install pybullet

"$venv_python" -c "import colcon_core; print('colcon:', colcon_core.__file__)"
"$venv_python" -c "import pybullet; print('pybullet: available')"

echo
echo "Bootstrap complete. Activate it in this shell:"
echo "  source $workspace_root/.venv/bin/activate"
echo "  hash -r"
echo "Then verify:"
echo "  command -v colcon"
echo "Expected: $workspace_root/.venv/bin/colcon"
