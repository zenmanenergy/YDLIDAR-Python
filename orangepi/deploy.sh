#!/usr/bin/env bash
#
# Installs the lidar service on the Orange Pi (or any Debian/Ubuntu machine).
# Run from this folder after cloning the repo:
#
#     sudo ./deploy.sh                    install or update code, keep existing config
#     sudo ./deploy.sh --replace-config   also overwrite /opt/lidar/config.json with this folder's copy
#
# Needs internet the first time (apt and pip). Safe to run again any time.

set -euo pipefail

INSTALL_DIR=/opt/lidar
SERVICE=lidar
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPLACE_CONFIG=0

for arg in "$@"; do
    case "$arg" in
        --replace-config) REPLACE_CONFIG=1 ;;
        *) echo "unknown option: $arg"; exit 1 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo ./deploy.sh"
    exit 1
fi

echo "== Installing system packages"
apt-get update -qq
apt-get install -y -qq python3 python3-venv python3-pip > /dev/null

echo "== Copying files to $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
install -m 644 "$SRC_DIR"/gs2.py "$SRC_DIR"/geometry.py "$INSTALL_DIR"/
install -m 755 "$SRC_DIR"/lidar.py "$SRC_DIR"/calibrate.py "$INSTALL_DIR"/
install -m 644 "$SRC_DIR"/requirements.txt "$INSTALL_DIR"/
mkdir -p "$INSTALL_DIR/tools"
install -m 755 "$SRC_DIR"/tools/fake_gs2.py "$INSTALL_DIR/tools/"

if [[ ! -f "$INSTALL_DIR/config.json" ]]; then
    install -m 644 "$SRC_DIR/config.json" "$INSTALL_DIR/config.json"
    echo "   installed config.json"
elif [[ $REPLACE_CONFIG -eq 1 ]]; then
    cp "$INSTALL_DIR/config.json" "$INSTALL_DIR/config.json.bak"
    install -m 644 "$SRC_DIR/config.json" "$INSTALL_DIR/config.json"
    echo "   replaced config.json (old copy saved as config.json.bak)"
elif ! cmp -s "$SRC_DIR/config.json" "$INSTALL_DIR/config.json"; then
    echo "   NOTE: $INSTALL_DIR/config.json differs from the repo copy and was kept."
    echo "         Use --replace-config to overwrite it."
fi

echo "== Setting up Python environment"
if [[ ! -x "$INSTALL_DIR/venv/bin/python" ]]; then
    python3 -m venv "$INSTALL_DIR/venv"
fi
"$INSTALL_DIR/venv/bin/pip" install -q --upgrade pip
"$INSTALL_DIR/venv/bin/pip" install -q pyserial pyntcore
if ! "$INSTALL_DIR/venv/bin/pip" install -q robotpy-wpimath; then
    echo "   robotpy-wpimath not available for this platform; AdvantageScope struct output disabled (everything else works)"
fi

echo "== Installing service"
install -m 644 "$SRC_DIR/lidar.service" /etc/systemd/system/$SERVICE.service
systemctl daemon-reload
systemctl enable $SERVICE > /dev/null
systemctl restart $SERVICE

echo "== Checking config"
"$INSTALL_DIR/venv/bin/python" - "$INSTALL_DIR/config.json" <<'EOF'
import json, os, sys
cfg = json.load(open(sys.argv[1]))
problems = []
if not cfg.get("server") and int(cfg.get("team", 0)) == 0:
    problems.append('"team" is 0. Set your team number so it can find the roboRIO.')
for lidar in cfg["lidars"]:
    if lidar.get("enabled", True) and not os.path.exists(lidar["port"]):
        problems.append('%s: port %s not found.' % (lidar["name"], lidar["port"]))
for p in problems:
    print("   WARNING: " + p)
if any("port" in p for p in problems):
    print("   Connected serial ports:")
    path = "/dev/serial/by-path"
    if os.path.isdir(path):
        for name in sorted(os.listdir(path)):
            print("     " + os.path.join(path, name))
    else:
        print("     (none found; is a lidar plugged in?)")
if not problems:
    print("   config looks good")
EOF

echo
echo "Done. Useful commands:"
echo "  journalctl -u $SERVICE -f          live log"
echo "  sudo systemctl restart $SERVICE    restart after editing $INSTALL_DIR/config.json"
echo "  sudo systemctl stop $SERVICE       stop (needed before running calibrate.py)"
