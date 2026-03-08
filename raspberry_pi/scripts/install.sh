#!/bin/bash

# Install the IRIG sender as a systemd service.
# Chrony config needs to be done separately (see install_chrony_server.sh).
#
# Usage:
#   ./install.sh                    # default: BCM GPIO 11, no inverted pin
#   ./install.sh -p 17              # normal output on GPIO 17
#   ./install.sh -p 17 -n 27        # normal on 17, inverted on 27
#   ./install.sh -w 2.0             # LED warning threshold 2.0 ms
#   ./install.sh -p 17 -n 27 -w 2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Defaults (match irig_sender.c defaults)
NORMAL_PIN=11
INVERTED_PIN=-1
WARN_THRESHOLD=""

while getopts "p:n:w:h" opt; do
    case $opt in
        p) NORMAL_PIN="$OPTARG" ;;
        n) INVERTED_PIN="$OPTARG" ;;
        w) WARN_THRESHOLD="$OPTARG" ;;
        h)
            echo "Usage: $0 [-p PIN] [-n PIN] [-w THRESHOLD]"
            echo "  -p PIN        BCM GPIO pin for normal output (default: 11)"
            echo "  -n PIN        BCM GPIO pin for inverted output (default: -1, disabled)"
            echo "  -w THRESHOLD  LED warning threshold in ms (default: 1.0)"
            exit 0
            ;;
        *)
            echo "Usage: $0 [-p PIN] [-n PIN] [-w THRESHOLD]"
            exit 1
            ;;
    esac
done

# Build the ExecStart command with the user's pin configuration
EXEC_CMD="/usr/local/bin/irig_sender -p $NORMAL_PIN -n $INVERTED_PIN"
if [ -n "$WARN_THRESHOLD" ]; then
    EXEC_CMD="$EXEC_CMD -w $WARN_THRESHOLD"
fi

echo "Installing IRIG sender service..."
echo "  Normal pin:    BCM GPIO $NORMAL_PIN"
if [ "$INVERTED_PIN" -eq -1 ]; then
    echo "  Inverted pin:  disabled"
else
    echo "  Inverted pin:  BCM GPIO $INVERTED_PIN"
fi
if [ -n "$WARN_THRESHOLD" ]; then
    echo "  LED threshold: $WARN_THRESHOLD ms"
fi

# Compile the program
make -C "$REPO_DIR/sender"

# Copy to system location
sudo cp "$REPO_DIR/sender/irig_sender" /usr/local/bin/
sudo chmod +x /usr/local/bin/irig_sender

sudo mkdir -p /var/log/irig-sender

# Generate the service file from the template, substituting in the pin config
sed "s|^ExecStart=.*|ExecStart=$EXEC_CMD|" \
    "$REPO_DIR/systemd/irig-sender.service" | \
    sudo tee /etc/systemd/system/irig-sender.service > /dev/null

# Reload systemd
sudo systemctl daemon-reload

# Enable service to start on boot
sudo systemctl enable irig-sender.service

# Start (or restart) the service now
if systemctl is-active --quiet irig-sender.service; then
    sudo systemctl restart irig-sender.service
    echo "IRIG sender service restarted with new configuration."
else
    sudo systemctl start irig-sender.service
    echo "IRIG sender service started."
fi


echo "Service command: $EXEC_CMD"
echo ""
echo "Useful commands:"
echo "  sudo systemctl status irig-sender    # check status"
echo "  sudo journalctl -u irig-sender -f    # follow logs"
echo "  sudo systemctl restart irig-sender   # restart"
echo "  ./raspberry_pi/scripts/uninstall.sh   # remove"
