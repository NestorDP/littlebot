#!/bin/bash

# Define the udev rules file path
RULES_FILE="/etc/udev/rules.d/99-rfcomm-serial.rules"
RULES_CONTENT='KERNEL=="rfcomm0", SUBSYSTEM=="tty", SYMLINK+="serial/by-id/littleBOT"'

# Check if the udev rules file exists
if [ ! -f "$RULES_FILE" ]; then
    echo "Creating udev rules file: $RULES_FILE"
    echo "$RULES_CONTENT" | sudo tee "$RULES_FILE" > /dev/null
    
    if [ $? -eq 0 ]; then
        echo "Udev rules file created successfully"
        echo "Reloading udev rules..."
        sudo udevadm control --reload-rules
        sudo udevadm trigger
    else
        echo "Error: Failed to create udev rules file"
        exit 1
    fi
else
    echo "Udev rules file already exists: $RULES_FILE"
fi

# Scan for littleBOT Bluetooth device
echo "Scanning for littleBOT Bluetooth device..."
DEVICE_INFO=$(hcitool scan | grep -i littlebot)

if [ -z "$DEVICE_INFO" ]; then
    echo "Error: littleBOT device not found in hcitool scan"
    echo "Make sure the device is powered on and in discoverable mode"
    exit 1
fi

# Extract the MAC address (first field of the grep result)
MAC_ADDRESS=$(echo "$DEVICE_INFO" | awk '{print $1}')

echo "Found littleBOT device with MAC address: $MAC_ADDRESS"

# Check if rfcomm0 is already connected
if [ -e "/dev/rfcomm0" ]; then
    echo "rfcomm0 already exists. Releasing existing connection..."
    sudo rfcomm release /dev/rfcomm0
    sleep 2
fi

# Create RFCOMM connection
echo "Connecting to littleBOT via RFCOMM..."
sudo rfcomm connect /dev/rfcomm0 "$MAC_ADDRESS"

# Wait a moment for connection to establish
sleep 3

# Verify connection
if [ -e "/dev/rfcomm0" ]; then
    echo "RFCOMM connection established successfully on /dev/rfcomm0"
    echo "Device should also be available at /dev/serial/by-id/littleBOT"
else
    echo "Warning: RFCOMM connection may not have been established"
    echo "Check if /dev/rfcomm0 exists"
fi