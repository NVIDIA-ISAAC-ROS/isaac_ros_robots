#!/usr/bin/env python3

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Use this script to setup a network interface to be used with a specific robot."""

import argparse
import shutil
import subprocess
import sys
import time

# pylint: disable=inconsistent-quotes

# Robot configurations dictionary.
ROBOT_CONFIGURATIONS = {
    'Unitree G1': {
        'ip_address': '192.168.123.99',
        'subnet_mask': '255.255.255.0',
        'gateway': '',
        'connection_name': 'Unitree G1',
        'robot_ip_address': '192.168.123.161',
    },
    'Booster T1': {
        'ip_address': '192.168.10.1',
        'subnet_mask': '255.255.255.0',
        'gateway': '192.168.10.1',
        'connection_name': 'Booster T1',
        'robot_ip_address': '192.168.10.101',
    },
}


class Colors:
    """Color codes for terminal output."""

    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    NC = '\033[0m'  # No Color


def print_info(message: str) -> None:
    """Print an info message with green color."""
    print(f'{Colors.GREEN}[INFO]{Colors.NC} {message}')


def print_warning(message: str) -> None:
    """Print a warning message with yellow color."""
    print(f'{Colors.YELLOW}[WARNING]{Colors.NC} {message}')


def print_error(message: str) -> None:
    """Print an error message with red color."""
    print(f'{Colors.RED}[ERROR]{Colors.NC} {message}')


def run_command(command: list[str], check: bool = True) -> subprocess.CompletedProcess:
    """Run a command and return the result."""
    result = subprocess.run(command, check=check, capture_output=True, text=True)
    return result


def check_sudo() -> None:
    """Check if sudo is available and working."""
    if not shutil.which('sudo'):
        print_error('sudo command not found. Please install sudo or run as root.')
        sys.exit(1)

    # Test if sudo works without password.
    result = run_command(['sudo', '-n', 'true'], check=False)
    if result.returncode != 0:
        print_info('Some commands require sudo privileges. You may be prompted for your password.')


def select_robot_type() -> dict[str, str]:
    """Interactively select robot type and return configuration."""
    print_info('Select robot type for network configuration:')
    print()

    robot_names = list(ROBOT_CONFIGURATIONS.keys())
    for i, robot_name in enumerate(robot_names, 1):
        print(f'  {i}) {robot_name}')
    print()

    while True:
        try:
            choice = input(f'Select robot type (1-{len(robot_names)}): ')
            choice_num = int(choice)

            if 1 <= choice_num <= len(robot_names):
                selected_robot = robot_names[choice_num - 1]
                config = ROBOT_CONFIGURATIONS[selected_robot].copy()
                config['robot_type'] = selected_robot

                print_info(f'Selected: {selected_robot}')
                print()
                print_info(f'Configuration for {selected_robot}:')
                print_info(f'  IP Address: {config["ip_address"]}')
                print_info(f'  Subnet Mask: {config["subnet_mask"]}')
                if config['gateway']:
                    print_info(f'  Gateway: {config["gateway"]}')
                print()

                return config
            else:
                print_error(
                    f'Invalid selection. Please enter a number between 1 and {len(robot_names)}.'
                )
        except ValueError:
            print_error('Invalid input. Please enter a number.')


def get_available_interfaces() -> list[tuple[str, str, str]]:
    """Get list of available network interfaces with their status and IP info."""
    interfaces = []

    # Get all network interfaces.
    result = run_command(['sudo', 'ip', 'link', 'show'])
    lines = result.stdout.split('\n')

    for line in lines:
        # Look for lines that start with a number followed by a colon and interface name.
        if ':' in line and line.strip().split(':')[0].isdigit():
            parts = line.split(':')
            if len(parts) >= 2:
                iface = parts[1].strip()

                # Skip loopback and virtual interfaces.
                if iface != 'lo' and not any(
                    iface.startswith(prefix)
                    for prefix in ['docker', 'veth', 'br-', 'virbr', 'vmnet']
                ):
                    # Get interface status.
                    status_result = run_command(
                        ['sudo', 'ip', 'link', 'show', iface], check=False
                    )
                    if status_result.returncode == 0:
                        status = 'UNKNOWN'
                        for status_line in status_result.stdout.split('\n'):
                            if 'state' in status_line:
                                status = status_line.split('state')[1].strip().split()[0]
                                break

                        # Get IP info.
                        ip_result = run_command(['sudo', 'ip', 'addr', 'show', iface], check=False)
                        ip_info = 'No IP assigned'
                        for ip_line in ip_result.stdout.split('\n'):
                            if 'inet ' in ip_line and '127.0.0.1' not in ip_line:
                                ip_info = ip_line.split('inet ')[1].split()[0]
                                break

                        interfaces.append((iface, status, ip_info))

    return interfaces


def select_interface() -> str:
    """Interactively select network interface."""
    print_info('Scanning for available network interfaces...')
    print()

    interfaces = get_available_interfaces()

    if not interfaces:
        print_error('No network interfaces found.')
        sys.exit(1)

    print_info(f'Found {len(interfaces)} network interface(s):')
    print()

    for i, (iface, status, ip_info) in enumerate(interfaces, 1):
        # Format status with colors.
        if status == 'UP':
            status_display = f'{Colors.GREEN}UP{Colors.NC}'
        elif status == 'DOWN':
            status_display = f'{Colors.RED}DOWN{Colors.NC}'
        else:
            status_display = f'{Colors.YELLOW}{status}{Colors.NC}'

        print(f'  {i}) {iface} ({status_display}) - {ip_info}')

    print()

    while True:
        try:
            choice = input(f'Select network interface (1-{len(interfaces)}): ')
            choice_num = int(choice)

            if 1 <= choice_num <= len(interfaces):
                selected_interface = interfaces[choice_num - 1][0]
                print_info(f'Selected interface: {selected_interface}')
                return selected_interface
            else:
                print_error(
                    f'Invalid selection. Please enter a number between 1 and {len(interfaces)}.'
                )
        except ValueError:
            print_error('Invalid input. Please enter a number.')


def backup_config(config: dict[str, str], interface: str) -> str:
    """Backup current network configuration."""
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    backup_file = f'/tmp/network_backup_{timestamp}.txt'

    print_info(f'Backing up current network configuration to {backup_file}')

    with open(backup_file, 'w', encoding='utf-8') as f:
        f.write(f'# Network configuration backup - {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
        f.write(f'# Robot Type: {config["robot_type"]}\n')
        f.write(f'# Connection: {config["connection_name"]}\n')
        f.write(f'# Interface: {interface}\n')
        f.write(f'# IP Address: {config["ip_address"]}\n')
        f.write(f'# Subnet Mask: {config["subnet_mask"]}\n')
        if config['gateway']:
            f.write(f'# Gateway: {config["gateway"]}\n')
        f.write('\n')

        # Current IP configuration.
        f.write('# Current IP configuration:\n')
        result = run_command(['sudo', 'ip', 'addr', 'show', interface], check=False)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'inet ' in line or 'inet6 ' in line:
                    f.write(line + '\n')
        else:
            f.write('# No IP addresses found\n')

        f.write('\n')

        # Current routes.
        f.write('# Current routes:\n')
        result = run_command(['sudo', 'ip', 'route', 'show'], check=False)
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if interface in line:
                    f.write(line + '\n')
        else:
            f.write('# No routes found for this interface\n')

    print_info(f'Backup saved to: {backup_file}')
    return backup_file


def configure_interface(config: dict[str, str], interface: str) -> None:
    """Configure the network interface."""
    print_info(
        f'Configuring network interface {interface} for connection: {config["connection_name"]}'
    )
    print_info(f'Setting IP address: {config["ip_address"]}')
    print_info(f'Setting subnet mask: {config["subnet_mask"]}')
    if config['gateway']:
        print_info(f'Setting gateway: {config["gateway"]}')

    # Clear any existing IP addresses on the interface.
    run_command(['sudo', 'ip', 'addr', 'flush', 'dev', interface], check=False)

    # Bring up the interface first.
    run_command(['sudo', 'ip', 'link', 'set', interface, 'up'])

    # Wait a moment for the interface to be ready.
    time.sleep(1)

    # Calculate CIDR notation from subnet mask.
    cidr = sum(bin(int(x)).count('1') for x in config['subnet_mask'].split('.'))

    # Configure the IP address and subnet mask.
    run_command(['sudo', 'ip', 'addr', 'add', f'{config["ip_address"]}/{cidr}', 'dev', interface])

    # Configure gateway if specified.
    if config['gateway']:
        run_command(
            ['sudo', 'ip', 'route', 'add', 'default', 'via', config['gateway'], 'dev', interface]
        )

    # Add multicast route so DDS/CycloneDDS traffic uses this interface instead of defaulting
    # to the primary network interface (e.g. WiFi). Without this, multicast packets from the
    # robot arrive at the Ethernet level but are never delivered to UDP sockets.
    run_command(['sudo', 'ip', 'route', 'add', '239.0.0.0/8', 'dev', interface], check=False)
    print_info(f'Added multicast route 239.0.0.0/8 via {interface}')

    # Allow DDS traffic through the firewall. UFW defaults to INPUT DROP, which silently blocks
    # incoming UDP on DDS discovery ports (7400-7420) from the robot subnet.
    subnet = config['ip_address'].rsplit('.', 1)[0] + '.0/24'
    run_command(['sudo', 'ufw', 'allow', 'in', 'on', interface, 'proto', 'udp',
                 'from', subnet], check=False)
    print_info(f'Allowed incoming UDP from {subnet} on {interface} through firewall')

    # Verify the configuration.
    result = run_command(['sudo', 'ip', 'addr', 'show', interface], check=False)
    if config['ip_address'] in result.stdout:
        print_info('Network configuration successful!')
        print_info(
            f"Connection '{config['connection_name']}' is now configured on interface {interface}:"
        )
        print_info(f'  IP Address: {config["ip_address"]}')
        print_info(f'  Subnet Mask: {config["subnet_mask"]}')
        if config['gateway']:
            print_info(f'  Gateway: {config["gateway"]}')
    else:
        print_error('Failed to configure network interface.')
        sys.exit(1)


def ping_robot(robot_ip: str) -> bool:
    """Try to ping the robot IP address."""
    print_info(f'Testing connectivity to robot at {robot_ip}...')

    result = run_command(['ping', '-c', '1', '-W', '3', robot_ip], check=False)
    if result.returncode == 0:
        print_info(f'Robot at {robot_ip} is reachable')
        return True
    else:
        print_warning(f'Robot at {robot_ip} is not reachable')
        print_warning('This could be normal if the robot is not yet connected')
        return False


def main() -> None:
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Configure network interface for robot communication'
    )
    parser.add_argument('-i', '--interface', help='Specify network interface')
    args = parser.parse_args()

    print_info('Starting network configuration script')

    # Check if sudo is available.
    check_sudo()

    # Select robot type and configure settings.
    config = select_robot_type()

    # Select network interface if not specified.
    if args.interface:
        interface = args.interface
        # Verify the interface exists.
        result = run_command(['sudo', 'ip', 'link', 'show', interface], check=False)
        if result.returncode != 0:
            print_error(f"Network interface '{interface}' does not exist.")
            sys.exit(1)
        print_info(f'Using specified interface: {interface}')
    else:
        interface = select_interface()

    # Backup current configuration.
    backup_config(config, interface)

    # Configure the interface.
    configure_interface(config, interface)

    # Try to ping the robot.
    ping_robot(config['robot_ip_address'])

    print_info('Network setup completed successfully!')
    print_info(f"Connection '{config['connection_name']}' is now active on interface {interface}")
    print_warning('Note: This configuration is temporary and will be lost on reboot.')


if __name__ == '__main__':
    main()
