# Neurokairos MVP (minimum viable product) Implementation Plan

## Objectives

Neurokairos is a Raspberry Pi–based time synchronization appliance that provides:

- NTP time synchronization
- IRIG and related timing outputs
- GNSS/PPS support (optional)
- Monitoring and notifications
- Simple web-based configuration

The MVP should:

- Use a single SD card image for all deployments.
- Require no display, keyboard, mouse, buttons, or serial console for normal setup.
- Be deployable by non-technical users.
- Support both server and client deployments through configuration rather than separate images.

---

# System Architecture

## Base Platform

- Raspberry Pi OS Lite
- chrony
- avahi-daemon
- OpenSSH
- Python 3
- Lightweight web UI (Flask)
- systemd services

Single image for all systems.

---

# Initial Boot Experience

Factory configuration:

```text
Hostname: neurokairos
mDNS name: neurokairos.local
User: pi
Password: neurokairos
```

Workflow:

1. User flashes SD card.
2. User powers device and connects Ethernet.
3. Device boots.
4. User visits:

   http://neurokairos.local

5. Setup wizard appears.

---

# First-Boot Identity Generation

Before image creation:

Remove:

- SSH host keys
- machine-id

On first boot:

Generate:

- SSH host keys
- machine-id
- Neurokairos device UUID

Store:

```text
/etc/neurokairos/device-id
```

Used for:

- Logging
- Diagnostics
- Notifications
- Internal identification

---

# Setup Wizard

The setup wizard gathers:

## Hostname

Example:

```text
neurokairos-server
```

Validation:

- Must be unique.
- Must not equal "neurokairos".
- Must contain valid hostname characters.

Hostname collision detection:

Attempt mDNS resolution of:

```text
hostname.local
```

If already present on the network, reject.

After acceptance:

```text
hostnamectl set-hostname <hostname>
```

Restart Avahi.

---

## Time Source

Options:

### GNSS/PPS Attached

Local GNSS receiver provides time.

### Neurokairos/NTP Server

User specifies:

```text
Primary server IP
Optional backup server IP
```

### Institutional NTP Appliance

User specifies IP.

### WAN NTP

Uses public NTP sources.

---

## NTP Service

Question:

```text
Serve NTP to other devices?
```

Options:

- Yes
- No

This determines whether chrony allows client access.

---

## Notifications

Optional:

```text
ntfy topic
```

If blank:

Generate:

```text
neurokairos-<short-id>
```

---

## Security

Allow user to change password.

Strongly recommended but not mandatory for MVP.

---

# Avahi / Discovery

Purpose:

- Initial discovery
- Browser access
- Convenience

Factory:

```text
neurokairos.local
```

Configured:

```text
<hostname>.local
```

Examples:

```text
neurokairos-server.local
neurokairos-microscope1.local
```

Avahi is not required for production NTP operation.

---

# NTP Configuration Philosophy

Preferred production configuration:

Use reserved DHCP leases or static IP addresses.

Example:

```conf
server 10.49.98.220 iburst prefer
```

Avoid multiple aliases of the same server in chrony.conf.

Do not configure:

```conf
server hostname.local
server hostname.domain.org
server IP
```

when all refer to the same device.

---

# Deployment Models

## Model 1: Single Building Time Server

```text
GNSS or WAN
      ↓
Neurokairos Server
      ↓
All Neurokairos Clients
```

Recommended default.

---

## Model 2: Existing GPS NTP Appliance

```text
GPS Appliance
      ↓
Neurokairos Clients
```

Clients generate local timing outputs.

---

## Model 3: Redundant GNSS Servers

```text
GNSS Server A
GNSS Server B
       ↓
      Clients
```

Clients configured with primary and backup servers.

---

# HDMI Status Console

If a monitor is connected:

Display:

```text
Hostname
IP address
MAC address

Web UI URLs

GNSS status
PPS status
Chrony status

NTP server information
```

Example:

Hostname: neurokairos-server
IP: 10.49.98.220
MAC: dc:a6:32:12:34:56

Web UI:
http://neurokairos-server.local
http://10.49.98.220

GNSS: LOCKED
PPS: OK
Chrony: SYNCHRONIZED

Login:
neurokairos login:

Local login remains available.

---

# Notification System

Use ntfy.

Events:

- IP address changed
- GNSS lost
- PPS lost
- Entered holdover
- Chrony unsynchronized
- Excessive offset
- Temperature warning
- Storage warning
- Reboot
- Software update available

---

# Chrony Monitoring

Expose through web UI:

- Current source
- Stratum
- Offset
- Frequency correction
- Holdover state
- PPS status
- GNSS lock status

Display both summary and detailed chronyc output.

---

# Web UI Pages

## Dashboard

System status.

## Time Sources

Configure chrony.

## Networking

Hostname, IP information, DNS information.

## Notifications

ntfy configuration.

## Diagnostics

Chrony, GNSS, PPS, logs.

## System

Password change, updates, reboot.

---

# Software Updates

Provide update mechanism without reimaging.

Minimum:

```bash
apt update
apt upgrade
```

through web UI.

Future:

Dedicated Neurokairos update channel.

---

# MVP Exclusions

Not required initially:

- Touchscreen support
- Physical buttons
- QR codes
- Wi-Fi configuration
- PTP
- Automatic server discovery
- Multi-device management dashboard
- High-availability clustering
- Cloud services

Focus on:

- Reliable NTP
- Reliable GNSS/PPS integration
- Simple deployment
- Clear diagnostics
- Notification capability
- Stable web-based configuration