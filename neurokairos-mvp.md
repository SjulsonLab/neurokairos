# Neurokairos MVP (Minimum Viable Product) Implementation Plan

## Objectives

Neurokairos is a Raspberry Pi–based time synchronization appliance that provides:

- NTP time synchronization
- IRIG and related timing outputs
- GNSS/PPS support (optional)
- Monitoring and notifications
- Simple web-based configuration

The Minimum Viable Product (MVP) should:

- Use a single SD card image for all deployments.
- Require no display, keyboard, mouse, buttons, or serial console for normal setup.
- Be deployable by non-technical users.
- Support both NTP servers and NTP clients through configuration rather than separate images.

---

# Time Source Configuration

The setup wizard should avoid networking jargon whenever possible.

Instead of asking whether the device is a "server" or "client", the wizard asks:

## Where should this device obtain its time?

### Option 1: GNSS Receiver Attached (Recommended for Time Servers)

A GNSS receiver (GPS, Galileo, etc.) and PPS source are connected directly to this Neurokairos device.

Configuration:

```text
GNSS/PPS
     ↓
Neurokairos
```

Chrony configuration includes:

- GNSS/PPS as primary source
- Internet time servers as backup sources

If GNSS lock is lost, the system can continue using WAN NTP sources and holdover behavior.

This is the recommended configuration for a building's primary time server.

---

### Option 2: Local Time Server or Time Appliance

This device obtains time from another device on the local network.

Examples:

- Another Neurokairos server
- TM2000
- Meinberg appliance
- Existing institutional NTP appliance

Configuration:

```text
Local NTP server/appliance
          ↓
      Neurokairos
```

User specifies:

```text
Primary server IP address
Optional backup server IP address
```

Example:

```text
10.49.98.220
10.49.98.221
```

This is the recommended configuration for Neurokairos client devices.

---

### Option 3: Internet Time Servers Only

This device synchronizes directly to public Internet time servers.

Configuration:

```text
Internet time servers
          ↓
      Neurokairos
```

Useful when:

- No GNSS receiver is available
- No local NTP server exists
- Simplicity is preferred over maximum timing accuracy

---

# Serving Time to Other Devices

Separate from the time-source selection:

```text
Should this device provide NTP service to other devices?
```

Options:

- Yes
- No

Examples:

### Building Time Server

```text
GNSS/PPS + WAN backup
          ↓
      Neurokairos
          ↓
     Other devices
```

Time source:

- GNSS Receiver Attached

Serve NTP:

- Yes

---

### Neurokairos Client

```text
Building NTP server
          ↓
      Neurokairos
```

Time source:

- Local Time Server or Time Appliance

Serve NTP:

- No

---

# Notifications (ntfy)

Neurokairos uses ntfy to send push notifications to phones and computers.

see neurokairos-notifications.md document

The user can continue setup without ntfy if desired.

---

# Recommended Deployment Models

## Model 1: GNSS-Based Building Time Server

```text
GNSS/PPS
     ↓
Neurokairos Server
     ↓
All Neurokairos Clients
```

Recommended default deployment.

Configuration:

Time Source:

- GNSS Receiver Attached

Serve NTP:

- Yes

---

## Model 2: Existing Institutional Time Appliance

```text
TM2000 / Meinberg / Other Appliance
                 ↓
          Neurokairos Clients
```

Configuration:

Time Source:

- Local Time Server or Time Appliance

Serve NTP:

- No

---

## Model 3: Redundant GNSS Servers

```text
GNSS Server A
GNSS Server B
       ↓
      Clients
```

Clients configured with:

```text
Primary server IP
Backup server IP
```

Configuration:

Time Source:

- Local Time Server or Time Appliance

Serve NTP:

- No

This provides resilience against server failure.