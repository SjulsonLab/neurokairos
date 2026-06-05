# Notifications (ntfy)

Neurokairos uses ntfy to send push notifications to phones and computers.

Notifications are optional but strongly recommended for systems acting as time servers.

During setup:

```text
Notifications (optional)
```

Options:

### Generate New Topic

Neurokairos creates a random topic such as:

```text
neurokairos-7A4C38B7
```

### Use Existing Topic

User enters an existing ntfy topic name.

---

# ntfy Setup Workflow

The setup wizard should provide instructions:

1. Install the ntfy application on iPhone or Android.
2. Open the app.
3. Subscribe to the selected topic.
4. Send a test notification.
5. Confirm receipt.

Example:

```text
Topic:
neurokairos-7A4C38B7

Test notification sent.
Did you receive it?
```

The user can continue setup without ntfy if desired.

---

# Notification Philosophy

Notifications should be actionable and should avoid alert fatigue.

Neurokairos should not generate notifications for brief, self-correcting events that commonly occur during normal operation.

General rule:

> Alert on sustained loss of timing resilience, not transient signal interruptions.

Events should use grace periods before generating warnings.

Repeated notifications of the same type should be suppressed to avoid excessive messaging.

---

# Notification Events

## Immediate Notifications

### IP Address Changed

```text
IP address changed.

Old: 10.49.98.220
New: 10.49.98.245
```

### System Rebooted

```text
Neurokairos restarted successfully.

Hostname: neurokairos-server
IP: 10.49.98.220
```

---

## Delayed Warning Notifications

### PPS Signal Lost

- Warning: 5 minutes
- Critical: 30 minutes

### GNSS Signal Lost

- Warning: 10 minutes
- Critical: 60 minutes

### Chrony Unsynchronized

- Warning: 10 minutes
- Critical: 60 minutes

### Operating on Internet Time Servers Because GNSS Is Unavailable

- Warning: 10 minutes
- Critical: 60 minutes

### Local Time Server Unreachable

Applies when Neurokairos is configured to obtain time from another local NTP server or time appliance.

- Warning: 5 minutes
- Critical: 30 minutes

Example:

```text
Local time server 10.49.98.220 unreachable for 5 minutes.
System is operating on holdover.
```

---

## Additional Monitoring

- High temperature
- Low disk space
- Software update available

These should initially appear in the dashboard and may optionally generate notifications later.

---

# Notification Suppression

To reduce alert fatigue:

- Do not repeatedly send identical warnings.
- Suppress duplicate notifications of the same type for 6 hours.
- If severity increases (Warning → Critical), send a new notification immediately.
- When a condition clears, optionally send a recovery notification.

Example:

```text
GNSS signal restored.
System returned to normal operation.
```