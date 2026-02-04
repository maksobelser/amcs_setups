## AMCS setups

This is a repo for AMCS setups, should contain all the different solutions.

## Run on boot (macOS)

To keep the gateway running across reboots and power loss, use `launchd`.

### 1) Edit the launchd plist

Update the paths and options in:

`launchd/com.amcs.moxa.plist`

Key fields to update:
1. Script path: `/Users/maks/Documents/2_services/AMCS/Tanzanite/read_moxa_gateway.py`
2. Backup directory: `/var/log/moxa_backups` (change if you want a different location)
3. Backup interval: `--backup-every-min 5`
4. Output/log paths if needed

### 2) Install as a LaunchDaemon (runs at boot, even without login)

Note: The default port is `502` (privileged port). LaunchDaemons run as root, which is required for ports < 1024.

Commands:
```bash
sudo cp /Users/maks/Documents/2_services/AMCS/Tanzanite/launchd/com.amcs.moxa.plist /Library/LaunchDaemons/
sudo launchctl bootstrap system /Library/LaunchDaemons/com.amcs.moxa.plist
sudo launchctl enable system/com.amcs.moxa
```

To check status:
```bash
sudo launchctl print system/com.amcs.moxa
```

To stop/uninstall:
```bash
sudo launchctl bootout system /Library/LaunchDaemons/com.amcs.moxa.plist
sudo rm /Library/LaunchDaemons/com.amcs.moxa.plist
```

### 3) Alternative: LaunchAgent (runs only when the user is logged in)

If you do not need port `502` or want to run without root, change the port to something >= 1024 and install as a user LaunchAgent:

```bash
cp /Users/maks/Documents/2_services/AMCS/Tanzanite/launchd/com.amcs.moxa.plist ~/Library/LaunchAgents/
launchctl bootstrap gui/$(id -u) ~/Library/LaunchAgents/com.amcs.moxa.plist
launchctl enable gui/$(id -u)/com.amcs.moxa
```

Stop/uninstall:
```bash
launchctl bootout gui/$(id -u) ~/Library/LaunchAgents/com.amcs.moxa.plist
rm ~/Library/LaunchAgents/com.amcs.moxa.plist
```
