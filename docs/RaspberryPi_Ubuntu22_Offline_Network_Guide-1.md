# Raspberry Pi - Ubuntu Server 22.04

## Complete Offline Network Configuration Guide

This guide contains ALL commands needed to configure: - Wi-Fi -
Ethernet - Static IP - DHCP - Network troubleshooting - Service
restart - Safe SSH reconfiguration

Works completely offline.

------------------------------------------------------------------------

# 1. Check Network Interfaces

``` bash
ip a
```

Look for: - eth0 → Ethernet - wlan0 → Wi-Fi

------------------------------------------------------------------------

# 2. Check Wi-Fi Not Blocked

``` bash
rfkill list
```

If blocked:

``` bash
sudo rfkill unblock wifi
```

------------------------------------------------------------------------

# 3. Locate Netplan Configuration

``` bash
ls /etc/netplan/
```

Usually:

``` bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

------------------------------------------------------------------------

# 4. Configure Wi-Fi (DHCP)

Replace SSID and PASSWORD.

``` yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "SSID":
          password: "PASSWORD"
```

Apply:

``` bash
sudo netplan generate
sudo netplan apply
```

Safer when remote:

``` bash
sudo netplan try
```

------------------------------------------------------------------------

# 5. Configure Ethernet (DHCP)

``` yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
```

Apply:

``` bash
sudo netplan apply
```

------------------------------------------------------------------------

# 6. Configure Both Ethernet + Wi-Fi

``` yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: true
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "SSID":
          password: "PASSWORD"
```

------------------------------------------------------------------------

# 7. Set Static IP (Ethernet Example)

``` yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply:

``` bash
sudo netplan apply
```

------------------------------------------------------------------------

# 8. Scan Available Wi-Fi Networks

``` bash
sudo iw dev wlan0 scan | grep SSID
```

Or:

``` bash
sudo iwlist wlan0 scan | grep ESSID
```

------------------------------------------------------------------------

# 9. Restart Network Services

``` bash
sudo systemctl restart systemd-networkd
```

Or reboot:

``` bash
sudo reboot
```

------------------------------------------------------------------------

# 10. Check Network Status

``` bash
networkctl status
```

``` bash
systemctl status systemd-networkd
```

------------------------------------------------------------------------

# 11. Verify Connection

``` bash
ip a
```

``` bash
ping 8.8.8.8
```

``` bash
ping google.com
```

------------------------------------------------------------------------

# 12. View Logs (Debugging)

``` bash
journalctl -u systemd-networkd
```

------------------------------------------------------------------------

# 13. Check DNS Configuration

``` bash
cat /etc/resolv.conf
```

------------------------------------------------------------------------

# 14. Safe SSH Reconfiguration Procedure

If connected via SSH and changing Wi-Fi:

1.  Keep Ethernet connected.
2.  Run:

``` bash
sudo netplan try
```

3.  Confirm if connection works.
4.  If broken, system auto-rolls back.

------------------------------------------------------------------------

# 15. Minimal Quick Setup (Fast Copy-Paste)

``` bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Paste:

``` yaml
network:
  version: 2
  renderer: networkd
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "SSID":
          password: "PASSWORD"
```

Then:

``` bash
sudo netplan apply
```

------------------------------------------------------------------------

# 16. Common Problems Checklist

No IP?

``` bash
ip a
```

Wi-Fi blocked?

``` bash
rfkill list
```

Wrong config?

``` bash
sudo netplan try
```

Service not running?

``` bash
sudo systemctl status systemd-networkd
```

------------------------------------------------------------------------

END OF OFFLINE NETWORK GUIDE
