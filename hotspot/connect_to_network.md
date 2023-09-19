Enable wifi

```bash
sudo nmcli radio wifi on
```

List networks

```bash
nmcli dev wifi list
```

Connect to a network

```bash
sudo nmcli --ask dev wifi connect network-ssid
```
