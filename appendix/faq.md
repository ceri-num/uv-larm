# Frequent Asked Question...

## There is no Wifi on my dell-xps13 ???

1. Connect with cable
2. Get the appropriate drivers: [killer driver](https://support.killernetworking.com/knowledge-base/killer-ax1650-in-debian-ubuntu-16-04/)

```bash
sudo add-apt-repository ppa:canonical-hwe-team/ backport-iwlwifi
sudo apt-get update
sudo apt-get install backport-iwlwifi-dkms
```

3. reboot
