# OpenAMS for Klipper  
OpenAMS Klipper Plugin

## Installation  

### Automatic Installation  

Install OpenAMS using the provided script:  

```bash  
cd ~  
git clone https://github.com/OpenAMSOrg/klipper_openams.git  
cd klipper_openams  
./install-openams.sh  
```  

If your directory structure differs, you can configure the installation script with additional parameters:  

```bash  
./install-openams.sh [-k <klipper path>] [-s <klipper service name>] [-c <configuration path>]  
```  

---

### Manual Installation  

1. Clone the repository:  

```bash  
cd ~  
git clone https://github.com/OpenAMSOrg/klipper_openams.git  
```  

2. Stop Klipper:  

```bash  
systemctl stop klipper  
```  

3. Link the necessary files (adjust paths if needed):  

```bash  
ln -s klipper_openams/openams.py ~/klipper/klippy/extras/openams.py  
```  

4. Restart Klipper:  

```bash  
systemctl start klipper  
```  

5. Add the updater section to `moonraker.conf` for updates:  

```ini  
[update_manager openams]  
type: git_repo  
path: ~/klipper_openams  
origin: https://github.com/OpenAMSOrg/klipper_openams.git  
is_system_service: False  
```  

6. Restart Moonraker.  

---

## Credits  

This project was made by Wick69 and knight.rad_iant on Discord, with support from the OpenAMS community.  

---