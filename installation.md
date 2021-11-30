Download Raspberri Pi Imager
https://www.raspberrypi.org/blog/raspberry-pi-imager-imaging-utility/

Install and Launch Imager, choose Raspberry Pi OS (2021-05-07 release)
Image Micro SD card

Start Pi, it will resize image to fit the card and restart
Next, go through the wizard to select region/time zone/language
Plug in wireless adapter
Connect WAN0 to internet (choose network and password)
Install OS updates

Download and install RaspAP
wget -q https://git.io/voEUQ -O /tmp/raspap && bash /tmp/raspap
Select all defaults, Hotspot should be on WLAN1
Navigate to 10.3.141.1 in Pi browser (username admin, password secret) and change hotspot name and password
Reference: http://raspberry-valley.azurewebsites.net/RaspAp-Wifi-Hotspot/


