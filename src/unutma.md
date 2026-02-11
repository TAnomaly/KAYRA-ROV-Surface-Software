Raspberry Pi 5 + Phantom Sublink ROV – Network Notları

Phantom sublink bir kablodur. IP almaz.
IP adresleri cihazlara verilir:
– Ground Station (PC / Laptop)
– Raspberry Pi 5 (ROV)

IP kabloya değil, cihazlara verilir.

Fiziksel bağlantı:
PC ethernet portu → Phantom sublink → Raspberry Pi 5 ethernet portu

Statik IP yapılandırması:

PC (Ground Station):
IP: 192.168.2.1
Subnet: 255.255.255.0
Gateway: boş

Raspberry Pi 5 (ROV):
IP: 192.168.2.2
Subnet: 255.255.255.0
Gateway: boş

DHCP kullanılmaz.
Gateway gerekmez (internet yok).

Raspberry Pi 5 statik IP (NetworkManager):

nmcli con add type ethernet ifname eth0 con-name rov-eth ipv4.method manual ipv4.addresses 192.168.2.2/24
nmcli con up rov-eth

Alternatif (dhcpcd):

sudo nano /etc/dhcpcd.conf

Dosya sonuna ekle:

interface eth0
static ip_address=192.168.2.2/24

Kaydet ve reboot:

sudo reboot

Bağlantı testi (PC’den):

ping 192.168.2.2

Ping geliyorsa bağlantı tamam.

Haberleşme prensibi:

Ground Station ↔ ROV : Ethernet (UDP)
Kamera görüntüsü : Ethernet
Joystick / komut : Ethernet
COM / UART : Sadece ROV içinde

COM port sublink yerine kullanılmaz.
Video + kontrol + telemetri aynı Ethernet hattından gider.

Basit ROV iç yapısı:

Raspberry Pi 5
– Kamera
– Motor kontrol (PCA9685 üzerinden)

PCA9685 → ESC → Motorlar (6–7 adet)

Motorlar GPIO’dan direkt sürülmez.
UART sadece ekstra MCU varsa kullanılır.

Altın cümle:

Sublink = yol
IP = cihaz
Ethernet = dış dünya
UART / I2C = ROV içi