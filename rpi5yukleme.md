# PC'den RPi'ye kopyala
scp -r rpi/* pi@192.168.2.2:~/kayra-rov/

# RPi'de derle ve calistir
ssh pi@192.168.2.2
cd ~/kayra-rov
make
./kayra-rov              # normal mod
./kayra-rov --no-pwm     # masa testi (motor yok)
./kayra-rov --no-camera  # kamera olmadan

# Boot'ta otomatik baslat
sudo make install
sudo systemctl start kayra-rov