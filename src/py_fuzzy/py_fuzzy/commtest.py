import serial
import time

# Port ve baudrate'i kendi ayarlarınıza göre değiştirin
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # STM32'nin hazır olması için bekle

# Göndermek istediğiniz veriyi newline ile birlikte gönderin
data = "j1=0,j2=0,j3=45,j4=0\n\r"
ser.write(data.encode())  # String'i byte tipine çevirip gönder

# Yanıtı kontrol etmek isterseniz:
response = ser.readline()
print("STM32'den yanıt:", response.decode().strip())

ser.close()