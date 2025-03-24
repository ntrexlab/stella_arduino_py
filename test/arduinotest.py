import serial
import time

arduino_port = '/dev/ttyACM0'
baud_rate = 9600

ser = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  

try:
    while True:

        line = ser.readline().decode('utf-8').strip()
        
        if line.isdigit():
            value = int(line)
            print(f"{value}")
            
            led_state = value >= 600

            if led_state:
                ser.write(b'\x01')  
            else:
                ser.write(b'\x00') 
        
        time.sleep(0.1)  

except KeyboardInterrupt:
    print("\nStopping communication...")

finally:
    ser.close()
