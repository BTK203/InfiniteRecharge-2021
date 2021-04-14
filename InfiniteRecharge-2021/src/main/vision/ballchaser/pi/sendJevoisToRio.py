import SerialUDPInterface as SUI
import serial

#data = SUI.readSerial("/dev/ttyUSB0", 115200, 1)
while 1:
    try:
        with serial.Serial("/dev/serial/by-id/usb-JeVois_Inc_JeVois-A33_Smart_Camera-if02", baudrate=115200, timeout=1, bytesize=8) as ser:
            while 1:
                data = ""
                dataBytes = ser.readline().rstrip()
                data += dataBytes.decode("utf-8")
                print (data)
                print (len(dataBytes))
                if (len(dataBytes) > 0):
                    dataBytes += ";".encode("utf-8")
                    SUI.sendUDP("10.36.95.2", 3696, dataBytes)
    except KeyboardInterrupt:
        exit()
    except:
        print ("JEVOIS NOT CONNECTED!")