import serial

ser = serial.Serial('/dev/ttyUSB0', 3000000, timeout=1)
ser.write(b'\x01\x01\x00\x00\x00\x00?\x00\x00\x00?\x00\x00')
i = 0
while True:
    try:
        i+=1
        # Read the line (until newline)
        line = ser.readline().decode('ascii').strip()  # Decoding and stripping off any trailing whitespace or newline characters
        float_1 = float(line[71:75])
        float_2 = float(line[75:78])
        print(f"Float 1: {float_1}, Float 2: {float_2}, index: {i}")
        # print(line[71:75], len(line) , line[75:])
    except ValueError:
        continue