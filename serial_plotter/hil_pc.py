import csv
import serial
from time import sleep
ser = serial.Serial('COM3')
# open the line, conf UART
with ser:
    ser.setDTR(False)
    sleep(1)
    ser.flushInput()
    ser.setDTR(True)

ser = serial.Serial('COM3', baudrate=115200, timeout=None)
with open('out_file.csv', mode='w') as out_file:
    #csv_writer = csv.writer(out_file, delimiter=',')

    with open('data4test_03_in.csv') as in_file:
            csv_reader = csv.reader(in_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                    if line_count > 0:
                        # print(row[13])
                        str1 = ','.join(row)
                        str1 = str1 + '\n'
                        ser.write(str1.encode())

                        rxString = ser.readline()
                        # csv_writer.writerow([rxString.decode('utf-8')])
                        out_file.write(rxString.decode('utf-8'))
                    line_count = line_count + 1
                    print(line_count)
    in_file.close()
out_file.close()
