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
outfile = open('data4test_03_out_msp.csv', mode='w')
#csv_writer = csv.writer(out_file, delimiter=',')

with open('data4test_03_in.csv') as infile:
    csv_reader = csv.reader(infile, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count > 0:
            # print(row[13])
            str1 = ','.join(row)
            str1 = str1 + '\n'
            ser.write(str1.encode())

            rxString = ser.readline()
            # csv_writer.writerow([rxString.decode('utf-8')])
            # HeadsUp: Windows and Python3 interpret things in a weird way.
            # decode to UTF-8 first, strip the newline
            # write already adds a carriage return (in Windows)
            temp_string = rxString.decode().rstrip('\n')
            outfile.write(temp_string)
        line_count = line_count + 1
        print(line_count)

infile.close()
outfile.close()
