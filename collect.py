import serial
import csv
import time
import datetime  

COM_PORT = 'COM5'  
BAUD_RATE = 115200  


ser = serial.Serial(COM_PORT, BAUD_RATE)


with open('InActive6March4.csv', 'a', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)

    
    max_duration = 300

    start_time = time.time()

    print("Collecting data...")

    while time.time() - start_time < max_duration:
        
        data = ser.readline().decode("latin-1").strip()

        # Get the current timestamp
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        
        # Split the data into a list of values using the comma as a delimiter
        values = data.split(',')

        if len(values) > 0  and values[0].isdigit():
            # Save the data to the CSV file along with the timestamp
            csvwriter.writerow([current_time, values[0]])

ser.close()
