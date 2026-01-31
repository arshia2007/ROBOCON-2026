import serial
import csv
import datetime
import os

# Configuration
SERIAL_PORT = 'COM8'  # Change to your port
BAUD_RATE = 9600
OUTPUT_FOLDER = 'robot_logs'

# Create output folder
os.makedirs(OUTPUT_FOLDER, exist_ok=True)

# Generate filename with timestamp
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
csv_filename = os.path.join(OUTPUT_FOLDER, f"robot_data_{timestamp}.csv")

print(f"Logging to: {csv_filename}")
print("Press Ctrl+C to stop\n")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = None
        line_count = 0
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line:
                    data = line.split(',')
                    
                    if csv_writer is None:
                        # Write header
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow(data)
                        csvfile.flush()
                        print(f"Header: {', '.join(data[:5])}...\n")
                    else:
                        # Write data
                        csv_writer.writerow(data)
                        line_count += 1
                        
                        if line_count % 10 == 0:
                            csvfile.flush()
                            print(f"Logged {line_count} rows")
                            
except KeyboardInterrupt:
    print(f"\nStopped. Total rows: {line_count}")
    print(f"Saved: {csv_filename}")
    
except serial.SerialException:
    print(f"\nERROR: Cannot open {SERIAL_PORT}")
    print("Check: Port name, USB connection, Serial Monitor closed")
    
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()