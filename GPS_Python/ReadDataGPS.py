import serial
import time

# Configure the serial connection to the GPS module
ser = serial.Serial(
    # for linux
    # port='/dev/ttyUSB0',  # Update this to your GPS module's serial port
    # for windows 
    port = 'COM4',
    baudrate=9600        # GPS modules commonly use 9600 or 115200 baud
    # timeout=1             # 1 second timeout for serial communication
)

def read_gps():
    while True:
        if ser.in_waiting > 0:
            # Read data from the GPS module
            gps_data = ser.readline().decode('utf-8').strip()
            
            # Only process lines starting with $G, which are NMEA sentences
            if gps_data.startswith('$G'):
                # print(f"Received GPS data: {gps_data}")
                parse_nmea(gps_data)
            # time.sleep(1)

def parse_nmea(nmea_sentence):
    # Example: Parse a GGA sentence (for latitude, longitude, altitude)
    if nmea_sentence.startswith('$GNGGA') or nmea_sentence.startswith('$GPGGA'):
        parts = nmea_sentence.split(',')
        
        if len(parts) < 15:
            print("Incomplete GGA data")
            return
        
        time_utc = parts[1]      # UTC time
        latitude = parts[2]      # Latitude
        lat_dir = parts[3]       # N/S direction
        longitude = parts[4]     # Longitude
        long_dir = parts[5]      # E/W direction
        fix_quality = parts[6]   # Fix quality (0 = invalid, 1 = GPS fix, 2 = DGPS fix)
        num_sats = parts[7]      # Number of satellites being tracked
        altitude = parts[9]      # Altitude above mean sea level
        
        print(f"Time: {time_utc}, Latitude: {latitude} {lat_dir}, "
              f"Longitude: {longitude} {long_dir}, Altitude: {altitude} meters, "
              f"Satellites: {num_sats}, Fix: {fix_quality}")
        # print(f"Latitude: {latitude:f} {lat_dir}, Longitude: {longitude:f} {long_dir}")



if __name__ == "__main__":
    try:
        read_gps()
    except KeyboardInterrupt:
        ser.close()
        print("GPS data reading stopped.")
