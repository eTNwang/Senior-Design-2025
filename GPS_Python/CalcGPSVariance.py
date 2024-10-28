import serial
import time
import matplotlib.pyplot as plt
import numpy as np

# Configure the serial connection to the GPS module
ser = serial.Serial(
    # Update port based on OS
    # port='/dev/ttyUSB0',  # Linux
    port='COM4',           # Windows
    baudrate=9600,         # Common GPS baud rates are 9600 or 115200
    timeout=1              # Timeout for serial communication
)

# Define origin for converting GPS to local coordinates
origin_lat = None
origin_lon = None
gps_points = []

# Conversion factors
LAT_TO_METERS = 111139.0 * 100 # Approximate meters per degree latitude to CM
LON_TO_METERS = 111139.0 * 100 # Will be recalculated based on latitude

def read_gps():
    while True:
        if ser.in_waiting > 0:
            # Read data from the GPS module
            gps_data = ser.readline().decode('utf-8').strip()
            
            # Only process lines starting with $G (NMEA sentences)
            if gps_data.startswith('$G'):
                parse_nmea(gps_data)

def parse_nmea(nmea_sentence):
    global origin_lat, origin_lon, gps_points, LON_TO_METERS
    
    if nmea_sentence.startswith('$GNGGA') or nmea_sentence.startswith('$GPGGA'):
        parts = nmea_sentence.split(',')
        
        if len(parts) < 15:
            print("Incomplete GGA data")
            return
        
        # Parse latitude and longitude
        latitude = convert_to_decimal(parts[2], parts[3])
        longitude = convert_to_decimal(parts[4], parts[5])
        
        # Initialize the origin to calculate relative positions
        if origin_lat is None or origin_lon is None:
            origin_lat = latitude
            origin_lon = longitude
            LON_TO_METERS = LAT_TO_METERS * np.cos(np.radians(origin_lat))
            print("Origin set:", origin_lat, origin_lon)
        
        # Convert latitude and longitude to local coordinates (x, y)
        x = (longitude - origin_lon) * LON_TO_METERS
        y = (latitude - origin_lat) * LAT_TO_METERS
        
        # Store the point
        gps_points.append((x, y))
        
        # Plot and compute variance
        plot_and_calculate_variance()

def convert_to_decimal(value, direction):
    """Convert NMEA format (degrees and minutes) to decimal degrees."""
    if value == '':
        return None
    degrees = int(value[:2])
    minutes = float(value[2:]) / 60
    decimal = degrees + minutes
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

def plot_and_calculate_variance():
    """Plot GPS points and calculate variance."""
    if len(gps_points) < 2:
        return
    
    # Separate x and y coordinates
    x_vals, y_vals = zip(*gps_points)
    
    # Plot the points
    plt.clf()
    plt.scatter(x_vals, y_vals, c='blue', marker='o')
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("Real-time GPS Tracking")
    plt.pause(0.1)
    
    # Calculate variance
    x_variance = np.var(x_vals)
    y_variance = np.var(y_vals)
    print(f"Variance - X: {x_variance:.2f} meters^2, Y: {y_variance:.2f} meters^2")

if __name__ == "__main__":
    # Initialize real-time plotting
    plt.ion()
    
    try:
        read_gps()
    except KeyboardInterrupt:
        ser.close()
        print("GPS data reading stopped.")
        plt.ioff()
        plt.show()
