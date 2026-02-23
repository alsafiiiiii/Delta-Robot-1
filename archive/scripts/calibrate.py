import serial
import time
import csv
import numpy as np
import sys

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your port (e.g., COM3 on Windows)
BAUD_RATE = 115200            # Make sure this matches your 'idf.py monitor' baud rate
SAMPLES_PER_POINT = 50        # How many readings to average per distance
FILENAME = 'sensor_data.csv'

def get_valid_reading(ser):
    """Reads lines until a valid integer is found."""
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            # Depending on your log output, you might get "I (123) Tag: ..."
            # But with your printf("%d\n"), it should just be numbers.
            val = int(line)
            return val
        except ValueError:
            continue # Ignore garbage lines (logs, startup text)

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException:
        print(f"Error: Could not open {SERIAL_PORT}. Check connection/permissions.")
        return

    data_points = [] # List of (voltage, distance)

    print("\n--- SHARP SENSOR CALIBRATION TOOL ---")
    print("1. Place object at a known distance.")
    print("2. Enter the distance in mm.")
    print("3. Wait for sampling.")
    print("4. Repeat for as many points as possible (100mm, 150mm, 200mm...).")
    print("Type 'q' or 'done' to finish and generate code.\n")

    # Clear buffer
    ser.reset_input_buffer()

    while True:
        user_input = input("Enter Distance (mm): ").strip().lower()
        
        if user_input in ['q', 'quit', 'exit', 'done']:
            break
        
        try:
            dist_mm = float(user_input)
        except ValueError:
            print("Invalid number. Try again.")
            continue

        print(f"Sampling {SAMPLES_PER_POINT} readings... keep sensor still.")
        
        readings = []
        ser.reset_input_buffer() # Clear old data
        
        while len(readings) < SAMPLES_PER_POINT:
            val = get_valid_reading(ser)
            readings.append(val)
            # Simple progress bar
            sys.stdout.write(f"\rProgress: {len(readings)}/{SAMPLES_PER_POINT} | Latest: {val}mV")
            sys.stdout.flush()
        
        avg_mv = np.mean(readings)
        std_mv = np.std(readings)
        
        print(f"\nCaptured: {dist_mm}mm = {avg_mv:.1f} mV (Noise: ±{std_mv:.1f})")
        
        data_points.append((avg_mv, dist_mm))
        
        # Save immediately to CSV just in case
        with open(FILENAME, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Voltage_mV", "Distance_mm"])
            writer.writerows(data_points)

    # --- CALCULATION PHASE ---
    if len(data_points) < 3:
        print("\nNot enough points to generate a curve. Need at least 3.")
        return

    print("\n\n--- GENERATING POLYNOMIAL ---")
    
    # Extract X (Voltage) and Y (Distance)
    voltages = np.array([p[0] for p in data_points])
    distances = np.array([p[1] for p in data_points])

    # Fit a 4th Degree Polynomial (Distance = f(Voltage))
    # We want Distance = A*V^4 + B*V^3 ...
    coeffs = np.polyfit(voltages, distances, 4)
    
    # coeffs returns [c4, c3, c2, c1, c0] (High power first)
    c4, c3, c2, c1, c0 = coeffs

    print("Success! Copy and paste this function into your C code:\n")
    print("-" * 60)
    
    code_snippet = f"""
// Auto-generated calibration from Python
// Points used: {len(data_points)}
static float poly_calculate_mm(double v) {{
    // v is in mV (e.g. 2150.0), but polyfit might be large.
    // Let's keep it simple: v is raw MILLIVOLTS in this formula.
    
    double x = v; 
    
    return (float)(({c4:.5e} * x * x * x * x) + 
                   ({c3:.5e} * x * x * x) + 
                   ({c2:.5e} * x * x) + 
                   ({c1:.5e} * x) + 
                   {c0:.5f});
}}
"""
    print(code_snippet)
    print("-" * 60)
    print(f"Data saved to {FILENAME}")

if __name__ == "__main__":
    main()