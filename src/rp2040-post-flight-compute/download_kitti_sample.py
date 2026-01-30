"""
Download and convert KITTI driving data to match RP2040 black box CSV format.

KITTI oxts format (30 columns):
  lat, lon, alt, roll, pitch, yaw, vn, ve, vf, vl, vu,
  ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu,
  posacc, velacc, navstat, numsats, posmode, velmode, orimode

Our format (20 columns):
  ms, qw, qx, qy, qz, ax, ay, az, gx, gy, gz, mx, my, mz, alt, gps_fix, lat, lon, speed, heading

Usage:
  python download_kitti_sample.py

This will create 'kitti_driving_sample.csv' in the current directory.
"""

import urllib.request
import zipfile
import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import shutil

# KITTI raw data sample URL (2011_09_26_drive_0001)
KITTI_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0001/2011_09_26_drive_0001_extract.zip"
EXTRACT_DIR = "kitti_temp"
OUTPUT_FILE = "kitti_driving_sample.csv"


def download_kitti():
    """Download KITTI sample if not already present."""
    zip_file = "kitti_sample.zip"

    if os.path.exists(zip_file):
        print(f"Using cached {zip_file}")
        return zip_file

    print(f"Downloading KITTI sample data (~25MB)...")
    print(f"URL: {KITTI_URL}")

    try:
        urllib.request.urlretrieve(KITTI_URL, zip_file)
        print("Download complete!")
        return zip_file
    except Exception as e:
        print(f"Download failed: {e}")
        print("\nManual download instructions:")
        print(f"1. Go to https://www.cvlibs.net/datasets/kitti/raw_data.php")
        print(f"2. Register and download any 'unsynced+unrectified data' drive")
        print(f"3. Extract and point this script to the oxts/data/ folder")
        return None


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (radians) to quaternion [w, x, y, z]."""
    r = Rotation.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # [x, y, z, w]
    return [q[3], q[0], q[1], q[2]]  # [w, x, y, z]


def load_kitti_oxts(folder_path):
    """Load all oxts files from KITTI extract."""
    oxts_files = sorted([f for f in os.listdir(folder_path) if f.endswith('.txt')])

    data = []
    for fname in oxts_files:
        with open(os.path.join(folder_path, fname), 'r') as f:
            values = [float(x) for x in f.read().strip().split()]
            data.append(values)

    return np.array(data)


def convert_to_blackbox_format(oxts_data, sample_rate_hz=100):
    """Convert KITTI oxts data to RP2040 black box CSV format."""

    rows = []
    for i, oxts in enumerate(oxts_data):
        # KITTI oxts columns (0-indexed):
        # 0:lat, 1:lon, 2:alt, 3:roll, 4:pitch, 5:yaw
        # 6:vn, 7:ve, 8:vf(forward vel), 9:vl, 10:vu
        # 11:ax, 12:ay, 13:az, 14:af, 15:al, 16:au
        # 17:wx, 18:wy, 19:wz

        lat, lon, alt = oxts[0], oxts[1], oxts[2]
        roll, pitch, yaw = oxts[3], oxts[4], oxts[5]
        vf = oxts[8]  # forward velocity (m/s)
        ax, ay, az = oxts[11], oxts[12], oxts[13]
        wx, wy, wz = oxts[17], oxts[18], oxts[19]

        # Convert to our format
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

        # Speed in knots (KITTI is m/s, GPS typically reports knots)
        speed_knots = vf * 1.94384

        # Heading in degrees (KITTI yaw: 0=east, CCW positive)
        # Convert to compass heading (0=north, CW positive)
        heading_deg = (90 - np.degrees(yaw)) % 360

        # Magnetometer - simulate based on heading (simplified)
        # Real values would come from actual sensor
        mag_strength = 50  # ~50 uT typical
        mx = mag_strength * np.cos(np.radians(heading_deg))
        my = -mag_strength * np.sin(np.radians(heading_deg))
        mz = 30  # vertical component

        row = {
            'ms': int(i * (1000 / sample_rate_hz)),  # 10ms intervals for 100Hz
            'qw': round(qw, 4),
            'qx': round(qx, 4),
            'qy': round(qy, 4),
            'qz': round(qz, 4),
            'ax': round(ax, 2),
            'ay': round(ay, 2),
            'az': round(az + 9.81, 2),  # Add gravity (KITTI removes it)
            'gx': round(wx, 4),
            'gy': round(wy, 4),
            'gz': round(wz, 4),
            'mx': round(mx, 2),
            'my': round(my, 2),
            'mz': round(mz, 2),
            'alt': round(alt, 2),
            'gps_fix': 1,  # KITTI always has GPS
            'lat': round(lat, 6),
            'lon': round(lon, 6),
            'speed': round(speed_knots, 2),
            'heading': round(heading_deg, 2),
        }
        rows.append(row)

    return pd.DataFrame(rows)


def main():
    print("=== KITTI to RP2040 Black Box Format Converter ===\n")

    # Try to download
    zip_file = download_kitti()

    if zip_file and os.path.exists(zip_file):
        print(f"\nExtracting {zip_file}...")
        with zipfile.ZipFile(zip_file, 'r') as z:
            z.extractall(EXTRACT_DIR)

        # Find oxts data folder
        oxts_path = None
        for root, dirs, files in os.walk(EXTRACT_DIR):
            if 'oxts' in dirs:
                oxts_path = os.path.join(root, 'oxts', 'data')
                break

        if oxts_path and os.path.exists(oxts_path):
            print(f"Found OXTS data at: {oxts_path}")

            # Load and convert
            print("Loading OXTS files...")
            oxts_data = load_kitti_oxts(oxts_path)
            print(f"Loaded {len(oxts_data)} frames")

            print("Converting to black box format...")
            df = convert_to_blackbox_format(oxts_data)

            # Save
            df.to_csv(OUTPUT_FILE, index=False)
            print(f"\nSaved: {OUTPUT_FILE}")
            print(f"Shape: {df.shape}")
            print(f"\nFirst few rows:")
            print(df.head())

            print(f"\nGPS track bounds:")
            print(f"  Lat: {df['lat'].min():.6f} to {df['lat'].max():.6f}")
            print(f"  Lon: {df['lon'].min():.6f} to {df['lon'].max():.6f}")

            # Cleanup
            shutil.rmtree(EXTRACT_DIR)
            print(f"\nCleaned up temp files.")
        else:
            print(f"Could not find oxts/data folder in extract")
    else:
        print("\nCreating synthetic driving sample instead...")
        create_synthetic_sample()


def create_synthetic_sample():
    """Create synthetic driving data if KITTI download fails."""
    print("Generating synthetic driving route...")

    # Simulate a simple drive around a block
    # Start point: arbitrary location
    start_lat, start_lon = 40.7128, -74.0060  # NYC

    n_samples = 1000
    dt = 0.01  # 100Hz

    rows = []
    lat, lon = start_lat, start_lon
    heading = 0  # North
    speed = 0

    for i in range(n_samples):
        t = i * dt

        # Simple acceleration/deceleration pattern
        if t < 2:
            speed = min(speed + 0.5, 15)  # Accelerate to 15 m/s (~33 mph)
        elif t > 8:
            speed = max(speed - 0.5, 0)

        # Turn at t=4s
        if 4 < t < 5:
            heading += 0.9  # Turn right ~90 degrees over 1 second

        # Update position (simple dead reckoning)
        lat += (speed * np.cos(np.radians(heading)) * dt) / 111000
        lon += (speed * np.sin(np.radians(heading)) * dt) / (111000 * np.cos(np.radians(lat)))

        # Quaternion from heading
        qw, qx, qy, qz = euler_to_quaternion(0, 0, np.radians(heading))

        row = {
            'ms': int(i * 10),
            'qw': round(qw, 4), 'qx': round(qx, 4), 'qy': round(qy, 4), 'qz': round(qz, 4),
            'ax': round(np.random.normal(0, 0.1), 2),
            'ay': round(np.random.normal(0, 0.1), 2),
            'az': round(9.81 + np.random.normal(0, 0.05), 2),
            'gx': round(np.random.normal(0, 0.01), 4),
            'gy': round(np.random.normal(0, 0.01), 4),
            'gz': round(np.random.normal(0, 0.01), 4),
            'mx': round(50 * np.cos(np.radians(heading)), 2),
            'my': round(-50 * np.sin(np.radians(heading)), 2),
            'mz': 30,
            'alt': 10.0,
            'gps_fix': 1,
            'lat': round(lat, 6),
            'lon': round(lon, 6),
            'speed': round(speed * 1.94384, 2),  # knots
            'heading': round(heading % 360, 2),
        }
        rows.append(row)

    df = pd.DataFrame(rows)
    df.to_csv(OUTPUT_FILE, index=False)
    print(f"Saved: {OUTPUT_FILE}")
    print(df.head())


if __name__ == "__main__":
    main()
