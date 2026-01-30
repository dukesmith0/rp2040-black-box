"""
Create a suite of 10 KITTI test samples for flight visualization testing.

Each sample:
- Minimum 2 minutes duration (120 seconds = 1200 frames at 10Hz)
- From different segments of KITTI raw data
- Converted to RP2040 black box CSV format

Strategy: Use successfully downloaded drive 0036 (8341 frames = 13.9 minutes)
to create 10 overlapping 2-minute segments with different characteristics.

Usage:
    python create_kitti_test_suite.py

Output:
    test_samples/kitti_sample_01.csv through kitti_sample_10.csv
"""

import urllib.request
import zipfile
import os
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
from pathlib import Path
import shutil

# Base URL for KITTI raw data
KITTI_BASE_URL = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"

# Primary drive to use (large dataset with 8341 frames)
PRIMARY_DRIVE = ("2011_09_26", "0036")

# Minimum frames for 2 minutes at 10Hz
MIN_FRAMES = 1200

# 10 samples configuration - different segments from the 8341 frame dataset
# Format: (name, description, start_frame, end_frame)
# Overlapping segments to maximize variety while ensuring 2+ min each
SAMPLE_CONFIGS = [
    ("kitti_sample_01_start", "Initial segment - vehicle starting", 0, 1500),
    ("kitti_sample_02_early", "Early driving segment", 500, 1900),
    ("kitti_sample_03_morning", "Morning drive section", 1200, 2600),
    ("kitti_sample_04_midway_a", "Midway segment A", 2000, 3500),
    ("kitti_sample_05_midway_b", "Midway segment B", 2800, 4300),
    ("kitti_sample_06_center", "Center of drive", 3500, 5000),
    ("kitti_sample_07_afternoon", "Afternoon section", 4500, 6000),
    ("kitti_sample_08_late", "Late driving segment", 5500, 7000),
    ("kitti_sample_09_final", "Final approach segment", 6500, 8000),
    ("kitti_sample_10_end", "End segment - complete drive", 6841, 8341),
]

EXTRACT_DIR = "kitti_temp"
OUTPUT_DIR = Path("test_samples")
CACHE_DIR = Path("kitti_cache")


def clean_corrupted_cache():
    """Remove any corrupted cached files."""
    if not CACHE_DIR.exists():
        return

    for zip_file in CACHE_DIR.glob("*.zip"):
        try:
            with zipfile.ZipFile(zip_file, 'r') as z:
                # Try to read the file list
                z.namelist()
        except (zipfile.BadZipFile, Exception) as e:
            print(f"Removing corrupted cache file: {zip_file.name}")
            zip_file.unlink()


def download_kitti_drive(date: str, drive: str) -> str | None:
    """Download a KITTI drive if not already cached."""
    CACHE_DIR.mkdir(exist_ok=True)

    zip_filename = f"{date}_drive_{drive}_extract.zip"
    cached_path = CACHE_DIR / zip_filename

    if cached_path.exists():
        # Verify the file is valid
        try:
            with zipfile.ZipFile(cached_path, 'r') as z:
                z.namelist()
            print(f"  Using cached: {zip_filename}")
            return str(cached_path)
        except zipfile.BadZipFile:
            print(f"  Removing corrupted: {zip_filename}")
            cached_path.unlink()

    url = f"{KITTI_BASE_URL}/{date}_drive_{drive}/{date}_drive_{drive}_extract.zip"
    print(f"  Downloading: {zip_filename}")
    print(f"    URL: {url}")

    try:
        urllib.request.urlretrieve(url, str(cached_path))
        print(f"    Download complete!")
        return str(cached_path)
    except Exception as e:
        print(f"    Download failed: {e}")
        if cached_path.exists():
            cached_path.unlink()
        return None


def extract_oxts_data(zip_path: str) -> np.ndarray | None:
    """Extract and load OXTS data from a KITTI zip file."""
    temp_dir = Path(EXTRACT_DIR)

    try:
        with zipfile.ZipFile(zip_path, "r") as z:
            z.extractall(temp_dir)

        # Find oxts data folder
        oxts_path = None
        for root, dirs, files in os.walk(temp_dir):
            if "oxts" in dirs:
                oxts_path = Path(root) / "oxts" / "data"
                break

        if not oxts_path or not oxts_path.exists():
            print(f"    Could not find oxts/data in {zip_path}")
            return None

        # Load all oxts files
        oxts_files = sorted([f for f in os.listdir(oxts_path) if f.endswith(".txt")])

        data = []
        for fname in oxts_files:
            with open(oxts_path / fname, "r") as f:
                values = [float(x) for x in f.read().strip().split()]
                data.append(values)

        return np.array(data)

    finally:
        if temp_dir.exists():
            shutil.rmtree(temp_dir)


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
    """Convert Euler angles (radians) to quaternion [w, x, y, z]."""
    r = Rotation.from_euler("xyz", [roll, pitch, yaw])
    q = r.as_quat()  # [x, y, z, w]
    return (q[3], q[0], q[1], q[2])  # [w, x, y, z]


def convert_oxts_to_blackbox(oxts_data: np.ndarray, start_ms: int = 0, sample_rate_hz: int = 10) -> pd.DataFrame:
    """Convert KITTI oxts data to RP2040 black box CSV format.

    Uses dead-reckoning from IMU velocity for accurate positions since
    KITTI GPS has ~1m precision which can't resolve 10Hz movements.

    Args:
        oxts_data: KITTI oxts data array
        start_ms: Starting timestamp in milliseconds
        sample_rate_hz: Sample rate (KITTI is 10Hz)

    Returns:
        DataFrame in black box format
    """
    rows = []
    dt_ms = int(1000 / sample_rate_hz)
    dt_sec = 1.0 / sample_rate_hz

    # Use first frame's GPS as starting position
    start_lat = oxts_data[0, 0]
    start_lon = oxts_data[0, 1]

    # Dead-reckoning position (in meters from start)
    pos_north = 0.0
    pos_east = 0.0

    for i, oxts in enumerate(oxts_data):
        # KITTI oxts columns:
        # 0:lat, 1:lon, 2:alt, 3:roll, 4:pitch, 5:yaw
        # 6:vn (velocity north), 7:ve (velocity east), 8:vf(forward vel), 9:vl, 10:vu
        # 11:ax, 12:ay, 13:az
        # 17:wx, 18:wy, 19:wz

        alt = oxts[2]
        roll, pitch, yaw = oxts[3], oxts[4], oxts[5]
        vn, ve = oxts[6], oxts[7]  # North and East velocity from IMU
        vf = oxts[8]  # forward velocity (m/s)
        ax, ay, az = oxts[11], oxts[12], oxts[13]
        wx, wy, wz = oxts[17], oxts[18], oxts[19]

        # Dead-reckon position from velocity
        if i > 0:
            pos_north += vn * dt_sec
            pos_east += ve * dt_sec

        # Convert position to lat/lon
        lat = start_lat + (pos_north / 111000.0)
        lon = start_lon + (pos_east / (111000.0 * np.cos(np.radians(start_lat))))

        # Convert to our format
        qw, qx, qy, qz = euler_to_quaternion(roll, pitch, yaw)

        # Speed in knots
        speed_knots = vf * 1.94384

        # Heading in degrees (KITTI yaw: 0=east, CCW positive)
        # Convert to compass heading (0=north, CW positive)
        heading_deg = (90 - np.degrees(yaw)) % 360

        # Magnetometer - simulate based on heading
        mag_strength = 50  # ~50 uT typical
        mx = mag_strength * np.cos(np.radians(heading_deg))
        my = -mag_strength * np.sin(np.radians(heading_deg))
        mz = 30  # vertical component

        row = {
            "ms": start_ms + (i * dt_ms),
            "qw": round(qw, 4),
            "qx": round(qx, 4),
            "qy": round(qy, 4),
            "qz": round(qz, 4),
            "ax": round(ax, 2),
            "ay": round(ay, 2),
            "az": round(az, 2),  # KITTI includes gravity in accelerometer readings
            "gx": round(wx, 4),
            "gy": round(wy, 4),
            "gz": round(wz, 4),
            "mx": round(mx, 2),
            "my": round(my, 2),
            "mz": round(mz, 2),
            "alt": round(alt, 2),
            "gps_fix": 1,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "speed": round(speed_knots, 2),
            "heading": round(heading_deg, 2),
        }
        rows.append(row)

    return pd.DataFrame(rows)


def create_samples_from_drive(oxts_data: np.ndarray) -> int:
    """Create all 10 samples from a single large KITTI drive."""
    OUTPUT_DIR.mkdir(exist_ok=True)

    total_frames = len(oxts_data)
    print(f"\nTotal available frames: {total_frames}")
    print(f"Total duration: {total_frames / 10:.1f} seconds ({total_frames / 600:.1f} minutes)")

    successful = 0

    for name, description, start_idx, end_idx in SAMPLE_CONFIGS:
        print(f"\n{'='*60}")
        print(f"Creating: {name}")
        print(f"Description: {description}")
        print(f"Frames: {start_idx} to {end_idx}")
        print(f"{'='*60}")

        # Clamp to available data
        actual_start = min(start_idx, total_frames)
        actual_end = min(end_idx, total_frames)

        if actual_end - actual_start < MIN_FRAMES:
            print(f"WARNING: Segment too short ({actual_end - actual_start} frames)")
            # Adjust to get minimum frames
            if actual_start + MIN_FRAMES <= total_frames:
                actual_end = actual_start + MIN_FRAMES
            elif total_frames >= MIN_FRAMES:
                actual_start = total_frames - MIN_FRAMES
                actual_end = total_frames
            else:
                print(f"SKIPPING: Not enough data")
                continue

        segment = oxts_data[actual_start:actual_end]
        duration_sec = len(segment) / 10

        print(f"Using frames {actual_start} to {actual_end} ({len(segment)} frames)")
        print(f"Duration: {duration_sec:.1f} seconds ({duration_sec/60:.1f} minutes)")

        # Convert to black box format
        df = convert_oxts_to_blackbox(segment)

        # Save
        output_path = OUTPUT_DIR / f"{name}.csv"
        df.to_csv(output_path, index=False)

        print(f"Saved: {output_path}")
        print(f"GPS bounds: Lat [{df['lat'].min():.6f}, {df['lat'].max():.6f}]")
        print(f"            Lon [{df['lon'].min():.6f}, {df['lon'].max():.6f}]")
        print(f"Speed range: {df['speed'].min():.1f} to {df['speed'].max():.1f} knots")
        print(f"Altitude range: {df['alt'].min():.1f} to {df['alt'].max():.1f} m")

        successful += 1

    return successful


def main():
    print("=" * 70)
    print("KITTI Test Suite Generator")
    print("Creating 10 test samples, each with minimum 2 minutes duration")
    print("=" * 70)

    # Clean up any corrupted cached files
    print("\nChecking cache for corrupted files...")
    clean_corrupted_cache()

    # Download primary drive
    date, drive = PRIMARY_DRIVE
    print(f"\nDownloading primary drive: {date}_drive_{drive}")

    zip_path = download_kitti_drive(date, drive)
    if not zip_path:
        print("\nFailed to download primary drive!")
        print("Please check your internet connection and try again.")
        return

    # Extract OXTS data
    print("\nExtracting OXTS data...")
    oxts_data = extract_oxts_data(zip_path)
    if oxts_data is None:
        print("Failed to extract data!")
        return

    print(f"Loaded {len(oxts_data)} frames from KITTI drive")

    # Create all samples
    successful = create_samples_from_drive(oxts_data)

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    print(f"Successfully created: {successful}/10 samples")
    print(f"Output directory: {OUTPUT_DIR.absolute()}")

    if OUTPUT_DIR.exists():
        files = list(OUTPUT_DIR.glob("kitti_sample_*.csv"))
        print(f"\nGenerated files:")
        total_size = 0
        for f in sorted(files):
            size_kb = f.stat().st_size / 1024
            total_size += size_kb
            df = pd.read_csv(f)
            duration = df["ms"].max() / 1000
            print(f"  {f.name}: {size_kb:.1f} KB, {len(df)} samples, {duration:.1f}s ({duration/60:.1f}m)")
        print(f"\nTotal size: {total_size:.1f} KB ({total_size/1024:.2f} MB)")


if __name__ == "__main__":
    main()
