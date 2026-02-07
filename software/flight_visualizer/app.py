"""
RP2040 Black Box - Flight Data Visualizer

Streamlit app for visualizing GPS tracks and sensor data from the RP2040 data logger.

Usage:
    streamlit run app.py
"""

import streamlit as st
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path

# --- Configuration ---
st.set_page_config(
    page_title="Flight Visualizer",
    page_icon="🛩️", # Incredible Emoji
    layout="wide"
)

# Clean styling
st.markdown("""
<style>
    .block-container {
        padding-top: 2rem;
        padding-bottom: 2rem;
    }
    h1 {
        font-size: 1.8rem;
        margin-bottom: 0.5rem;
    }
</style>
""", unsafe_allow_html=True)

# --- Sample Data ---
# Build path to /data/samples directory
SAMPLES_DIR = Path(__file__).resolve().parent.parent.parent / "data" / "samples"
SAMPLES = {
    "None": None,
    # KITTI Test Suite (10 samples, 2+ minutes each)
    "KITTI 01: Start (2.5m)": SAMPLES_DIR / "kitti_sample_01_start.csv",
    "KITTI 02: Early (2.3m)": SAMPLES_DIR / "kitti_sample_02_early.csv",
    "KITTI 03: Morning (2.3m)": SAMPLES_DIR / "kitti_sample_03_morning.csv",
    "KITTI 04: Midway A (2.5m)": SAMPLES_DIR / "kitti_sample_04_midway_a.csv",
    "KITTI 05: Midway B (2.5m)": SAMPLES_DIR / "kitti_sample_05_midway_b.csv",
    "KITTI 06: Center (2.5m)": SAMPLES_DIR / "kitti_sample_06_center.csv",
    "KITTI 07: Afternoon (2.5m)": SAMPLES_DIR / "kitti_sample_07_afternoon.csv",
    "KITTI 08: Late (2.5m)": SAMPLES_DIR / "kitti_sample_08_late.csv",
    "KITTI 09: Final (2.5m)": SAMPLES_DIR / "kitti_sample_09_final.csv",
    "KITTI 10: End (2.5m)": SAMPLES_DIR / "kitti_sample_10_end.csv",
}


def load_csv(file) -> pd.DataFrame:
    """Load and validate CSV data."""
    df = pd.read_csv(file, comment='#')

    # Only ms is required
    if 'ms' not in df.columns:
        st.error("Missing required column: ms")
        return None

    # Filter invalid GPS if GPS columns exist
    has_gps = 'lat' in df.columns and 'lon' in df.columns
    if has_gps:
        if 'gps_fix' in df.columns:
            df = df[df['gps_fix'] == 1]
        df = df[(df['lat'] != 0) & (df['lon'] != 0)]

    return df


def create_map(df: pd.DataFrame, color_by: str = 'speed') -> go.Figure:
    """Create GPS track map."""
    color_data = df[color_by] if color_by in df.columns else df['ms'] / 1000
    color_label = {'speed': 'Speed (kts)', 'alt': 'Altitude (m)',
                   'heading': 'Heading (°)', 'ms': 'Time (s)'}.get(color_by, color_by)

    fig = go.Figure()

    # Track line
    fig.add_trace(go.Scattermapbox(
        lat=df['lat'], lon=df['lon'],
        mode='lines+markers',
        marker=dict(size=5, color=color_data, colorscale='Viridis',
                    showscale=True, colorbar=dict(title=color_label, thickness=15)),
        line=dict(width=2),
        hovertemplate='%{lat:.5f}, %{lon:.5f}<extra></extra>',
        name='Track'
    ))

    # Start/end markers
    fig.add_trace(go.Scattermapbox(
        lat=[df['lat'].iloc[0]], lon=[df['lon'].iloc[0]],
        mode='markers', marker=dict(size=12, color='#22c55e'),
        name='Start', hoverinfo='name'
    ))
    fig.add_trace(go.Scattermapbox(
        lat=[df['lat'].iloc[-1]], lon=[df['lon'].iloc[-1]],
        mode='markers', marker=dict(size=12, color='#ef4444'),
        name='End', hoverinfo='name'
    ))

    # Auto zoom
    lat_range = df['lat'].max() - df['lat'].min()
    lon_range = df['lon'].max() - df['lon'].min()
    max_range = max(lat_range, lon_range)
    zoom = 17 if max_range < 0.001 else 14 if max_range < 0.01 else 11 if max_range < 0.1 else 9

    fig.update_layout(
        mapbox=dict(style='open-street-map',
                    center=dict(lat=df['lat'].mean(), lon=df['lon'].mean()),
                    zoom=zoom),
        margin=dict(l=0, r=0, t=0, b=0),
        height=450,
        showlegend=False
    )
    return fig


def create_3d_path(df: pd.DataFrame) -> go.Figure:
    """Create 3D trajectory visualization."""
    if 'alt' not in df.columns or 'lat' not in df.columns or 'lon' not in df.columns:
        return None

    # Convert to meters from start
    lat_m = (df['lat'] - df['lat'].iloc[0]) * 111000
    lon_m = (df['lon'] - df['lon'].iloc[0]) * 111000 * np.cos(np.radians(df['lat'].mean()))
    alt_m = df['alt'] - df['alt'].iloc[0]

    fig = go.Figure()
    fig.add_trace(go.Scatter3d(
        x=lat_m, y=lon_m, z=alt_m,
        mode='lines',
        line=dict(color=df['ms']/1000, colorscale='Viridis', width=4,
                  showscale=True, colorbar=dict(title='Time (s)', thickness=15)),
        hovertemplate='N: %{x:.1f}m<br>E: %{y:.1f}m<br>Alt: %{z:.1f}m<extra></extra>'
    ))

    # Start/end
    fig.add_trace(go.Scatter3d(x=[lat_m.iloc[0]], y=[lon_m.iloc[0]], z=[alt_m.iloc[0]],
                                mode='markers', marker=dict(size=6, color='#22c55e'), name='Start'))
    fig.add_trace(go.Scatter3d(x=[lat_m.iloc[-1]], y=[lon_m.iloc[-1]], z=[alt_m.iloc[-1]],
                                mode='markers', marker=dict(size=6, color='#ef4444'), name='End'))

    fig.update_layout(
        scene=dict(xaxis_title='North (m)', yaxis_title='East (m)', zaxis_title='Alt (m)',
                   aspectmode='data'),
        height=400,
        margin=dict(l=0, r=0, t=30, b=0),
        showlegend=False
    )
    return fig


def create_sensor_charts(df: pd.DataFrame) -> go.Figure:
    """Create sensor data time series."""
    t = df['ms'] / 1000

    fig = make_subplots(rows=2, cols=2, vertical_spacing=0.15, horizontal_spacing=0.1,
                        subplot_titles=('Speed & Altitude', 'Orientation (Quaternion)',
                                       'Accelerometer', 'Gyroscope'))

    # Speed & Altitude
    if 'speed' in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df['speed'], name='Speed (kts)',
                                  legendgroup='speed_alt', legendgrouptitle_text='Speed & Alt',
                                  line=dict(color='#3b82f6', width=1)), row=1, col=1)
    if 'alt' in df.columns:
        fig.add_trace(go.Scatter(x=t, y=df['alt'], name='Altitude (m)',
                                  legendgroup='speed_alt',
                                  line=dict(color='#22c55e', width=1)), row=1, col=1)

    # Quaternion
    quat_colors = {'qw': ('#6b7280', 'w'), 'qx': ('#ef4444', 'x'), 'qy': ('#22c55e', 'y'), 'qz': ('#3b82f6', 'z')}
    for col, (color, label) in quat_colors.items():
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=label,
                                      legendgroup='quat', legendgrouptitle_text='Quaternion',
                                      line=dict(color=color, width=1)), row=1, col=2)

    # Accelerometer
    accel_colors = {'ax': ('#ef4444', 'X'), 'ay': ('#22c55e', 'Y'), 'az': ('#3b82f6', 'Z')}
    for col, (color, label) in accel_colors.items():
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=label,
                                      legendgroup='accel', legendgrouptitle_text='Accel (m/s²)',
                                      line=dict(color=color, width=1)), row=2, col=1)

    # Gyroscope
    gyro_colors = {'gx': ('#ef4444', 'X'), 'gy': ('#22c55e', 'Y'), 'gz': ('#3b82f6', 'Z')}
    for col, (color, label) in gyro_colors.items():
        if col in df.columns:
            fig.add_trace(go.Scatter(x=t, y=df[col], name=label,
                                      legendgroup='gyro', legendgrouptitle_text='Gyro (rad/s)',
                                      line=dict(color=color, width=1)), row=2, col=2)

    fig.update_layout(
        height=550,
        showlegend=True,
        legend=dict(
            orientation='h',
            yanchor='top',
            y=-0.25,
            xanchor='center',
            x=0.5,
            font=dict(size=10),
            groupclick='toggleitem'
        ),
        margin=dict(l=50, r=20, t=40, b=120)
    )
    fig.update_xaxes(title_text='Time (s)', row=2)
    fig.update_yaxes(title_text='kts / m', row=1, col=1)
    fig.update_yaxes(title_text='q', row=1, col=2)
    fig.update_yaxes(title_text='m/s²', row=2, col=1)
    fig.update_yaxes(title_text='rad/s', row=2, col=2)

    return fig


def compute_stats(df: pd.DataFrame) -> dict:
    """Compute summary statistics."""
    stats = {
        'duration': (df['ms'].max() - df['ms'].min()) / 1000,
        'samples': len(df),
    }

    # Distance (only if GPS data exists)
    has_gps = 'lat' in df.columns and 'lon' in df.columns
    if has_gps and len(df) > 1:
        lat_diff = np.diff(df['lat']) * 111000
        lon_diff = np.diff(df['lon']) * 111000 * np.cos(np.radians(df['lat'].mean()))
        stats['distance'] = np.sqrt(lat_diff**2 + lon_diff**2).sum()

    if 'speed' in df.columns:
        stats['avg_speed'] = df['speed'].mean()
        stats['max_speed'] = df['speed'].max()

    if 'alt' in df.columns:
        stats['alt_gain'] = df['alt'].max() - df['alt'].min()

    return stats


def main():
    st.title("Flight Data Visualizer")

    # Sidebar
    with st.sidebar:
        st.subheader("Data Source")

        uploaded_file = st.file_uploader("Upload CSV", type=['csv'])

        # Sample data dropdown
        available_samples = {k: v for k, v in SAMPLES.items() if v is None or v.exists()}
        sample_choice = st.selectbox("Or load sample data", options=list(available_samples.keys()))

        st.divider()
        st.subheader("Display Options")
        color_by = st.selectbox("Color track by", ['speed', 'alt', 'heading', 'ms'])
        show_3d = st.checkbox("Show 3D view", value=True)
        show_sensors = st.checkbox("Show sensor charts", value=True)

    # Determine data source
    data_source = None
    if uploaded_file is not None:
        data_source = uploaded_file
    elif sample_choice != "None" and SAMPLES[sample_choice]:
        data_source = str(SAMPLES[sample_choice])

    # Main content
    if data_source:
        df = load_csv(data_source)

        if df is not None and len(df) > 0:
            stats = compute_stats(df)
            has_gps = 'lat' in df.columns and 'lon' in df.columns

            # Metrics row
            cols = st.columns(5)
            cols[0].metric("Duration", f"{stats['duration']:.1f}s")
            cols[1].metric("Samples", f"{stats['samples']:,}")
            cols[2].metric("Distance", f"{stats.get('distance', 0):.0f}m" if has_gps else "N/A")
            cols[3].metric("Avg Speed", f"{stats.get('avg_speed', 0):.1f} kts" if 'avg_speed' in stats else "N/A")
            cols[4].metric("Alt Gain", f"{stats.get('alt_gain', 0):.1f}m" if 'alt_gain' in stats else "N/A")

            st.divider()

            # GPS visualizations (only if GPS data exists)
            if has_gps:
                if show_3d and 'alt' in df.columns:
                    col1, col2 = st.columns([3, 2])
                    with col1:
                        st.subheader("GPS Track")
                        st.plotly_chart(create_map(df, color_by), use_container_width=True)
                    with col2:
                        st.subheader("3D Path")
                        fig_3d = create_3d_path(df)
                        if fig_3d:
                            st.plotly_chart(fig_3d, use_container_width=True)
                else:
                    st.subheader("GPS Track")
                    st.plotly_chart(create_map(df, color_by), use_container_width=True)
            else:
                st.info("No GPS data (lat/lon columns) found. Showing sensor data only.")

            # Sensor charts
            if show_sensors:
                st.divider()
                st.subheader("Sensor Data")
                st.plotly_chart(create_sensor_charts(df), use_container_width=True)

            # Raw data
            with st.expander("View raw data"):
                st.dataframe(df, use_container_width=True, height=300)
        else:
            st.warning("No valid data found in file.")
    else:
        st.info("Upload a CSV file or select sample data from the sidebar to begin.")

        with st.expander("Expected CSV format"):
            st.markdown("""
            | Column | Required | Description |
            |--------|----------|-------------|
            | `ms` | Yes | Timestamp (milliseconds) |
            | `lat`, `lon` | No | GPS coordinates (decimal degrees) |
            | `alt` | No | Altitude (meters) |
            | `speed` | No | Speed (knots) |
            | `heading` | No | Heading (degrees) |
            | `qw`, `qx`, `qy`, `qz` | No | Quaternion orientation |
            | `ax`, `ay`, `az` | No | Accelerometer (m/s²) |
            | `gx`, `gy`, `gz` | No | Gyroscope (rad/s) |
            | `gps_fix` | No | GPS fix status (0/1) |
            """)


if __name__ == "__main__":
    main()
