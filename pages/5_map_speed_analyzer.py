import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel
from scipy.signal import savgol_filter # Essential for smoothing path noise
import math

st.set_page_config(layout="wide")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER (UNRESTRICTED)
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")

mass = st.sidebar.number_input("Mass (kg)", value=1470.0) # Defaulted to GT2 RS weight
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["RWD", "FWD", "AWD", "4WD"])

drive_mu_modifier = {
    "FWD": 0.92,
    "RWD": 0.96,
    "AWD": 1.05,
    "4WD": 1.08
}

friction = st.sidebar.number_input("Surface Grip μ", value=1.2) # Defaulted to High-Performance tires
friction *= drive_mu_modifier[drive_type]

# ------------------------------------------------
# TRACK SETTINGS
# ------------------------------------------------
st.sidebar.header("Track Settings")
track_width = st.sidebar.number_input("Track Width (m)", value=12.0)
track_distance = st.sidebar.number_input("Track Length Scale (m)", value=20832.0) # Nordschleife length
rotation = st.sidebar.number_input("Rotate Track (deg)", value=0.0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# FILE INPUT & SEPARATE PREVIEWS
# ------------------------------------------------
uploaded = st.file_uploader("Upload Track Map")

if uploaded:
    file_bytes = np.frombuffer(uploaded.read(), np.uint8)
    img = cv2.imdecode(file_bytes, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    prev_col1, prev_col2 = st.columns(2)
    with prev_col1:
        st.subheader("Original Upload")
        st.image(img_rgb, use_container_width=True)
    
    centerline = extract_track_centerline(img)

    if centerline is None:
        st.error("No track detected")
        st.stop()

    with prev_col2:
        st.subheader("AI Vision View")
        viz_ai = np.zeros_like(img)
        cv2.polylines(viz_ai, [centerline.astype(np.int32)], False, (0, 255, 255), 2)
        st.image(viz_ai, use_container_width=True)

    path = centerline.astype(float)

    # ----------------------------
    # ROTATION & OPTIMIZATION
    # ----------------------------
    theta = np.radians(rotation)
    rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    path = path @ rot

    if optimize_line:
        optimizer = RacingLineOptimizer()
        path = optimizer.optimize(path)

    xs, ys = path[:, 0], path[:, 1]

    # ----------------------------
    # SCALE TRACK
    # ----------------------------
    d_raw = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    scale = track_distance / np.sum(d_raw)
    xs *= scale
    ys *= scale

    # ----------------------------
    # PATH SMOOTHING (Surgical Fix for 150-min Lap)
    # ----------------------------
    # This window removes pixel-level noise while keeping the overall track shape
    window_size = 15 
    if len(xs) > window_size:
        xs = savgol_filter(xs, window_size, 3)
        ys = savgol_filter(ys, window_size, 3)

    # ----------------------------
    # CURVATURE & PHYSICS
    # ----------------------------
    curvature = []
    for i in range(1, len(xs)-1):
        p1, p2, p3 = np.array([xs[i-1], ys[i-1]]), np.array([xs[i], ys[i]]), np.array([xs[i+1], ys[i+1]])
        # Normalized Menger curvature
        k = np.linalg.norm(p3 - 2*p2 + p1) / (scale * 2.0)
        curvature.append(k)
    curvature = np.array([curvature[0]] + curvature + [curvature[-1]])

    g = 9.81
    downforce = 0.5 * 1.225 * frontal_area * drag * (curvature + 1)
    grip_limit = friction * g + (downforce / mass)
    
    # Speed calculation with a 15 m/s (54 km/h) floor to prevent the "crawl"
    max_speeds = np.sqrt(grip_limit / (curvature + 1e-6))
    max_speeds = np.clip(max_speeds, 15.0, 95.0) 

    # ----------------------------
    # TELEMETRY DATA
    # ----------------------------
    distances = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    distances = np.append(distances, distances[-1])
    cumulative_distance = np.cumsum(distances)

    energy_model = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy = energy_model.energy_used(max_speeds, distances)
    regen = energy_model.regen_energy(np.abs(np.diff(max_speeds)), 0.6)
    net_energy = energy - np.append(regen, 0)

    battery_j = battery_capacity * 3.6e6
    soc = np.clip(100 - (np.cumsum(net_energy) / battery_j * 100), 0, 100)

    # Thermal rise from Ambient
    tire_temp, brake_temp = [], []
    tt, bt = 25.0, 30.0 
    for v in max_speeds:
        tt += 0.08 * (v * 0.4) - 0.02 * (tt - 25)
        bt += 0.12 * (v * 0.7) - 0.03 * (bt - 30)
        tire_temp.append(tt)
        brake_temp.append(bt)

    lat_g = (max_speeds**2 * curvature) / g

    # ------------------------------------------------
    # VISUALIZATION
    # ------------------------------------------------
    st.divider()
    col1, col2 = st.columns([2, 1])

    with col1:
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=xs, y=ys, mode="lines+markers",
            line=dict(width=2, color="rgba(255,255,255,0.3)"),
            marker=dict(size=4, color=max_speeds, colorscale="Turbo", showscale=True, colorbar=dict(title="m/s")),
            hovertemplate="Speed %{marker.color:.1f} m/s"
        ))
        fig.update_layout(template="plotly_dark", height=600, yaxis=dict(scaleanchor="x"))
        st.plotly_chart(fig, use_container_width=True)

    with col2:
        lap_time_s = np.sum(distances / (max_speeds + 1e-4))
        st.metric("Lap Time", f"{int(lap_time_s//60)}m {lap_time_s%60:.2f}s")
        st.metric("Avg Speed", f"{np.mean(max_speeds)*3.6:.1f} km/h")
        st.metric("Max Speed", f"{np.max(max_speeds)*3.6:.1f} km/h")
        st.metric("Energy Used", f"{np.sum(net_energy)/1000:.2f} kJ")

    # ------------------------------------------------
    # GRAPHS
    # ------------------------------------------------
    st.subheader("Speed & G-Force Telemetry")
    f_speed = go.Figure()
    f_speed.add_trace(go.Scatter(x=cumulative_distance, y=max_speeds*3.6, name="Speed (km/h)"))
    f_speed.add_trace(go.Scatter(x=cumulative_distance, y=lat_g, name="Lateral G", yaxis="y2"))
    f_speed.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", title="G-Force"))
    st.plotly_chart(f_speed, use_container_width=True)

    st.subheader("Thermal & Energy Profile")
    f_therm = go.Figure()
    f_therm.add_trace(go.Scatter(x=cumulative_distance, y=tire_temp, name="Tire Temp (°C)"))
    f_therm.add_trace(go.Scatter(x=cumulative_distance, y=soc, name="Battery %", yaxis="y2"))
    f_therm.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", range=[0, 100]))
    st.plotly_chart(f_therm, use_container_width=True)