import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel
from scipy.signal import savgol_filter
import math

st.set_page_config(layout="wide")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER (UNRESTRICTED)
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")

# Using number_input for unrestricted values
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["RWD", "FWD", "AWD", "4WD"])

drive_mu_modifier = {"FWD": 0.92, "RWD": 0.96, "AWD": 1.05, "4WD": 1.08}
friction = st.sidebar.number_input("Surface Grip μ", value=1.2)
friction *= drive_mu_modifier[drive_type]

# ------------------------------------------------
# TRACK SETTINGS
# ------------------------------------------------
st.sidebar.header("Track Settings")
track_width = st.sidebar.number_input("Track Width (m)", value=12.0)
track_distance = st.sidebar.number_input("Track Length Scale (m)", value=20832.0)
rotation = st.sidebar.number_input("Rotate Track (deg)", value=0.0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# FILE INPUT & SEPARATE PREVIEWS (RESTORED)
# ------------------------------------------------
uploaded = st.file_uploader("Upload Track Map")

if uploaded:
    # Read Image
    file_bytes = np.frombuffer(uploaded.read(), np.uint8)
    img = cv2.imdecode(file_bytes, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # UI: Restoration of the Dual Preview (Side-by-Side)
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
    # SCALE & PATH SMOOTHING
    # ----------------------------
    d_raw = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    scale = track_distance / np.sum(d_raw)
    xs *= scale
    ys *= scale

    # Savgol smoothing to remove pixel jitter (The 150-min lap fix)
    if len(xs) > 15:
        xs = savgol_filter(xs, 15, 3)
        ys = savgol_filter(ys, 15, 3)

    # ----------------------------
    # CURVATURE (MENGER METHOD)
    # ----------------------------
    curvature = []
    window = 3 
    for i in range(window, len(xs) - window):
        p1 = np.array([xs[i - window], ys[i - window]])
        p2 = np.array([xs[i], ys[i]])
        p3 = np.array([xs[i + window], ys[i + window]])
        # 3-Point Curvature formula
        area = 0.5 * abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        d1, d2, d3 = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
        k = (4 * area) / (d1 * d2 * d3 + 1e-6)
        curvature.append(k)
    
    # Pad ends
    curvature = np.array([curvature[0]]*window + curvature + [curvature[-1]]*window)

    # ----------------------------
    # THREE-PASS SPEED SOLVER (FIXED DYNAMIC SPEED)
    # ----------------------------
    g = 9.81
    # 1. Lateral Pass (Grip Limit)
    downforce = 0.5 * 1.225 * frontal_area * drag * (curvature + 1)
    grip_limit = friction * g + (downforce / mass)
    max_speeds = np.sqrt(grip_limit / (curvature + 1e-6))
    max_speeds = np.clip(max_speeds, 10.0, 95.0) 

    # Distances for passes
    distances = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    distances = np.append(distances, distances[-1])

    # 2. Forward Pass (Acceleration)
    a_max = 5.0 # m/s^2
    for i in range(1, len(max_speeds)):
        v_reach = np.sqrt(max_speeds[i-1]**2 + 2 * a_max * distances[i-1])
        max_speeds[i] = min(max_speeds[i], v_reach)

    # 3. Backward Pass (Braking)
    b_max = 8.0 # m/s^2
    for i in range(len(max_speeds)-2, -1, -1):
        v_need = np.sqrt(max_speeds[i+1]**2 + 2 * b_max * distances[i])
        max_speeds[i] = min(max_speeds[i], v_need)

    # ----------------------------
    # TELEMETRY & THERMAL
    # ----------------------------
    cumulative_distance = np.cumsum(distances)
    energy_model = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy = energy_model.energy_used(max_speeds, distances)
    net_energy = energy - np.append(energy_model.regen_energy(np.abs(np.diff(max_speeds)), 0.6), 0)

    battery_j = battery_capacity * 3.6e6
    soc = np.clip(100 - (np.cumsum(net_energy) / battery_j * 100), 0, 100)

    tire_temp, brake_temp = [], []
    tt, bt = 25.0, 30.0 # Start at ambient
    for v in max_speeds:
        tt += 0.08 * (v * 0.4) - 0.02 * (tt - 25)
        bt += 0.12 * (v * 0.7) - 0.03 * (bt - 30)
        tire_temp.append(tt)
        brake_temp.append(bt)

    lat_g = (max_speeds**2 * curvature) / g

    # ------------------------------------------------
    # MAIN VISUALIZATION (FIXED PLOTLY TRACE)
    # ------------------------------------------------
    st.divider()
    col1, col2 = st.columns([2, 1])

    with col1:
        fig = go.Figure()
        # Marker dictionary handles the color array for the heatmap
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
    # TELEMETRY GRAPHS
    # ------------------------------------------------
    st.subheader("Dynamic Speed Profile")
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