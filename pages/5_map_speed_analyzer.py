import streamlit as st
import numpy as np
import cv2
import pandas as pd
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel
from scipy.signal import savgol_filter

st.set_page_config(layout="wide", page_title="World-Auditor | Physics Lab")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["RWD", "FWD", "AWD", "4WD"])
friction = st.sidebar.number_input("Surface Grip μ", value=1.1) # Lowered for more dynamic cornering

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length Scale (m)", value=20832.0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

uploaded = st.file_uploader("Upload Track Map")

if uploaded:
    file_bytes = np.frombuffer(uploaded.read(), np.uint8)
    img = cv2.imdecode(file_bytes, 1)
    
    # Previews
    p_col1, p_col2 = st.columns(2)
    with p_col1:
        st.image(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), caption="Original", use_container_width=True)
    
    centerline = extract_track_centerline(img)
    if centerline is None: st.stop()
        
    path = centerline.astype(float)
    if optimize_line:
        path = RacingLineOptimizer().optimize(path)

    xs, ys = path[:, 0], path[:, 1]
    
    # Scaling & Smoothing
    d_raw = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    scale = track_distance / np.sum(d_raw)
    xs *= scale
    ys *= scale
    xs = savgol_filter(xs, 15, 3)
    ys = savgol_filter(ys, 15, 3)

    # ----------------------------
    # HIGH-SENSITIVITY CURVATURE
    # ----------------------------
    curvature = []
    window = 5 # Wider window to detect real corners, not pixel noise
    for i in range(window, len(xs) - window):
        p1, p2, p3 = np.array([xs[i-window], ys[i-window]]), np.array([xs[i], ys[i]]), np.array([xs[i+window], ys[i+window]])
        area = 0.5 * abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        d1, d2, d3 = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
        # The core curvature math
        k = (4 * area) / (d1 * d2 * d3 + 1e-6)
        curvature.append(k)
    curvature = np.array([curvature[0]]*window + curvature + [curvature[-1]]*window)

    # ----------------------------
    # THREE-PASS PHYSICS SOLVER
    # ----------------------------
    g = 9.81
    # Pass 1: Lateral Limit (The 'V' bottom)
    max_speeds = np.sqrt((friction * g) / (curvature + 1e-6))
    max_speeds = np.clip(max_speeds, 12.0, 92.0) # 92 m/s ~ 331 km/h
    
    dist = np.append(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2), 0)
    
    # Pass 2: Forward (Accel) - Car speeding up
    for i in range(1, len(max_speeds)):
        max_speeds[i] = min(max_speeds[i], np.sqrt(max_speeds[i-1]**2 + 2 * 4.0 * dist[i-1]))
    
    # Pass 3: Backward (Braking) - Car slowing down BEFORE turn
    for i in range(len(max_speeds)-2, -1, -1):
        max_speeds[i] = min(max_speeds[i], np.sqrt(max_speeds[i+1]**2 + 2 * 7.0 * dist[i]))

    # ----------------------------
    # TELEMETRY MODELS
    # ----------------------------
    cum_dist = np.cumsum(dist)
    lat_g = (max_speeds**2 * curvature) / g
    
    # Thermal & Energy
    tire_temp, brake_temp = [], []
    tt, bt = 25.0, 30.0
    for v in max_speeds:
        tt += 0.08 * (v * 0.4) - 0.02 * (tt - 25)
        bt += 0.15 * (v * 0.8) - 0.03 * (bt - 30) # Aggressive brake heat
        tire_temp.append(tt)
        brake_temp.append(bt)

    em = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy_used = em.energy_used(max_speeds, dist)
    soc = np.clip(100 - (np.cumsum(energy_used) / (battery_capacity * 3.6e6) * 100), 0, 100)

    # ----------------------------
    # UI: TABS FOR GRAPHS
    # ----------------------------
    st.divider()
    tab_map, tab_speed, tab_thermal, tab_energy = st.tabs(["🗺️ Track Map", "📈 Speed/G", "🔥 Thermal", "⚡ Energy"])

    with tab_map:
        fig_m = go.Figure(go.Scatter(x=xs, y=ys, mode="markers", marker=dict(color=max_speeds, colorscale="Turbo", size=4, showscale=True)))
        fig_m.update_layout(template="plotly_dark", height=600, yaxis=dict(scaleanchor="x"))
        st.plotly_chart(fig_m, use_container_width=True)

    with tab_speed:
        st.subheader("Speed (km/h) vs Lateral G")
        f1 = go.Figure()
        f1.add_trace(go.Scatter(x=cum_dist, y=max_speeds*3.6, name="Speed"))
        f1.add_trace(go.Scatter(x=cum_dist, y=lat_g, name="G-Force", yaxis="y2"))
        f1.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right"))
        st.plotly_chart(f1, use_container_width=True)

    with tab_thermal:
        st.subheader("Component Temperatures (°C)")
        f2 = go.Figure()
        f2.add_trace(go.Scatter(x=cum_dist, y=tire_temp, name="Tire Temp", line=dict(color="orange")))
        f2.add_trace(go.Scatter(x=cum_dist, y=brake_temp, name="Brake Temp", line=dict(color="red")))
        f2.update_layout(template="plotly_dark")
        st.plotly_chart(f2, use_container_width=True)

    with tab_energy:
        st.subheader("Battery Consumption")
        f3 = go.Figure()
        f3.add_trace(go.Scatter(x=cum_dist, y=soc, name="Battery %", fill='tozeroy'))
        f3.update_layout(template="plotly_dark", yaxis=dict(range=[0, 105]))
        st.plotly_chart(f3, use_container_width=True)