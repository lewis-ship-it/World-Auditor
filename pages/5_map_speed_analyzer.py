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
import math

st.set_page_config(layout="wide", page_title="World-Auditor | Speed Analyzer")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER (UNRESTRICTED)
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["RWD", "FWD", "AWD", "4WD"])
drive_mu_modifier = {"FWD": 0.92, "RWD": 0.96, "AWD": 1.05, "4WD": 1.08}
base_friction = st.sidebar.number_input("Surface Grip μ", value=1.2)
friction = base_friction * drive_mu_modifier[drive_type]

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length Scale (m)", value=20832.0)
rotation = st.sidebar.number_input("Rotate Track (deg)", value=0.0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# FILE INPUT & PREVIEWS
# ------------------------------------------------
uploaded = st.file_uploader("Upload Track Map", type=['png', 'jpg', 'jpeg'])

if uploaded:
    file_bytes = np.frombuffer(uploaded.read(), np.uint8)
    img = cv2.imdecode(file_bytes, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    p_col1, p_col2 = st.columns(2)
    with p_col1:
        st.subheader("Original Upload")
        st.image(img_rgb, use_container_width=True)
    
    centerline = extract_track_centerline(img)
    if centerline is None:
        st.error("AI Perception failed to detect track boundaries.")
        st.stop()

    with p_col2:
        st.subheader("AI Vision View")
        viz_ai = np.zeros_like(img)
        cv2.polylines(viz_ai, [centerline.astype(np.int32)], False, (0, 255, 255), 2)
        st.image(viz_ai, use_container_width=True)

    # ----------------------------
    # PATH PROCESSING
    # ----------------------------
    path = centerline.astype(float)
    theta = np.radians(rotation)
    rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    path = path @ rot_matrix

    if optimize_line:
        optimizer = RacingLineOptimizer()
        path = optimizer.optimize(path)

    xs, ys = path[:, 0], path[:, 1]
    
    # Scale to real-world meters
    d_raw = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    scale = track_distance / np.sum(d_raw)
    xs *= scale
    ys *= scale

    # Path Smoothing (Fixes the 150-min lap jitter)
    if len(xs) > 15:
        xs = savgol_filter(xs, 15, 3)
        ys = savgol_filter(ys, 15, 3)

    # ----------------------------
    # DYNAMIC PHYSICS ENGINE
    # ----------------------------
    # 1. Curvature Calculation (Menger Method)
    curvature = []
    window = 3
    for i in range(window, len(xs) - window):
        p1, p2, p3 = np.array([xs[i-window], ys[i-window]]), np.array([xs[i], ys[i]]), np.array([xs[i+window], ys[i+window]])
        area = 0.5 * abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        d1, d2, d3 = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
        curvature.append((4 * area) / (d1 * d2 * d3 + 1e-6))
    curvature = np.array([curvature[0]]*window + curvature + [curvature[-1]]*window)

    # 2. Three-Pass Speed Solver
    g = 9.81
    # Pass A: Lateral Grip Limits
    max_speeds = np.sqrt((friction * g) / (curvature + 1e-6))
    max_speeds = np.clip(max_speeds, 12.0, 95.0) 
    
    distances = np.append(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2), 0)

    # Pass B: Forward Acceleration
    for i in range(1, len(max_speeds)):
        max_speeds[i] = min(max_speeds[i], np.sqrt(max_speeds[i-1]**2 + 2 * 4.5 * distances[i-1]))
    
    # Pass C: Backward Braking
    for i in range(len(max_speeds)-2, -1, -1):
        max_speeds[i] = min(max_speeds[i], np.sqrt(max_speeds[i+1]**2 + 2 * 8.0 * distances[i]))

    # ----------------------------
    # TELEMETRY CALCS
    # ----------------------------
    cum_dist = np.cumsum(distances)
    lat_g = (max_speeds**2 * curvature) / g
    
    # Thermal Model (Ambient Start)
    tire_temp, brake_temp = [], []
    tt, bt = 25.0, 30.0
    for v in max_speeds:
        tt += 0.08 * (v * 0.4) - 0.02 * (tt - 25)
        bt += 0.12 * (v * 0.7) - 0.03 * (bt - 30)
        tire_temp.append(tt)
        brake_temp.append(bt)

    # Energy Model
    em = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy_net = em.energy_used(max_speeds, distances) - np.append(em.regen_energy(np.abs(np.diff(max_speeds)), 0.6), 0)
    soc = np.clip(100 - (np.cumsum(energy_net) / (battery_capacity * 3.6e6) * 100), 0, 100)

    # ----------------------------
    # UI: VISUALIZATION & AUDIT
    # ----------------------------
    st.divider()
    m_col1, m_col2 = st.columns([2, 1])

    with m_col1:
        st.subheader("Spatial Speed Map")
        fig_map = go.Figure()
        fig_map.add_trace(go.Scatter(
            x=xs, y=ys, mode="lines+markers",
            line=dict(color="rgba(255,255,255,0.2)", width=1),
            marker=dict(color=max_speeds, colorscale="Turbo", size=4, showscale=True, colorbar=dict(title="m/s"))
        ))
        fig_map.update_layout(template="plotly_dark", height=600, yaxis=dict(scaleanchor="x"))
        st.plotly_chart(fig_map, use_container_width=True)

    with m_col2:
        lap_time_s = np.sum(distances / (max_speeds + 1e-4))
        st.metric("Lap Time", f"{int(lap_time_s//60)}m {lap_time_s%60:.2f}s")
        st.metric("Top Speed", f"{np.max(max_speeds)*3.6:.1f} km/h")
        st.metric("Avg Lat G", f"{np.mean(lat_g):.2f} G")
        st.metric("Energy/Lap", f"{np.sum(energy_net)/1000:.1f} kJ")

    # Audit Data Table vs Graph Tabs
    st.subheader("📊 Telemetry Audit")
    tab_table, tab_graph = st.tabs(["📋 Raw Telemetry Data", "📈 Performance Graphs"])
    
    with tab_table:
        df = pd.DataFrame({
            "Distance (m)": cum_dist,
            "Speed (km/h)": max_speeds * 3.6,
            "Lateral G": lat_g,
            "Tire Temp (C)": tire_temp,
            "Battery SOC (%)": soc
        })
        st.dataframe(df, use_container_width=True, height=350)
        st.download_button("Export Audit CSV", df.to_csv().encode('utf-8'), "audit.csv")

    with tab_graph:
        # Speed/G Graph
        fig_tel = go.Figure()
        fig_tel.add_trace(go.Scatter(x=cum_dist, y=max_speeds*3.6, name="Speed (km/h)"))
        fig_tel.add_trace(go.Scatter(x=cum_dist, y=lat_g, name="Lateral G", yaxis="y2"))
        fig_tel.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", title="G-Force"))
        st.plotly_chart(fig_tel, use_container_width=True)

        # Thermal Graph
        fig_th = go.Figure()
        fig_th.add_trace(go.Scatter(x=cum_dist, y=tire_temp, name="Tire Temp"))
        fig_th.add_trace(go.Scatter(x=cum_dist, y=soc, name="Battery %", yaxis="y2"))
        fig_th.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", range=[0, 105]))
        st.plotly_chart(fig_th, use_container_width=True)