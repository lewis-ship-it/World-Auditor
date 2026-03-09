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
# Forced a slightly lower friction to ensure the physics engine respects corners
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 

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
    if centerline is None:
        st.error("Track detection failed.")
        st.stop()
        
    path = centerline.astype(float)
    if optimize_line:
        path = RacingLineOptimizer().optimize(path)

    xs, ys = path[:, 0], path[:, 1]
    
    # ----------------------------
    # SCALING & NOISE REDUCTION
    # ----------------------------
    d_raw = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    scale = track_distance / np.sum(d_raw)
    xs *= scale
    ys *= scale
    
    # Smooth to remove 'pixel-steps' that ruin curvature math
    xs = savgol_filter(xs, 11, 3)
    ys = savgol_filter(ys, 11, 3)

    # ----------------------------
    # AGGRESSIVE CURVATURE MATH
    # ----------------------------
    curvature = []
    w = 5 # Look-ahead window
    for i in range(w, len(xs) - w):
        p1, p2, p3 = np.array([xs[i-w], ys[i-w]]), np.array([xs[i], ys[i]]), np.array([xs[i+w], ys[i+w]])
        area = 0.5 * abs(p1[0]*(p2[1]-p3[1]) + p2[0]*(p3[1]-p1[1]) + p3[0]*(p1[1]-p2[1]))
        d1, d2, d3 = np.linalg.norm(p2-p1), np.linalg.norm(p3-p2), np.linalg.norm(p3-p1)
        k = (4 * area) / (d1 * d2 * d3 + 1e-6)
        curvature.append(k)
    curvature = np.array([curvature[0]]*w + curvature + [curvature[-1]]*w)

    # ----------------------------
    # THE THREE-PASS SOLVER (THE "342 FIX")
    # ----------------------------
    g = 9.81
    dist_segments = np.append(np.sqrt(np.diff(xs)**2 + np.diff(ys)**2), 0)
    
    # PASS 1: LATERAL (Cornering Limit)
    # V = sqrt(mu * g / k)
    lateral_limit = np.sqrt((friction * g) / (curvature + 1e-7))
    v_profile = np.clip(lateral_limit, 10.0, 92.0) # Cap at ~330km/h

    # PASS 2: FORWARD (Acceleration)
    a_max = 4.2 # m/s^2 
    for i in range(1, len(v_profile)):
        v_reachable = np.sqrt(v_profile[i-1]**2 + 2 * a_max * dist_segments[i-1])
        v_profile[i] = min(v_profile[i], v_reachable)

    # PASS 3: BACKWARD (Braking)
    b_max = 7.5 # m/s^2
    for i in range(len(v_profile)-2, -1, -1):
        v_needed = np.sqrt(v_profile[i+1]**2 + 2 * b_max * dist_segments[i])
        v_profile[i] = min(v_profile[i], v_needed)

    # ----------------------------
    # TELEMETRY GENERATION
    # ----------------------------
    cum_dist = np.cumsum(dist_segments)
    lat_g = (v_profile**2 * curvature) / g
    
    # Thermal simulation
    tire_temp, brake_temp = [], []
    t_curr, b_curr = 25.0, 30.0
    for v in v_profile:
        t_curr += 0.09 * (v * 0.35) - 0.02 * (t_curr - 25)
        b_curr += 0.18 * (v * 0.75) - 0.04 * (b_curr - 30)
        tire_temp.append(t_curr)
        brake_temp.append(b_curr)

    # Energy simulation
    em = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    step_energy = em.energy_used(v_profile, dist_segments)
    soc = np.clip(100 - (np.cumsum(step_energy) / (battery_capacity * 3.6e6) * 100), 0, 100)

    # ----------------------------
    # THE 4-GRAPH DASHBOARD
    # ----------------------------
    st.divider()
    
    # Row 1: Speed and G-Force
    st.subheader("📈 Speed & Lateral G-Force")
    f_speed = go.Figure()
    f_speed.add_trace(go.Scatter(x=cum_dist, y=v_profile*3.6, name="Speed (km/h)", line=dict(color='#00CCFF')))
    f_speed.add_trace(go.Scatter(x=cum_dist, y=lat_g, name="Lat G", yaxis="y2", line=dict(color='#FF3300')))
    f_speed.update_layout(
        template="plotly_dark",
        yaxis=dict(title="km/h"),
        yaxis2=dict(title="G", overlaying="y", side="right"),
        height=400
    )
    st.plotly_chart(f_speed, use_container_width=True)

    # Row 2: Thermal and Energy
    col_a, col_b = st.columns(2)
    
    with col_a:
        st.subheader("🔥 Component Thermals")
        f_temp = go.Figure()
        f_temp.add_trace(go.Scatter(x=cum_dist, y=tire_temp, name="Tires (°C)", line=dict(color='orange')))
        f_temp.add_trace(go.Scatter(x=cum_dist, y=brake_temp, name="Brakes (°C)", line=dict(color='red')))
        f_temp.update_layout(template="plotly_dark", height=350)
        st.plotly_chart(f_temp, use_container_width=True)

    with col_b:
        st.subheader("⚡ Battery Energy")
        f_energy = go.Figure()
        f_energy.add_trace(go.Scatter(x=cum_dist, y=soc, name="SOC %", fill='tozeroy', line=dict(color='#00FF00')))
        f_energy.update_layout(template="plotly_dark", yaxis=dict(range=[0, 105]), height=350)
        st.plotly_chart(f_energy, use_container_width=True)

    # Row 3: The Map
    st.subheader("🗺️ Speed Heatmap")
    fig_map = go.Figure(go.Scatter(
        x=xs, y=ys, mode="markers",
        marker=dict(color=v_profile*3.6, colorscale="Turbo", size=4, showscale=True, colorbar=dict(title="km/h"))
    ))
    fig_map.update_layout(template="plotly_dark", height=600, yaxis=dict(scaleanchor="x"))
    st.plotly_chart(fig_map, use_container_width=True)