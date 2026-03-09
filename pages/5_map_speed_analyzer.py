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

st.title("🏁 Ultimate Track Speed Analyzer (Fixed Physics)")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 
max_power_kw = st.sidebar.number_input("Max Motor Power (kW)", value=150.0)

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length (m)", value=420.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# IMAGE UPLOAD & PROCESSING
# ------------------------------------------------
uploaded_file = st.file_uploader("Upload Track Image", type=["jpg", "png", "jpeg"])

if uploaded_file:
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    # 1. Extract Path
    with st.spinner("Analyzing track geometry..."):
        points = extract_track_centerline(image)
        if use_racing_line:
            optimizer = RacingLineOptimizer(points)
            points = optimizer.optimize()

    # 2. Geometry & Curvature
    xs, ys = points[:, 0], points[:, 1]
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature calculation with safety epsilon
    curvature = np.abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
    curvature = savgol_filter(curvature, 15, 3) # Smooth pixel noise
    
    # 3. Scaling to real world
    pixel_dist = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    total_pixel_len = np.sum(pixel_dist)
    scale = track_distance / total_pixel_len
    cum_dist = np.insert(np.cumsum(pixel_dist) * scale, 0, 0)

    # ------------------------------------------------
    # THE FIXED THREE-PASS SOLVER
    # ------------------------------------------------
    g = 9.81
    # Physical limits derived from Sidebar
    a_max = (friction * g) * 0.8  # Max acceleration (limited by grip)
    b_max = (friction * g) * 0.9  # Max braking (limited by grip)
    v_max_abs = 92.0 # Hard cap (330 km/h)

    # PASS 1: LATERAL LIMITS (Cornering Speed)
    # V = sqrt(μ * g / curvature)
    target_v = np.sqrt((friction * g) / (curvature + 1e-6))
    target_v = np.clip(target_v, 0, v_max_abs)

    # PASS 2: FORWARD (Acceleration/Engine Limit)
    v_forward = np.zeros_like(target_v)
    v_forward[0] = 0 # Start from stop
    for i in range(1, len(v_forward)):
        ds = pixel_dist[i-1] * scale
        # Physics: v_final^2 = v_initial^2 + 2*a*ds
        max_v_possible = np.sqrt(v_forward[i-1]**2 + 2 * a_max * ds)
        v_forward[i] = min(target_v[i], max_v_possible)

    # PASS 3: BACKWARD (Braking Look-ahead)
    v_final = np.copy(v_forward)
    for i in range(len(v_final)-2, -1, -1):
        ds = pixel_dist[i] * scale
        # Physics: can we slow down enough for the next point?
        max_v_allowed = np.sqrt(v_final[i+1]**2 + 2 * b_max * ds)
        v_final[i] = min(v_final[i], max_v_allowed)

    # Convert to km/h for display
    v_kmh = v_final * 3.6

    # ------------------------------------------------
    # RESULTS & VISUALIZATION
    # ------------------------------------------------
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.metric("Avg Speed", f"{np.mean(v_kmh):.1f} km/h")
        st.metric("Lap Time", f"{np.sum((pixel_dist*scale)/np.maximum(v_final[1:], 0.1)):.2f} s")
        
        # Track Map Visualization
        fig_map = go.Figure()
        fig_map.add_trace(go.Scatter(x=xs, y=ys, mode='lines', 
                                    line=dict(color=v_kmh, colorscale='Turbo', width=6)))
        fig_map.update_layout(title="Speed Map", template="plotly_dark", height=400)
        st.plotly_chart(fig_map, use_container_width=True)

    with col2:
        st.subheader("Velocity & G-Force Profile")
        f_speed = go.Figure()
        f_speed.add_trace(go.Scatter(x=cum_dist, y=v_kmh, name="Velocity (km/h)", line=dict(color='#00FFCC')))
        
        # Calculate Lateral Gs for the second axis
        lat_g = (v_final**2 * curvature) / g
        f_speed.add_trace(go.Scatter(x=cum_dist, y=lat_g, name="Lateral G", yaxis="y2", line=dict(dash='dot')))
        
        f_speed.update_layout(
            template="plotly_dark",
            yaxis=dict(title="km/h"),
            yaxis2=dict(title="G-Force", overlaying="y", side="right"),
            height=500
        )
        st.plotly_chart(f_speed, use_container_width=True)