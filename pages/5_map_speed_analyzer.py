import streamlit as st
import numpy as np
import cv2
import pandas as pd
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from scipy.signal import savgol_filter

st.set_page_config(layout="wide", page_title="World-Auditor | Physics Lab")

st.title("🏁 Ultimate Track Speed Analyzer (Fixed & Robust)")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length (m)", value=420.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line (May cause errors on noisy images)", False)

uploaded_file = st.file_uploader("Upload Track Image", type=["jpg", "png", "jpeg"])

if uploaded_file:
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    with st.spinner("Analyzing track geometry..."):
        # 1. Extract raw centerline
        raw_points = extract_track_centerline(image)
        
        if raw_points is None or len(raw_points) < 5:
            st.error("Could not detect a clear track. Try an image with higher contrast.")
            st.stop()

        # 2. Safety Check for Optimizer
        points = raw_points
        if use_racing_line:
            try:
                optimizer = RacingLineOptimizer(raw_points)
                optimized = optimizer.optimize()
                if optimized is not None and len(optimized) > 0:
                    points = np.array(optimized)
                else:
                    st.warning("Optimizer failed to converge. Using raw centerline instead.")
            except Exception as e:
                st.warning(f"Optimization Error: {e}. Falling back to raw centerline.")
                points = raw_points

    # 3. Geometry & Smoothing
    # Ensure points is a numpy array
    points = np.array(points)
    xs, ys = points[:, 0], points[:, 1]
    
    # Smooth the coordinates to prevent "infinite curvature" spikes from pixels
    window_size = min(len(xs) // 5, 15)
    if window_size % 2 == 0: window_size += 1 # Must be odd for savgol
    
    xs_smooth = savgol_filter(xs, window_size, 3)
    ys_smooth = savgol_filter(ys, window_size, 3)

    # Calculate Curvature (k)
    dx = np.gradient(xs_smooth)
    dy = np.gradient(ys_smooth)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # K = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    curvature = np.abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)

    # 4. Physical Scaling
    pixel_segments = np.sqrt(np.diff(xs_smooth)**2 + np.diff(ys_smooth)**2)
    total_pixel_len = np.sum(pixel_segments)
    scale_factor = track_distance / (total_pixel_len + 1e-6)
    cum_dist = np.insert(np.cumsum(pixel_segments) * scale_factor, 0, 0)

    # ------------------------------------------------
    # DYNAMIC PHYSICS SOLVER
    # ------------------------------------------------
    g = 9.81
    # Max Accel/Brake limited by friction
    a_max = friction * g * 0.7  # Conservative motor limit
    b_max = friction * g * 0.9  # Grip-limited braking
    v_top_limit = 92.0          # 330 km/h cap

    # PASS 1: LATERAL (Cornering Speed)
    # V = sqrt(Friction * g / Curvature)
    v_target = np.sqrt((friction * g) / (curvature + 1e-6))
    v_target = np.clip(v_target, 1.0, v_top_limit)

    # PASS 2: FORWARD (Accelerating out of corners)
    v_forward = np.zeros_like(v_target)
    for i in range(1, len(v_forward)):
        ds = pixel_segments[i-1] * scale_factor
        max_possible = np.sqrt(v_forward[i-1]**2 + 2 * a_max * ds)
        v_forward[i] = min(v_target[i], max_possible)

    # PASS 3: BACKWARD (Braking into corners)
    v_final = np.copy(v_forward)
    for i in range(len(v_final)-2, -1, -1):
        ds = pixel_segments[i] * scale_factor
        max_allowed = np.sqrt(v_final[i+1]**2 + 2 * b_max * ds)
        v_final[i] = min(v_final[i], max_allowed)

    # ------------------------------------------------
    # UI RENDERING
    # ------------------------------------------------
    v_kmh = v_final * 3.6
    
    col1, col2 = st.columns([1, 1])
    with col1:
        st.metric("Top Speed", f"{np.max(v_kmh):.1f} km/h")
        fig_map = go.Figure(data=go.Scatter(
            x=xs_smooth, y=ys_smooth, mode='lines',
            line=dict(color=v_kmh, colorscale='Electric', width=5)
        ))
        fig_map.update_layout(title="Speed Heatmap", template="plotly_dark")
        st.plotly_chart(fig_map)

    with col2:
        st.metric("Avg Speed", f"{np.mean(v_kmh):.1f} km/h")
        fig_curve = go.Figure()
        fig_curve.add_trace(go.Scatter(x=cum_dist, y=v_kmh, name="Speed (km/h)"))
        fig_curve.update_layout(title="Velocity Profile", template="plotly_dark", xaxis_title="Distance (m)")
        st.plotly_chart(fig_curve)