import streamlit as st
import numpy as np
import cv2
import pandas as pd
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from scipy.signal import savgol_filter

st.set_page_config(layout="wide", page_title="World-Auditor | Physics Lab")

st.title("🏁 Ultimate Track Speed Analyzer (Length-Fixed)")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length (m)", value=420.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line", False)

uploaded_file = st.file_uploader("Upload Track Image", type=["jpg", "png", "jpeg"])

if uploaded_file:
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    with st.spinner("Analyzing track geometry..."):
        raw_points = extract_track_centerline(image)
        
        if raw_points is None or len(raw_points) < 10:
            st.error("Track too short or not detected. Use a clearer image.")
            st.stop()

        points = raw_points
        if use_racing_line:
            try:
                optimizer = RacingLineOptimizer(raw_points)
                optimized = optimizer.optimize()
                if optimized is not None: points = np.array(optimized)
            except:
                st.warning("Optimizer failed. Using raw path.")

    # 1. Coordinate Smoothing
    points = np.array(points)
    xs, ys = points[:, 0], points[:, 1]
    
    window = min(len(xs) // 5, 15)
    if window % 2 == 0: window += 1
    
    xs_smooth = savgol_filter(xs, window, 3)
    ys_smooth = savgol_filter(ys, window, 3)

    # 2. Distance and Curvature
    # We calculate step-by-step distances (length = N-1)
    dx_steps = np.diff(xs_smooth)
    dy_steps = np.diff(ys_smooth)
    ds_pixels = np.sqrt(dx_steps**2 + dy_steps**2)
    
    total_px = np.sum(ds_pixels)
    scale = track_distance / (total_px + 1e-6)
    ds_meters = ds_pixels * scale
    cum_dist = np.insert(np.cumsum(ds_meters), 0, 0.0)

    # Curvature (Length = N)
    dx = np.gradient(xs_smooth)
    dy = np.gradient(ys_smooth)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = np.abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)

    # 3. Physics Solver (N-Length)
    g = 9.81
    a_max = friction * g * 0.7
    b_max = friction * g * 0.9
    
    # Pass 1: Lateral Limit
    v_target = np.sqrt((friction * g) / (curvature + 1e-6))
    v_target = np.clip(v_target, 2.0, 92.0)

    # Pass 2: Forward
    v_phys = np.zeros_like(v_target)
    for i in range(1, len(v_phys)):
        dist = ds_meters[i-1]
        v_phys[i] = min(v_target[i], np.sqrt(v_phys[i-1]**2 + 2 * a_max * dist))

    # Pass 3: Backward
    for i in range(len(v_phys)-2, -1, -1):
        dist = ds_meters[i]
        v_phys[i] = min(v_phys[i], np.sqrt(v_phys[i+1]**2 + 2 * b_max * dist))

    # 4. Final Alignment Check
    v_kmh = v_phys * 3.6
    
    # CRITICAL FIX: Ensure v_kmh is exactly the same length as xs_smooth
    if len(v_kmh) != len(xs_smooth):
        v_kmh = np.interp(np.linspace(0, 1, len(xs_smooth)), np.linspace(0, 1, len(v_kmh)), v_kmh)

    # ------------------------------------------------
    # UI DISPLAY
    # ------------------------------------------------
    col1, col2 = st.columns(2)
    
    with col1:
        st.metric("Lap Time", f"{np.sum(ds_meters / np.maximum(v_phys[1:], 0.5)):.2f}s")
        # Map with heat-mapped speed
        fig_map = go.Figure(data=go.Scatter(
            x=xs_smooth, y=ys_smooth,
            mode='lines',
            line=dict(
                color=v_kmh, # Now guaranteed to match coordinate length
                colorscale='Turbo',
                width=6,
                colorbar=dict(title="km/h")
            )
        ))
        fig_map.update_layout(template="plotly_dark", title="Spatial Velocity Map")
        st.plotly_chart(fig_map, use_container_width=True)

    with col2:
        st.metric("Max Lateral G", f"{np.max((v_phys**2 * curvature)/g):.2f} G")
        fig_prof = go.Figure(data=go.Scatter(x=cum_dist, y=v_kmh, name="Speed", fill='tozeroy'))
        fig_prof.update_layout(template="plotly_dark", title="Velocity Profile", xaxis_title="Distance (m)", yaxis_title="km/h")
        st.plotly_chart(fig_prof, use_container_width=True)