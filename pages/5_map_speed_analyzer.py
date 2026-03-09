import streamlit as st
import numpy as np
import cv2
import pandas as pd
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from scipy.signal import savgol_filter

st.set_page_config(layout="wide", page_title="World-Auditor | Physics Lab")

st.title("🏁 Ultimate Track Speed Analyzer (Fixed Alignment)")

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
        # 1. Extraction
        raw_points = extract_track_centerline(image)
        if raw_points is None or len(raw_points) < 10:
            st.error("Track path too complex or not found.")
            st.stop()

        points = raw_points
        if use_racing_line:
            try:
                optimizer = RacingLineOptimizer(raw_points)
                optimized = optimizer.optimize()
                if optimized is not None: points = np.array(optimized)
            except:
                st.warning("Racing line optimization failed. Using centerline.")

    # 2. Geometry Smoothing (Fixes "Jagged" Curvature)
    points = np.array(points)
    xs, ys = points[:, 0], points[:, 1]
    
    # Dynamic window for Savitzky-Golay filter
    win = min(len(xs) // 7, 21)
    if win % 2 == 0: win += 1
    if win < 5: win = 5
    
    xs_s = savgol_filter(xs, win, 3)
    ys_s = savgol_filter(ys, win, 3)

    # 3. Physics Calculations (N-Length)
    dx = np.gradient(xs_s)
    dy = np.gradient(ys_s)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    
    # Curvature (k)
    curvature = np.abs(dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
    
    # Distance scaling
    ds_px = np.sqrt(np.diff(xs_s)**2 + np.diff(ys_s)**2)
    total_px = np.sum(ds_px)
    scale = track_distance / (total_px + 1e-6)
    ds_m = ds_px * scale
    cum_dist = np.insert(np.cumsum(ds_m), 0, 0)

    # 4. Three-Pass Solver
    g = 9.81
    a_max = friction * g * 0.7 
    b_max = friction * g * 0.9 
    
    # Pass 1: Max Cornering Speed
    v_limit = np.sqrt((friction * g) / (curvature + 1e-6))
    v_limit = np.clip(v_limit, 1.0, 92.0)

    # Pass 2: Forward (Accel)
    v_final = np.zeros_like(v_limit)
    for i in range(1, len(v_final)):
        v_final[i] = min(v_limit[i], np.sqrt(v_final[i-1]**2 + 2 * a_max * ds_m[i-1]))

    # Pass 3: Backward (Braking)
    for i in range(len(v_final)-2, -1, -1):
        v_final[i] = min(v_final[i], np.sqrt(v_final[i+1]**2 + 2 * b_max * ds_m[i]))

    v_kmh = v_final * 3.6

    # ------------------------------------------------
    # MANDATORY ALIGNMENT FIX FOR PLOTLY
    # ------------------------------------------------
    # This ensures x, y, and color arrays are EXACTLY the same length
    if len(v_kmh) != len(xs_s):
        v_kmh = np.interp(np.arange(len(xs_s)), np.arange(len(v_kmh)), v_kmh)

    # ------------------------------------------------
    # DASHBOARD
    # ------------------------------------------------
    c1, c2 = st.columns(2)
    with c1:
        st.metric("Avg Speed", f"{np.mean(v_kmh):.1f} km/h")
        fig_map = go.Figure(data=go.Scatter(
            x=xs_s, y=ys_s,
            mode='lines',
            line=dict(
                color=v_kmh, # Length now guaranteed to match
                colorscale='Turbo',
                width=6,
                colorbar=dict(title="km/h")
            )
        ))
        fig_map.update_layout(template="plotly_dark", title="Spatial Speed Heatmap", margin=dict(l=0,r=0,b=0,t=40))
        st.plotly_chart(fig_map, use_container_width=True)

    with c2:
        st.metric("Lap Time", f"{np.sum(ds_m / np.maximum(v_final[1:], 1.0)):.2f}s")
        fig_prof = go.Figure(data=go.Scatter(x=cum_dist, y=v_kmh, fill='tozeroy', line=dict(color='#00d4ff')))
        fig_prof.update_layout(template="plotly_dark", title="Velocity over Distance", xaxis_title="Meters", yaxis_title="km/h")
        st.plotly_chart(fig_prof, use_container_width=True)