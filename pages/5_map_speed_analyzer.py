import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from scipy.signal import savgol_filter

st.set_page_config(layout="wide", page_title="World-Auditor | Fixed Physics")

st.title("🏁 Track Speed Analyzer: Final Stability Fix")

# --- Sidebar Inputs ---
st.sidebar.header("Vehicle & Track")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0)
mu = st.sidebar.number_input("Surface Grip μ", value=0.9) # Friction
track_len = st.sidebar.number_input("Track Length (m)", value=500.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line", False)

uploaded_file = st.file_uploader("Upload Track Map", type=["jpg", "png", "jpeg"])

if uploaded_file:
    # 1. Image Processing
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    with st.spinner("Calculating Physics..."):
        points = extract_track_centerline(image)
        if use_racing_line:
            try:
                points = RacingLineOptimizer(points).optimize()
            except: pass
        
        # 2. Geometry (Smoothing Coordinates)
        points = np.array(points)
        x, y = points[:, 0], points[:, 1]
        win = min(len(x)//5, 15) | 1 # Ensure odd
        xs, ys = savgol_filter(x, win, 3), savgol_filter(y, win, 3)

        # 3. Calculate Curvature per Point
        dx = np.gradient(xs); dy = np.gradient(ys)
        ddx = np.gradient(dx); ddy = np.gradient(dy)
        curvature = np.abs(dx*ddy - dy*ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)

        # 4. Calculate Distances between points
        ds_px = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
        scale = track_len / np.sum(ds_px)
        ds = ds_px * scale # Distance in meters between points
        
        # 5. The Physics Solver
        g = 9.81
        v_max_corner = np.sqrt((mu * g) / (curvature + 1e-6))
        v_max_corner = np.clip(v_max_corner, 2.0, 80.0) # Cap at ~280km/h
        
        # Acceleration/Braking limits (m/s^2)
        a_limit = mu * g * 0.6 
        b_limit = mu * g * 0.8
        
        # Forward Pass (Acceleration)
        v_final = np.zeros_like(v_max_corner)
        v_final[0] = 5.0 # Start with some speed
        for i in range(1, len(v_final)):
            v_accel = np.sqrt(v_final[i-1]**2 + 2 * a_limit * ds[i-1])
            v_final[i] = min(v_max_corner[i], v_accel)
            
        # Backward Pass (Braking)
        for i in range(len(v_final)-2, -1, -1):
            v_brake = np.sqrt(v_final[i+1]**2 + 2 * b_limit * ds[i])
            v_final[i] = min(v_final[i], v_brake)

    # --- UI & VISUALIZATION ---
    v_kmh = v_final * 3.6
    
    # Check for Plotly alignment
    if len(v_kmh) != len(xs):
        st.error("Physics alignment mismatch. Try a clearer image.")
    else:
        col1, col2 = st.columns(2)
        with col1:
            # Map View
            fig = go.Figure(data=go.Scatter(
                x=xs, y=ys, mode='lines',
                line=dict(color=v_kmh, colorscale='Turbo', width=6, colorbar=dict(title="km/h"))
            ))
            fig.update_layout(template="plotly_dark", title="Speed Map (Physical Limit)")
            st.plotly_chart(fig, use_container_width=True)
            
        with col2:
            # Velocity Chart
            dist_axis = np.insert(np.cumsum(ds), 0, 0)
            fig2 = go.Figure(data=go.Scatter(x=dist_axis, y=v_kmh, name="Velocity"))
            fig2.update_layout(template="plotly_dark", title="Velocity Profile", xaxis_title="Distance (m)", yaxis_title="km/h")
            st.plotly_chart(fig2, use_container_width=True)