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

# --- SIDEBAR (Restored all features) ---
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length (m)", value=420.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line", True)

uploaded_file = st.file_uploader("Upload Track Image", type=["jpg", "png", "jpeg"])

if uploaded_file:
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    with st.spinner("Processing High-Fidelity Physics..."):
        # 1. Extraction & Optimization
        points = extract_track_centerline(image)
        if use_racing_line:
            try:
                points = RacingLineOptimizer(points).optimize()
            except: st.warning("Optimizer failed, using centerline.")
        
        points = np.array(points)
        xs, ys = points[:, 0], points[:, 1]
        
        # 2. Smoothing
        win = min(len(xs)//7, 21) | 1
        xs = savgol_filter(xs, win, 3)
        ys = savgol_filter(ys, win, 3)

        # 3. Geometry (Curvature & Distance)
        dx = np.gradient(xs); dy = np.gradient(ys)
        ddx = np.gradient(dx); ddy = np.gradient(dy)
        curvature = np.abs(dx*ddy - dy*ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
        
        ds_px = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
        scale = track_distance / np.sum(ds_px)
        ds = ds_px * scale # Distance segments (N-1)
        
        # 4. The Speed Solver (Pass 1: Grip Limits)
        g = 9.81
        v_target = np.sqrt((friction * g) / (curvature + 1e-6))
        v_target = np.clip(v_target, 2.0, 92.0)

        # Pass 2 & 3: Accel/Brake (Using Sidebar Constants)
        # We ensure v_final matches the length of xs/ys
        v_final = np.zeros(len(xs))
        v_final[0] = 2.0 
        
        a_max = 4.2  # Restore your original logic constants
        b_max = 7.5

        # Forward
        for i in range(1, len(v_final)):
            dist = ds[i-1]
            v_final[i] = min(v_target[i], np.sqrt(v_final[i-1]**2 + 2 * a_max * dist))
        
        # Backward
        for i in range(len(v_final)-2, -1, -1):
            dist = ds[i]
            v_final[i] = min(v_final[i], np.sqrt(v_final[i+1]**2 + 2 * b_max * dist))

        # 5. RESTORING ALL SECONDARY MODELS (Energy/Thermals)
        energy_model = EnergyModel(mass=mass, drag_coeff=drag, area=frontal_area)
        soc = [100.0]
        tire_temp = [25.0]
        brake_temp = [25.0]
        cum_dist = np.insert(np.cumsum(ds), 0, 0)

        for i in range(1, len(v_final)):
            dt = ds[i-1] / max(v_final[i], 1.0)
            # Energy consumption
            pwr = energy_model.calculate_power(v_final[i], (v_final[i]-v_final[i-1])/dt)
            soc.append(soc[-1] - (pwr * dt / (battery_capacity * 36000)))
            
            # Simplified thermal logic (Restoring feature set)
            accel = (v_final[i]-v_final[i-1])/dt
            t_inc = max(0, accel * 0.5) if accel > 0 else abs(accel * 1.2)
            tire_temp.append(tire_temp[-1] + t_inc - 0.1)
            brake_temp.append(brake_temp[-1] + (abs(accel)*2 if accel < 0 else -0.5))

    # --- FULL DASHBOARD DISPLAY ---
    v_kmh = v_final * 3.6
    
    col1, col2 = st.columns([2, 1])
    with col1:
        st.subheader("Spatial Telemetry")
        fig_map = go.Figure(data=go.Scatter(
            x=xs, y=ys, mode='lines',
            line=dict(color=v_kmh, colorscale='Turbo', width=6, colorbar=dict(title="km/h"))
        ))
        fig_map.update_layout(template="plotly_dark", height=600)
        st.plotly_chart(fig_map, use_container_width=True)

    with col2:
        st.metric("Lap Time", f"{np.sum(ds/v_final[1:]):.2f}s")
        st.metric("Energy Used", f"{100 - soc[-1]:.2f}%")
        
        st.subheader("Velocity Profile")
        f_v = go.Figure(data=go.Scatter(x=cum_dist, y=v_kmh, fill='tozeroy'))
        f_v.update_layout(template="plotly_dark", height=300)
        st.plotly_chart(f_v, use_container_width=True)

    # Restoring Thermal/Energy Rows
    st.write("---")
    c_low1, c_low2 = st.columns(2)
    with c_low1:
        st.subheader("🔥 Component Thermals")
        f_t = go.Figure()
        f_t.add_trace(go.Scatter(x=cum_dist, y=tire_temp, name="Tires"))
        f_t.add_trace(go.Scatter(x=cum_dist, y=brake_temp, name="Brakes"))
        f_t.update_layout(template="plotly_dark")
        st.plotly_chart(f_t, use_container_width=True)
    
    with c_low2:
        st.subheader("⚡ Battery State of Charge")
        f_e = go.Figure(data=go.Scatter(x=cum_dist, y=soc, line=dict(color='lime')))
        f_e.update_layout(template="plotly_dark", yaxis=dict(range=[0,100]))
        st.plotly_chart(f_e, use_container_width=True)