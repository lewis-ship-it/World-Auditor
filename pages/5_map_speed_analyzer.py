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
# SIDEBAR VEHICLE BUILDER (FULL RESTORED)
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")
mass = st.sidebar.number_input("Mass (kg)", value=1470.0) 
drag = st.sidebar.number_input("Drag Coefficient", value=0.35)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.1)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["RWD", "FWD", "AWD", "4WD"])
friction = st.sidebar.number_input("Surface Grip μ", value=1.0) 

st.sidebar.header("Track Settings")
track_distance = st.sidebar.number_input("Track Length (m)", value=420.0)
use_racing_line = st.sidebar.checkbox("Optimize Racing Line", True)

uploaded_file = st.file_uploader("Upload Track Image", type=["jpg", "png", "jpeg"])

if uploaded_file:
    file_bytes = np.asarray(bytearray(uploaded_file.read()), dtype=np.uint8)
    image = cv2.imdecode(file_bytes, 1)
    
    with st.spinner("Analyzing high-fidelity physics..."):
        # 1. Track Extraction
        points = extract_track_centerline(image)
        if use_racing_line:
            try:
                points = RacingLineOptimizer(points).optimize()
            except:
                st.warning("Optimizer failed. Using raw centerline.")
        
        points = np.array(points)
        xs, ys = points[:, 0], points[:, 1]
        
        # 2. Advanced Smoothing (Prevents 'infinite' curvature spikes)
        window = min(len(xs)//5, 21) | 1
        xs = savgol_filter(xs, window, 3)
        ys = savgol_filter(ys, window, 3)

        # 3. Geometry & Distances
        dx = np.gradient(xs); dy = np.gradient(ys)
        ddx = np.gradient(dx); ddy = np.gradient(dy)
        curvature = np.abs(dx*ddy - dy*ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
        
        ds_px = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
        total_px = np.sum(ds_px)
        scale = track_distance / (total_px + 1e-6)
        ds = ds_px * scale # Meters between points
        cum_dist = np.insert(np.cumsum(ds), 0, 0)

        # 4. THREE-PASS PHYSICS SOLVER
        g = 9.81
        v_target = np.sqrt((friction * g) / (curvature + 1e-6))
        v_target = np.clip(v_target, 2.0, 92.0)

        v_final = np.zeros(len(xs))
        v_final[0] = 2.0 # Initial velocity
        
        # Pass 2: Forward (Acceleration)
        a_max = 4.2 
        for i in range(1, len(v_final)):
            dist = ds[i-1]
            v_final[i] = min(v_target[i], np.sqrt(v_final[i-1]**2 + 2 * a_max * dist))
        
        # Pass 3: Backward (Braking)
        b_max = 7.5
        for i in range(len(v_final)-2, -1, -1):
            dist = ds[i]
            v_final[i] = min(v_final[i], np.sqrt(v_final[i+1]**2 + 2 * b_max * dist))

        # 5. ENERGY & THERMAL MODELS (FIXED ARGS)
        en_model = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
        
        soc = [100.0]
        tire_temp = [25.0]
        brake_temp = [25.0]

        for i in range(1, len(v_final)):
            v = v_final[i]
            d = ds[i-1]
            t_step = d / max(v, 1.0)
            
            # Power Calculation
            p_watts = en_model.power_usage(v)
            energy_kwh = (p_watts * t_step) / 3600000
            soc.append(max(0, soc[-1] - (energy_kwh / battery_capacity * 100)))
            
            # Thermal Estimation
            dv = v - v_final[i-1]
            tire_temp.append(tire_temp[-1] + (abs(dv) * 0.12) - 0.04)
            brake_temp.append(brake_temp[-1] + (abs(dv)*0.6 if dv < 0 else -0.15))

    # ------------------------------------------------
    # UI RENDERING (ALL ROWS RESTORED)
    # ------------------------------------------------
    v_kmh = v_final * 3.6
    
    col1, col2 = st.columns([2, 1])
    with col1:
        st.subheader("Spatial Velocity Heatmap")
        fig_map = go.Figure(data=go.Scatter(
            x=xs, y=ys, mode='lines',
            line=dict(color=v_kmh, colorscale='Turbo', width=6, colorbar=dict(title="km/h"))
        ))
        fig_map.update_layout(template="plotly_dark", height=600)
        st.plotly_chart(fig_map, use_container_width=True)

    with col2:
        st.metric("Estimated Lap Time", f"{np.sum(ds/np.maximum(v_final[1:], 1.0)):.2f}s")
        st.metric("Final Battery %", f"{soc[-1]:.1f}%")
        
        st.subheader("Velocity Profile")
        f_v = go.Figure(data=go.Scatter(x=cum_dist, y=v_kmh, fill='tozeroy', line=dict(color='cyan')))
        f_v.update_layout(template="plotly_dark", height=400, yaxis_title="km/h")
        st.plotly_chart(f_v, use_container_width=True)

    # Secondary Telemetry Row
    st.write("---")
    c_low1, c_low2 = st.columns(2)
    with c_low1:
        st.subheader("🔥 Component Thermals")
        f_t = go.Figure()
        f_t.add_trace(go.Scatter(x=cum_dist, y=tire_temp, name="Tires", line=dict(color='orange')))
        f_t.add_trace(go.Scatter(x=cum_dist, y=brake_temp, name="Brakes", line=dict(color='red')))
        f_t.update_layout(template="plotly_dark")
        st.plotly_chart(f_t, use_container_width=True)
    
    with c_low2:
        st.subheader("⚡ Battery Consumption")
        f_e = go.Figure(data=go.Scatter(x=cum_dist, y=soc, fill='tozeroy', line=dict(color='lime')))
        f_e.update_layout(template="plotly_dark", yaxis=dict(range=[0, 105]), yaxis_title="SOC %")
        st.plotly_chart(f_e, use_container_width=True)