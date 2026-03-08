import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel
import math

st.set_page_config(layout="wide")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER (RESTORED & UNRESTRICTED)
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")

# Replaced sliders with number_input to allow any value
mass = st.sidebar.number_input("Mass (kg)", value=1200.0)
drag = st.sidebar.number_input("Drag Coefficient", value=0.32)
frontal_area = st.sidebar.number_input("Frontal Area", value=2.2)
battery_capacity = st.sidebar.number_input("Battery Capacity (kWh)", value=75.0)

drive_type = st.sidebar.selectbox("Drive Type", ["FWD", "RWD", "AWD", "4WD"])

drive_mu_modifier = {
    "FWD": 0.92,
    "RWD": 0.96,
    "AWD": 1.05,
    "4WD": 1.08
}

friction = st.sidebar.number_input("Surface Grip μ", value=1.0)
friction *= drive_mu_modifier[drive_type]

# ------------------------------------------------
# TRACK SETTINGS
# ------------------------------------------------
st.sidebar.header("Track Settings")
track_width = st.sidebar.number_input("Track Width (m)", value=12.0)
track_distance = st.sidebar.number_input("Track Length Scale (m)", value=1200.0)
rotation = st.sidebar.number_input("Rotate Track (deg)", value=0.0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# FILE INPUT & SEPARATE PREVIEWS (RESTORED)
# ------------------------------------------------
uploaded = st.file_uploader("Upload Track Map")

if uploaded:
    # Read Image
    file_bytes = np.frombuffer(uploaded.read(), np.uint8)
    img = cv2.imdecode(file_bytes, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # PREVIEW SECTION (Separate from Final Analysis)
    prev_col1, prev_col2 = st.columns(2)
    with prev_col1:
        st.subheader("Original Upload")
        st.image(img_rgb, use_container_width=True)
    
    centerline = extract_track_centerline(img)

    if centerline is None:
        st.error("No track detected")
        st.stop()

    with prev_col2:
        st.subheader("AI Vision View")
        viz_ai = np.zeros_like(img)
        cv2.polylines(viz_ai, [centerline.astype(np.int32)], False, (0, 255, 255), 2)
        st.image(viz_ai, use_container_width=True)

    path = centerline.astype(float)

    # ----------------------------
    # ROTATION
    # ----------------------------
    theta = np.radians(rotation)
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    path = path @ rot

    # ----------------------------
    # RACING LINE
    # ----------------------------
    if optimize_line:
        optimizer = RacingLineOptimizer()
        path = optimizer.optimize(path)

    xs = path[:, 0]
    ys = path[:, 1]

    # ----------------------------
    # SCALE TRACK
    # ----------------------------
    d = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    total = np.sum(d)
    scale = track_distance / total
    xs *= scale
    ys *= scale

    # ----------------------------
    # CURVATURE
    # ----------------------------
    curvature = []
    for i in range(1, len(xs)-1):
        p1 = np.array([xs[i-1], ys[i-1]])
        p2 = np.array([xs[i], ys[i]])
        p3 = np.array([xs[i+1], ys[i+1]])
        k = np.linalg.norm(p3 - 2*p2 + p1)
        curvature.append(k)
    curvature = np.array([curvature[0]] + curvature + [curvature[-1]])

    # ----------------------------
    # SPEED PHYSICS
    # ----------------------------
    g = 9.81
    downforce = 0.5 * 1.225 * frontal_area * drag * (curvature + 1)
    grip = friction * g + downforce / mass
    max_speeds = np.sqrt(grip / (curvature + 1e-4))
    max_speeds = np.clip(max_speeds, 0, 200) # Increased clip for unrestriced input

    # ----------------------------
    # DISTANCES
    # ----------------------------
    distances = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    distances = np.append(distances, distances[-1])
    cumulative_distance = np.cumsum(distances)

    # ----------------------------
    # BRAKING
    # ----------------------------
    braking = BrakingModel(friction)
    braking_zones = []
    for i in range(len(max_speeds)-1):
        if max_speeds[i+1] < max_speeds[i]:
            braking_zones.append(i)

    # ----------------------------
    # ENERGY (FIXED: NON-CUMULATIVE PLOTTING)
    # ----------------------------
    energy_model = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy = energy_model.energy_used(max_speeds, distances)
    regen = energy_model.regen_energy(np.abs(np.diff(max_speeds)), 0.6)
    regen = np.append(regen, 0)
    net_energy = energy - regen

    # ----------------------------
    # BATTERY (SOC TRACKING)
    # ----------------------------
    battery_j = battery_capacity * 3.6e6
    soc = 100 - (np.cumsum(net_energy) / battery_j * 100)
    soc = np.clip(soc, 0, 100)

    # ----------------------------
    # THERMAL MODEL (FIXED: AMBIENT START)
    # ----------------------------
    tire_temp, brake_temp = [], []
    tt, bt = 25.0, 30.0 # Ambient starting point
    for v in max_speeds:
        tt += 0.1 * (v * 0.5) - 0.02 * (tt - 25)
        bt += 0.15 * (v * 0.8) - 0.03 * (bt - 30)
        tire_temp.append(tt)
        brake_temp.append(bt)
    tire_temp = np.array(tire_temp)
    brake_temp = np.array(brake_temp)

    # ----------------------------
    # LATERAL G
    # ----------------------------
    lat_g = max_speeds**2 * curvature / g

    # ------------------------------------------------
    # MAP VISUALIZATION (FIXED PLOTLY TRACE)
    # ------------------------------------------------
    st.divider()
    col1, col2 = st.columns([2, 1])

    with col1:
        fig = go.Figure()
        # FIXED: color is assigned to marker, not line, to prevent crash
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode="lines+markers",
            line=dict(width=2, color="rgba(255,255,255,0.3)"),
            marker=dict(
                size=5, 
                color=max_speeds, 
                colorscale="Turbo", 
                showscale=True,
                colorbar=dict(title="m/s")
            ),
            hovertemplate="Speed %{marker.color:.1f} m/s"
        ))

        if len(braking_zones) > 0:
            fig.add_trace(go.Scatter(
                x=xs[braking_zones], y=ys[braking_zones],
                mode="markers", marker=dict(color="red", size=7, symbol="x"),
                name="Braking"
            ))

        fig.update_layout(
            template="plotly_dark", height=600,
            title="Racing Line Speed Map", yaxis=dict(scaleanchor="x")
        )
        st.plotly_chart(fig, use_container_width=True)

    # ------------------------------------------------
    # METRICS
    # ------------------------------------------------
    with col2:
        lap_length = np.sum(distances)
        lap_time = np.sum(distances / (max_speeds + 1e-4))
        st.metric("Track Length", f"{lap_length:.1f} m")
        st.metric("Lap Time", f"{lap_time:.2f} s")
        st.metric("Max Speed", f"{np.max(max_speeds):.1f} m/s")
        st.metric("Energy/Lap", f"{np.sum(net_energy)/1000:.2f} kJ")

    # ------------------------------------------------
    # TELEMETRY GRAPHS (RESTORED ALL GRAPHS)
    # ------------------------------------------------
    st.subheader("Speed & Lateral G")
    fig_speed = go.Figure()
    fig_speed.add_trace(go.Scatter(x=cumulative_distance, y=max_speeds, name="Speed"))
    fig_speed.add_trace(go.Scatter(x=cumulative_distance, y=lat_g, name="Lateral G", yaxis="y2"))
    fig_speed.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", title="G-Force"))
    st.plotly_chart(fig_speed, use_container_width=True)

    st.subheader("Thermal Telemetry")
    fig_temp = go.Figure()
    fig_temp.add_trace(go.Scatter(x=cumulative_distance, y=tire_temp, name="Tire Temp"))
    fig_temp.add_trace(go.Scatter(x=cumulative_distance, y=brake_temp, name="Brake Temp"))
    fig_temp.update_layout(template="plotly_dark", title="Thermal Profile (Rise from Ambient)")
    st.plotly_chart(fig_temp, use_container_width=True)

    st.subheader("Energy + Battery")
    fig_energy = go.Figure()
    fig_energy.add_trace(go.Scatter(x=cumulative_distance, y=net_energy/1000, name="Energy Delta (kJ)"))
    fig_energy.add_trace(go.Scatter(x=cumulative_distance, y=soc, name="Battery SOC (%)", yaxis="y2"))
    fig_energy.update_layout(template="plotly_dark", yaxis2=dict(overlaying="y", side="right", title="SOC %"))
    st.plotly_chart(fig_energy, use_container_width=True)