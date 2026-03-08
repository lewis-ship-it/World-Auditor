import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel
import math

st.set_page_config(layout="wide", page_title="Ultimate Track Speed Analyzer")

st.title("🏁 Ultimate Track Speed Analyzer")

# ------------------------------------------------
# SIDEBAR VEHICLE BUILDER
# ------------------------------------------------
st.sidebar.header("Vehicle Builder")

mass = st.sidebar.slider("Mass (kg)", 200, 2000, 1200)
drag = st.sidebar.slider("Drag Coefficient", 0.2, 0.7, 0.32)
frontal_area = st.sidebar.slider("Frontal Area", 1.2, 3.0, 2.2)
battery_capacity = st.sidebar.slider("Battery Capacity (kWh)", 20, 120, 75)

drive_type = st.sidebar.selectbox("Drive Type", ["FWD", "RWD", "AWD", "4WD"])

drive_mu_modifier = {
    "FWD": 0.92,
    "RWD": 0.96,
    "AWD": 1.05,
    "4WD": 1.08
}

friction = st.sidebar.slider("Surface Grip μ", 0.3, 1.5, 1.0)
friction *= drive_mu_modifier[drive_type]

# ------------------------------------------------
# TRACK SETTINGS
# ------------------------------------------------
st.sidebar.header("Track Settings")
track_width = st.sidebar.slider("Track Width (m)", 6, 20, 12)
track_distance = st.sidebar.slider("Track Length Scale (m)", 200, 7000, 1200)
rotation = st.sidebar.slider("Rotate Track (deg)", -180, 180, 0)
optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# ------------------------------------------------
# FILE INPUT & PERCEPTION
# ------------------------------------------------
uploaded = st.file_uploader("Upload Track Map")

if uploaded:
    img = cv2.imdecode(np.frombuffer(uploaded.read(), np.uint8), 1)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    centerline = extract_track_centerline(img)

    if centerline is None:
        st.error("No track detected")
        st.stop()

    path = centerline.astype(float)

    # ROTATION
    theta = np.radians(rotation)
    rot = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])
    path = path @ rot

    # RACING LINE OPTIMIZATION
    if optimize_line:
        optimizer = RacingLineOptimizer()
        path = optimizer.optimize(path)

    xs = path[:, 0]
    ys = path[:, 1]

    # SCALE TRACK
    d = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    total = np.sum(d)
    scale = track_distance / total
    xs *= scale
    ys *= scale

    # CURVATURE CALCULATION
    curvature = []
    for i in range(1, len(xs)-1):
        p1 = np.array([xs[i-1], ys[i-1]])
        p2 = np.array([xs[i], ys[i]])
        p3 = np.array([xs[i+1], ys[i+1]])
        k = np.linalg.norm(p3 - 2*p2 + p1)
        curvature.append(k)
    
    curvature = np.array([curvature[0]] + curvature + [curvature[-1]])

    # SPEED PHYSICS
    g = 9.81
    downforce = 0.5 * 1.225 * frontal_area * drag * (curvature + 1)
    grip = friction * g + downforce / mass
    max_speeds = np.sqrt(grip / (curvature + 1e-4))
    max_speeds = np.clip(max_speeds, 0, 120)

    # DISTANCE MAPPING
    distances = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    distances = np.append(distances, distances[-1])
    cumulative_distance = np.cumsum(distances)

    # BRAKING MODEL
    braking_model = BrakingModel(friction)
    braking_zones = []
    for i in range(len(max_speeds)-1):
        if max_speeds[i+1] < max_speeds[i]:
            braking_zones.append(i)

    # ENERGY MODEL
    energy_model = EnergyModel(vehicle_mass=mass, drag_coeff=drag, frontal_area=frontal_area)
    energy = energy_model.energy_used(max_speeds, distances)
    regen = energy_model.regen_energy(np.abs(np.diff(max_speeds)), 0.6)
    regen = np.append(regen, 0)
    net_energy = energy - regen

    # BATTERY SOC
    battery_j = battery_capacity * 3.6e6
    soc = []
    remaining = battery_j
    for e in net_energy:
        remaining -= e
        soc.append(max(remaining / battery_j * 100, 0))
    soc = np.array(soc)

    # THERMAL MODEL
    tire_temp, brake_temp = [], []
    tt, bt = 25, 40
    for v in max_speeds:
        tt += 0.06 * (v * 4 - tt)
        bt += 0.1 * (v * 6 - bt)
        tire_temp.append(tt)
        brake_temp.append(bt)
    tire_temp = np.array(tire_temp)
    brake_temp = np.array(brake_temp)

    # LATERAL G
    lat_g = (max_speeds**2 * curvature) / g

    # ------------------------------------------------
    # MAP VISUALIZATION (FIXED section)
    # ------------------------------------------------
    col1, col2 = st.columns([2, 1])

    with col1:
        fig = go.Figure()

        # FIXED: Array mapping is moved to 'marker' to avoid Plotly ValueError
        fig.add_trace(go.Scatter(
            x=xs,
            y=ys,
            mode="lines+markers",
            line=dict(width=2, color="rgba(255,255,255,0.3)"),
            marker=dict(
                size=5,
                color=max_speeds,
                colorscale="Turbo",
                showscale=True,
                colorbar=dict(title="Speed (m/s)")
            ),
            hovertemplate="Speed: %{marker.color:.1f} m/s<extra></extra>"
        ))

        if len(braking_zones) > 0:
            fig.add_trace(go.Scatter(
                x=xs[braking_zones],
                y=ys[braking_zones],
                mode="markers",
                marker=dict(color="red", size=7, symbol="x"),
                name="Braking"
            ))

        fig.update_layout(
            template="plotly_dark",
            height=600,
            title="Racing Line Speed Map",
            yaxis=dict(scaleanchor="x"),
            xaxis=dict(showgrid=False),
            margin=dict(l=0, r=0, t=40, b=0)
        )
        st.plotly_chart(fig, use_container_width=True)

    # ------------------------------------------------
    # PERFORMANCE METRICS
    # ------------------------------------------------
    with col2:
        lap_length = np.sum(distances)
        lap_time = np.sum(distances / (max_speeds + 1e-4))
        st.metric("Track Length", f"{lap_length:.1f} m")
        st.metric("Lap Time", f"{lap_time:.2f} s")
        st.metric("Max Speed", f"{np.max(max_speeds):.1f} m/s")
        st.metric("Energy/Lap", f"{np.sum(net_energy)/1000:.2f} kJ")

    # ------------------------------------------------
    # TELEMETRY GRAPHS
    # ------------------------------------------------
    st.subheader("Speed & Lateral G")
    fig_speed = go.Figure()
    fig_speed.add_trace(go.Scatter(x=cumulative_distance, y=max_speeds, name="Speed (m/s)"))
    fig_speed.add_trace(go.Scatter(x=cumulative_distance, y=lat_g, name="Lateral G"))
    fig_speed.update_layout(template="plotly_dark", height=400)
    st.plotly_chart(fig_speed, use_container_width=True)

    st.subheader("Thermal Telemetry")
    fig_temp = go.Figure()
    fig_temp.add_trace(go.Scatter(x=cumulative_distance, y=tire_temp, name="Tire Temp (°C)"))
    fig_temp.add_trace(go.Scatter(x=cumulative_distance, y=brake_temp, name="Brake Temp (°C)"))
    fig_temp.update_layout(template="plotly_dark", height=400)
    st.plotly_chart(fig_temp, use_container_width=True)

    st.subheader("Energy + Battery Status")
    fig_energy = go.Figure()
    fig_energy.add_trace(go.Scatter(x=cumulative_distance, y=net_energy, name="Energy Delta (J)"))
    fig_energy.add_trace(go.Scatter(x=cumulative_distance, y=soc, name="Battery SOC (%)"))
    fig_energy.update_layout(template="plotly_dark", height=400)
    st.plotly_chart(fig_energy, use_container_width=True)