import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image

from alignment_core.perception.track_extractor import extract_track_centerline
from alignment_core.planning.racing_line import RacingLineOptimizer
from alignment_core.physics.energy_model import EnergyModel
from alignment_core.physics.braking_model import BrakingModel

st.set_page_config(layout="wide")

st.title("🏁 Ultimate Track Speed Analyzer")

# -------------------------------
# SIDEBAR VEHICLE SETTINGS
# -------------------------------

st.sidebar.header("Vehicle")

mass = st.sidebar.slider("Vehicle Mass (kg)", 200, 2000, 1200)
friction = st.sidebar.slider("Surface Grip μ", 0.3, 1.5, 1.0)
drag = st.sidebar.slider("Drag Coefficient", 0.2, 0.6, 0.32)

battery_capacity = st.sidebar.slider("Battery Capacity (kWh)", 20, 120, 75)

st.sidebar.header("AI Optimization")

optimize_line = st.sidebar.checkbox("Optimize Racing Line", True)

# -------------------------------
# MAP INPUT
# -------------------------------

uploaded = st.file_uploader("Upload Track Map")

if uploaded:

    img = Image.open(uploaded).convert("RGB")
    img_np = np.array(img)

    # -------------------------------
    # TRACK EXTRACTION
    # -------------------------------

    centerline = extract_track_centerline(img_np)

    if centerline is None:
        st.error("No track detected")
        st.stop()

    path = centerline.astype(float)

    # -------------------------------
    # AI RACING LINE
    # -------------------------------

    if optimize_line:

        optimizer = RacingLineOptimizer()

        path = optimizer.optimize(path)

    xs = path[:,0]
    ys = path[:,1]

    # -------------------------------
    # CURVATURE
    # -------------------------------

    curvature = []

    for i in range(1,len(xs)-1):

        p1 = np.array([xs[i-1],ys[i-1]])
        p2 = np.array([xs[i],ys[i]])
        p3 = np.array([xs[i+1],ys[i+1]])

        k = np.linalg.norm(p3 - 2*p2 + p1)

        curvature.append(k)

    curvature = np.array([curvature[0]] + curvature + [curvature[-1]])

    # -------------------------------
    # SPEED LIMIT FROM PHYSICS
    # -------------------------------

    g = 9.81

    max_speeds = np.sqrt((friction*g)/(curvature + 1e-4))

    max_speeds = np.clip(max_speeds,0,120)

    # -------------------------------
    # DISTANCE ALONG TRACK
    # -------------------------------

    distances = np.sqrt(np.diff(xs)**2 + np.diff(ys)**2)
    distances = np.append(distances,distances[-1])

    cumulative_distance = np.cumsum(distances)

    # -------------------------------
    # BRAKING MODEL
    # -------------------------------

    braking = BrakingModel(friction)

    braking_zones = []

    for i in range(len(max_speeds)-1):

        v1 = max_speeds[i]
        v2 = max_speeds[i+1]

        if v2 < v1:

            stop_dist = braking.braking_distance(v1)

            braking_zones.append(i)

    # -------------------------------
    # ENERGY MODEL
    # -------------------------------

    energy_model = EnergyModel(
        vehicle_mass=mass,
        drag_coeff=drag
    )

    energy = energy_model.energy_used(max_speeds, distances)

    regen = energy_model.regen_energy(np.abs(np.diff(max_speeds)),0.6)

    regen = np.append(regen,0)

    net_energy = energy - regen

    # -------------------------------
    # BATTERY SOC
    # -------------------------------

    battery_joules = battery_capacity * 3.6e6

    soc = []

    remaining = battery_joules

    for e in net_energy:

        remaining -= e

        soc.append(max(remaining / battery_joules * 100,0))

    soc = np.array(soc)

    # -------------------------------
    # THERMAL MODELS
    # -------------------------------

    tire_temp = []
    brake_temp = []

    tt = 60
    bt = 120

    for v in max_speeds:

        tt += v*0.25 - 0.04*tt
        bt += v*0.45 - 0.08*bt

        tire_temp.append(tt)
        brake_temp.append(bt)

    tire_temp = np.array(tire_temp)
    brake_temp = np.array(brake_temp)

    # -------------------------------
    # MAIN MAP
    # -------------------------------

    col1,col2 = st.columns([2,1])

    with col1:

        fig = go.Figure()

        fig.add_trace(go.Scatter(
            x=xs,
            y=ys,
            mode="lines",
            line=dict(width=4,color="cyan"),
            name="Racing Line"
        ))

        fig.add_trace(go.Scatter(
            x=xs[braking_zones],
            y=ys[braking_zones],
            mode="markers",
            marker=dict(color="red",size=6),
            name="Braking Zones"
        ))

        fig.update_layout(
            images=[dict(
                source=img,
                xref="x",
                yref="y",
                x=0,
                y=img_np.shape[0],
                sizex=img_np.shape[1],
                sizey=img_np.shape[0],
                sizing="stretch",
                opacity=0.5,
                layer="below"
            )],
            template="plotly_dark",
            yaxis=dict(scaleanchor="x"),
            margin=dict(l=0,r=0,t=0,b=0)
        )

        st.plotly_chart(fig,use_container_width=True)

    # -------------------------------
    # METRICS
    # -------------------------------

    with col2:

        lap_length = np.sum(distances)

        lap_time = np.sum(distances / (max_speeds + 1e-4))

        st.metric("Track Length", f"{lap_length:.1f} m")

        st.metric("Estimated Lap Time", f"{lap_time:.2f} s")

        st.metric("Max Speed", f"{np.max(max_speeds):.1f} m/s")

        st.metric("Energy / Lap", f"{np.sum(net_energy)/1000:.2f} kJ")

    # -------------------------------
    # TELEMETRY GRAPHS
    # -------------------------------

    st.subheader("Speed Profile")

    fig_speed = go.Figure()

    fig_speed.add_trace(go.Scatter(
        x=cumulative_distance,
        y=max_speeds,
        mode="lines"
    ))

    st.plotly_chart(fig_speed,use_container_width=True)

    st.subheader("Brake Temperature")

    fig_bt = go.Figure()

    fig_bt.add_trace(go.Scatter(
        x=cumulative_distance,
        y=brake_temp
    ))

    st.plotly_chart(fig_bt,use_container_width=True)

    st.subheader("Tire Temperature")

    fig_tt = go.Figure()

    fig_tt.add_trace(go.Scatter(
        x=cumulative_distance,
        y=tire_temp
    ))

    st.plotly_chart(fig_tt,use_container_width=True)

    st.subheader("Energy Usage")

    fig_energy = go.Figure()

    fig_energy.add_trace(go.Scatter(
        x=cumulative_distance,
        y=net_energy
    ))

    st.plotly_chart(fig_energy,use_container_width=True)

    st.subheader("Battery State of Charge")

    fig_soc = go.Figure()

    fig_soc.add_trace(go.Scatter(
        x=cumulative_distance,
        y=soc
    ))

    st.plotly_chart(fig_soc,use_container_width=True)