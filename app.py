import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import random
import cv2
from PIL import Image
import tempfile

from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.stability import StabilityConstraint

from alignment_core.world_model.agent import AgentState
from alignment_core.world_model.environment import EnvironmentState
from alignment_core.world_model.world_state import WorldState


st.set_page_config(page_title="World Auditor", layout="wide")

st.title("World Auditor — AI Physics Safety Layer")


# ------------------------------
# Sidebar controls
# ------------------------------

st.sidebar.header("Scenario")

velocity = st.sidebar.slider("Velocity (m/s)", 0.0, 15.0, 4.0)
distance = st.sidebar.slider("Distance to obstacle (m)", 1.0, 50.0, 12.0)
friction = st.sidebar.slider("Surface friction", 0.1, 1.2, 0.6)
slope = st.sidebar.slider("Slope angle", -20.0, 20.0, 0.0)

load_weight = st.sidebar.slider("Load weight (kg)", 0.0, 2000.0, 200.0)

mass = st.sidebar.slider("Robot mass (kg)", 50.0, 3000.0, 500.0)

com_height = st.sidebar.slider("Center of mass height (m)", 0.1, 2.0, 0.5)
wheelbase = st.sidebar.slider("Wheelbase (m)", 0.5, 3.0, 1.2)

mode = st.sidebar.radio(
    "Mode",
    ["Safety Audit", "Monte Carlo Simulation", "Trajectory Prediction", "3D Visualizer", "AI Action Auditor", "Image / Video Physics Audit"]
)


# ------------------------------
# Engine builder
# ------------------------------

def build_world():

    agent = AgentState(
        id="robot",
        type="mobile",
        mass=mass,
        velocity=velocity,
        braking_force=5.0,
        max_deceleration=5.0,
        load_weight=load_weight,
        center_of_mass_height=com_height,
        wheelbase=wheelbase
    )

    env = EnvironmentState(
        friction=friction,
        slope=slope,
        distance_to_obstacles=distance
    )

    world_state = WorldState(
        agent=agent,
        environment=env
    )

    return world_state


def build_engine():

    return SafetyEngine([
        BrakingConstraint(),
        FrictionConstraint(),
        LoadConstraint(),
        StabilityConstraint()
    ])


# ------------------------------
# SAFETY AUDIT
# ------------------------------

if mode == "Safety Audit":

    st.header("Deterministic Safety Audit")

    if st.button("Run Audit"):

        engine = build_engine()
        world_state = build_world()

        results = engine.evaluate(world_state)

        for r in results:

            if r["safe"]:
                st.success(f"{r['constraint']} OK")
            else:
                st.error(f"{r['constraint']} FAILED")

            st.json(r)

elif mode == "AI Action Auditor":

    st.header("AI Action Safety Auditor")

    ai_velocity = st.slider("AI proposed velocity", 0.0, 15.0, 5.0)
    ai_distance = st.slider("Distance to obstacle", 1.0, 50.0, 10.0)

    if st.button("Audit AI Decision"):

        world_state = build_world()

        world_state.agent.velocity = ai_velocity
        world_state.environment.distance_to_obstacles = ai_distance

        engine = build_engine()

        from alignment_core.decision.action_auditor import ActionAuditor

        auditor = ActionAuditor(engine)

        result = auditor.audit(world_state, "move_forward")

        if result["allowed"]:

            st.success("AI ACTION APPROVED")

        else:

            st.error("AI ACTION BLOCKED")

            st.write(result["reason"])
# ------------------------------
# MONTE CARLO SIMULATION
# ------------------------------

elif mode == "Monte Carlo Simulation":

    st.header("Monte Carlo Risk Simulation")

    runs = st.slider("Simulation runs", 10, 500, 200)

    if st.button("Run Simulation"):

        collision_count = 0
        stop_distances = []

        for _ in range(runs):

            noisy_friction = friction + random.uniform(-0.15, 0.15)
            noisy_velocity = velocity + random.uniform(-1, 1)

            g = 9.81
            stopping_distance = (noisy_velocity ** 2) / (2 * noisy_friction * g)

            stop_distances.append(stopping_distance)

            if stopping_distance > distance:
                collision_count += 1

        prob = collision_count / runs

        st.metric("Collision probability", f"{prob*100:.2f}%")

        fig = go.Figure()
        fig.add_histogram(x=stop_distances)
        st.plotly_chart(fig, use_container_width=True)


# ------------------------------
# TRAJECTORY PREDICTION
# ------------------------------

elif mode == "Trajectory Prediction":

    st.header("Robot Trajectory Prediction")

    dt = 0.1
    g = 9.81

    v = velocity
    x = 0

    xs = []
    vs = []

    while v > 0:

        decel = friction * g
        v -= decel * dt
        x += v * dt

        xs.append(x)
        vs.append(v)

    fig = go.Figure()

    fig.add_trace(go.Scatter(
        x=xs,
        y=vs,
        mode="lines",
        name="Velocity"
    ))

    fig.update_layout(
        title="Velocity decay over distance",
        xaxis_title="Distance",
        yaxis_title="Velocity"
    )

    st.plotly_chart(fig, use_container_width=True)


# ------------------------------
# 3D VISUALIZATION
# ------------------------------

elif mode == "3D Visualizer":

    st.header("3D Motion Visualization")

    robot_path = np.linspace(0, distance, 50)

    fig = go.Figure()

    fig.add_trace(go.Scatter3d(
        x=robot_path,
        y=np.zeros_like(robot_path),
        z=np.zeros_like(robot_path),
        mode="lines+markers",
        marker=dict(size=4),
        name="Robot path"
    ))

    fig.add_trace(go.Scatter3d(
        x=[distance],
        y=[0],
        z=[0],
        mode="markers",
        marker=dict(size=8),
        name="Obstacle"
    ))

    fig.update_layout(
        scene=dict(
            xaxis_title="X",
            yaxis_title="Y",
            zaxis_title="Z"
        )
    )

    st.plotly_chart(fig, use_container_width=True)


# ------------------------------
# IMAGE / VIDEO PHYSICS AUDIT
# ------------------------------

elif mode == "Image / Video Physics Audit":

    st.header("Upload Media")

    uploaded = st.file_uploader("Upload image or video")

    if uploaded:

        filetype = uploaded.type

        if "image" in filetype:

            image = Image.open(uploaded)
            st.image(image)

            st.write("Basic heuristic analysis:")

            width, height = image.size

            if width > height:
                st.warning("Wide frame detected — possible high-speed scene")

            else:
                st.info("Frame geometry normal")

        elif "video" in filetype:

            temp = tempfile.NamedTemporaryFile(delete=False)
            temp.write(uploaded.read())

            cap = cv2.VideoCapture(temp.name)

            frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = cap.get(cv2.CAP_PROP_FPS)

            st.write(f"Frames: {frame_count}")
            st.write(f"FPS: {fps}")

            st.info("Video ingestion successful")

            cap.release()


# ------------------------------
# SAFETY ENVELOPE PLOT
# ------------------------------

st.divider()

st.header("Operational Safety Envelope")

velocities = np.linspace(0.1, 15, 50)
distances = []

g = 9.81

for v in velocities:

    d = (v ** 2) / (2 * friction * g)
    distances.append(d)

fig = go.Figure()

fig.add_trace(go.Scatter(
    x=velocities,
    y=distances,
    mode="lines",
    name="Stopping distance"
))

fig.add_hline(y=distance)

fig.update_layout(
    xaxis_title="Velocity",
    yaxis_title="Stopping distance"
)

st.plotly_chart(fig, use_container_width=True)