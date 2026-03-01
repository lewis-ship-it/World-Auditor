import streamlit as st
import math
import matplotlib.pyplot as plt
import json
from datetime import datetime

from alignment_core.engine import SafetyEngine, SafetyReport
from alignment_core.constraints import (
    BrakingConstraint,
    FrictionConstraint,
    LoadConstraint,
    StabilityConstraint,
)
from alignment_core.world_model import Agent, Environment, WorldState


st.set_page_config(page_title="AI Physics Safety Middleware", layout="wide")

st.title("AI Physics Commonsense Safety Layer")
st.markdown("Deterministic Middleware for AI-Controlled Robotics")


# -------------------------
# ROBOT PROFILES
# -------------------------

ROBOT_PROFILES = {
    "Warehouse Forklift": {
        "mass": 4000,
        "max_load": 2000,
        "center_of_mass_height": 1.2,
        "wheelbase": 2.0,
    },
    "Delivery Rover": {
        "mass": 80,
        "max_load": 20,
        "center_of_mass_height": 0.4,
        "wheelbase": 0.6,
    },
}

profile_name = st.sidebar.selectbox("Robot Profile", list(ROBOT_PROFILES.keys()))
profile = ROBOT_PROFILES[profile_name]


# -------------------------
# SIMULATOR INPUTS
# -------------------------

velocity = st.slider("Velocity (m/s)", 0.0, 15.0, 5.0)
distance = st.slider("Distance to Obstacle (m)", 0.5, 20.0, 5.0)
deceleration = st.slider("Max Deceleration (m/s²)", 0.5, 10.0, 2.0)
load_weight = st.slider("Load Weight (kg)", 0.0, 3000.0, 500.0)
friction = st.slider("Surface Friction Coefficient", 0.1, 1.0, 0.6)
slope = st.slider("Slope Angle (degrees)", 0.0, 45.0, 5.0)


if st.button("Run Full Safety Audit"):

    agent = Agent(
        velocity,
        deceleration,
        profile["mass"],
        load_weight,
        profile["max_load"],
        profile["center_of_mass_height"],
        profile["wheelbase"],
    )

    environment = Environment(distance, friction, slope)
    world_state = WorldState(agent, environment)

    engine = SafetyEngine()
    engine.register_constraint(BrakingConstraint())
    engine.register_constraint(FrictionConstraint())
    engine.register_constraint(LoadConstraint())
    engine.register_constraint(StabilityConstraint())

    results = engine.evaluate(world_state)
    report = SafetyReport(results)

    # -------------------------
    # DECISION
    # -------------------------

    st.subheader("Safety Decision")

    if report.is_safe():
        st.success("ALLOW: Action is physically feasible.")
    else:
        st.error("BLOCK: Physics violation detected.")

    # -------------------------
    # HUMAN EXPLANATION
    # -------------------------

    explanation = []

    for r in report.results:
        if r.violated:

            if r.name == "BrakingFeasibility":
                explanation.append(
                    "The robot cannot stop within the available distance."
                )

            if r.name == "FrictionSlipRisk":
                explanation.append(
                    "Surface friction is insufficient to counter downhill forces."
                )

            if r.name == "LoadOverCapacity":
                explanation.append(
                    "The load exceeds the robot's rated capacity."
                )

            if r.name == "TippingRisk":
                explanation.append(
                    "The slope exceeds the robot's tipping stability threshold."
                )

    if not explanation:
        explanation.append("All safety checks passed under current conditions.")

    st.write(" ".join(explanation))

    # -------------------------
    # RISK SCORE
    # -------------------------

    risk_score = sum(50 for r in report.results if r.violated)
    risk_score = min(risk_score, 100)

    st.subheader("Risk Score")
    st.progress(risk_score / 100)

    # -------------------------
    # TRAJECTORY GRAPH
    # -------------------------

    required_stop = (velocity ** 2) / (2 * deceleration)

    times = [t * 0.1 for t in range(50)]
    distances = [
        max(velocity * t - 0.5 * deceleration * t**2, 0) for t in times
    ]

    fig, ax = plt.subplots()
    ax.plot(times, distances)
    ax.axhline(distance, linestyle="--")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance Traveled (m)")
    ax.set_title("Stopping Trajectory")

    st.pyplot(fig)

    # -------------------------
    # EXPORT REPORT
    # -------------------------

    report_data = {
        "timestamp": str(datetime.utcnow()),
        "robot_profile": profile_name,
        "inputs": {
            "velocity": velocity,
            "distance": distance,
            "load": load_weight,
            "friction": friction,
            "slope": slope,
        },
        "violations": [r.name for r in report.results if r.violated],
    }

    st.download_button(
        label="Download Safety Report (JSON)",
        data=json.dumps(report_data, indent=4),
        file_name="safety_report.json",
        mime="application/json",
    )