import streamlit as st
import pandas as pd
import plotly.graph_objects as go

from alignment_core.engine.safety_engine import SafetyEngine
from alignment_core.constraints.braking import BrakingConstraint
from alignment_core.constraints.load import LoadConstraint
from alignment_core.constraints.friction import FrictionConstraint
from alignment_core.constraints.stability import StabilityConstraint

from alignment_core.state.agent import AgentState
from alignment_core.state.environment import EnvironmentState
from alignment_core.state.world_state import WorldState


st.set_page_config(page_title="Safety Audit", layout="wide")

st.title("🛡️ Robot Physics Safety Audit")

st.markdown("""
This tool evaluates whether a robot's planned action violates **real-world physics constraints**.

It analyzes:

• braking feasibility  
• traction / friction  
• load safety  
• stability and tipping risk  

The result is a **robot safety score and explanation**.
""")

st.divider()

# -----------------------------
# SIDEBAR INPUTS
# -----------------------------

st.sidebar.header("Robot Profile")

robot_name = st.sidebar.text_input("Robot Name", "WarehouseBot")

mass = st.sidebar.number_input("Robot Mass (kg)", 1.0, 10000.0, 1200.0)

wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 5.0, 1.2)

com_height = st.sidebar.number_input(
    "Center of Mass Height (m)", 0.05, 3.0, 0.6
)

max_load = st.sidebar.number_input("Maximum Load (kg)", 0.0, 10000.0, 1500.0)

max_speed = st.sidebar.number_input("Maximum Speed (m/s)", 0.1, 20.0, 4.0)


st.sidebar.header("Environment")

friction = st.sidebar.slider("Surface Friction", 0.1, 1.5, 0.7)

slope = st.sidebar.slider("Slope Angle (degrees)", -30, 30, 0)

distance = st.sidebar.number_input(
    "Distance to Obstacle (m)", 0.1, 100.0, 5.0
)

st.sidebar.header("Robot Action")

velocity = st.sidebar.number_input(
    "Current Speed (m/s)", 0.0, max_speed, 2.0
)

deceleration = st.sidebar.number_input(
    "Max Deceleration (m/s²)", 0.1, 20.0, 4.0
)

load_weight = st.sidebar.number_input(
    "Load Weight (kg)", 0.0, max_load, 500.0
)

run = st.sidebar.button("Run Safety Audit")

st.divider()

# -----------------------------
# BUILD WORLD STATE
# -----------------------------


def build_world_state():

    agent = AgentState(
        id="robot",
        type="mobile",
        mass=mass,
        velocity=velocity,
        max_speed=max_speed,
        wheelbase=wheelbase,
        center_of_mass_height=com_height,
        load_weight=load_weight,
        max_load=max_load,
    )

    environment = EnvironmentState(
        friction=friction,
        slope=slope,
        obstacle_distance=distance,
        temperature=20,
    )

    world = WorldState(
        agent=agent,
        environment=environment
    )

    return world


# -----------------------------
# SAFETY ENGINE
# -----------------------------


def run_audit():

    world_state = build_world_state()

    constraints = [
        BrakingConstraint(),
        LoadConstraint(),
        FrictionConstraint(),
        StabilityConstraint(),
    ]

    engine = SafetyEngine(constraints)

    results = engine.evaluate(world_state)

    return results


# -----------------------------
# SAFETY SCORE
# -----------------------------


def compute_safety_score(results):

    if not results:
        return 100

    failures = 0

    for r in results:
        if not r.passed:
            failures += 1

    total = len(results)

    score = max(0, int(100 - (failures / total) * 100))

    return score


# -----------------------------
# HUMAN EXPLANATION
# -----------------------------


def explain_results(results):

    messages = []

    for r in results:

        if r.passed:
            continue

        name = r.name.lower()

        if "braking" in name:
            messages.append(
                "The robot cannot stop before reaching the obstacle."
            )

        elif "load" in name:
            messages.append(
                "The robot is carrying more load than its safe limit."
            )

        elif "friction" in name:
            messages.append(
                "Surface traction is too low for the current speed."
            )

        elif "stability" in name:
            messages.append(
                "The robot may tip over due to high center of mass or slope."
            )

        else:
            messages.append(r.message)

    return messages


# -----------------------------
# RESULTS
# -----------------------------


if run:

    results = run_audit()

    score = compute_safety_score(results)

    col1, col2 = st.columns([1, 2])

    with col1:

        st.subheader("Safety Score")

        fig = go.Figure(
            go.Indicator(
                mode="gauge+number",
                value=score,
                title={"text": "Safety"},
                gauge={
                    "axis": {"range": [0, 100]},
                    "bar": {"color": "green"},
                    "steps": [
                        {"range": [0, 40], "color": "red"},
                        {"range": [40, 70], "color": "orange"},
                        {"range": [70, 100], "color": "lightgreen"},
                    ],
                },
            )
        )

        st.plotly_chart(fig, use_container_width=True)

    with col2:

        st.subheader("Physics Constraint Results")

        table = []

        for r in results:

            table.append(
                {
                    "Constraint": r.name,
                    "Passed": r.passed,
                    "Message": r.message,
                }
            )

        df = pd.DataFrame(table)

        st.dataframe(df)

    st.divider()

    st.subheader("Human Explanation")

    messages = explain_results(results)

    if not messages:
        st.success("The robot action appears physically safe.")
    else:
        for m in messages:
            st.warning(m)

    st.divider()

    st.subheader("Scenario Analysis")

    st.write(
        "The system predicts whether the robot action is safe under current conditions."
    )

    if score < 50:
        st.error("High risk scenario detected.")
    elif score < 80:
        st.warning("Moderate safety risk.")
    else:
        st.success("Robot action is likely safe.")