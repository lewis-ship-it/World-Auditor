import sys
import os
import streamlit as st
import pandas as pd
import plotly.graph_objects as go

current_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.abspath(os.path.join(current_dir, ".."))

if root_path not in sys.path:
    sys.path.insert(0, root_path)

from ui.world_builder import build_world
from ui.engine_builder import build_engine


st.set_page_config(page_title="SafeBot Audit", layout="wide")

st.title("🛡️ Robot Physics Safety Audit")

with st.sidebar:

    st.header("Robot")

    mass = st.number_input("Mass (kg)",1.0,5000.0,1200.0)
    wheelbase = st.number_input("Wheelbase (m)",0.5,5.0,1.2)
    cog_h = st.number_input("Center of Mass Height (m)",0.1,2.0,0.6)

    st.header("Environment")

    friction = st.slider("Surface Friction",0.1,1.2,0.7)
    slope = st.slider("Slope (deg)",-20,20,0)
    distance = st.number_input("Obstacle Distance",0.1,100.0,10.0)

    st.header("Action")

    velocity = st.number_input("Velocity (m/s)",0.0,20.0,5.0)

    run = st.button("Run Audit")


if run:

    world = build_world(
        velocity=velocity,
        mass=mass,
        friction=friction,
        slope=slope,
        distance=distance,
        load=0,
        com_height=cog_h,
        wheelbase=wheelbase
    )

    engine = build_engine()

    results = engine.evaluate(world)

    data = []

    for r in results:

        for res in r:

            data.append({
                "Constraint": res.name,
                "Violation": res.violated,
                "Severity": res.severity
            })

    df = pd.DataFrame(data)

    st.subheader("Constraint Results")

    st.table(df)

    violations = df["Violation"].sum()

    score = max(0,100 - violations * 25)

    fig = go.Figure(go.Indicator(
        mode="gauge+number",
        value=score,
        title={"text":"Safety Score"},
        gauge={"axis":{"range":[0,100]}}
    ))

    st.plotly_chart(fig,use_container_width=True)