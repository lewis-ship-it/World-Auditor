import streamlit as st

from core.robot_profiles import ROBOT_PROFILES, SURFACE_MAP, BRAKE_MAP
from core.physics_engine import stopping_distance, safety_margin
from visual.graphs import stopping_graph


st.title("Robot Physics Simulator")

profile_name = st.selectbox(

    "Robot",
    list(ROBOT_PROFILES.keys())

)

profile = ROBOT_PROFILES[profile_name]

velocity = st.slider("Velocity (m/s)",0.0,20.0,5.0)

distance = st.slider("Obstacle Distance",1.0,30.0,10.0)

surface = st.selectbox(

    "Surface",
    list(SURFACE_MAP.keys())

)

friction = SURFACE_MAP[surface]

brake = st.select_slider(

    "Brake Condition",
    options=list(BRAKE_MAP.keys())

)

decel = BRAKE_MAP[brake]

stop_dist = stopping_distance(

    velocity,
    decel,
    friction

)

margin = safety_margin(

    distance,
    stop_dist

)

if margin > 0:
    st.success("SAFE")
else:
    st.error("COLLISION RISK")

fig = stopping_graph(stop_dist, margin)

st.plotly_chart(fig, use_container_width=True)