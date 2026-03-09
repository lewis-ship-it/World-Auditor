import streamlit as st
from core.robot_profiles import ROBOT_PROFILES, SURFACE_MAP, BRAKE_MAP
from alignment_core.physics.braking_model import BrakingModel
from visual.graphs import stopping_graph


st.title("Robot Physics Simulator")

robot = st.selectbox("Robot",list(ROBOT_PROFILES.keys()))

velocity = st.slider("Velocity",0.0,20.0,5.0)
distance = st.slider("Obstacle Distance",1.0,30.0,10.0)

surface = st.selectbox("Surface",list(SURFACE_MAP.keys()))
friction = SURFACE_MAP[surface]

brake_condition = st.selectbox("Brake Condition",list(BRAKE_MAP.keys()))

decel = BRAKE_MAP[brake_condition]

model = BrakingModel(friction)

stop_dist = model.braking_distance(velocity)

margin = distance - stop_dist

if margin > 0:
    st.success("SAFE")
else:
    st.error("COLLISION RISK")

fig = stopping_graph(stop_dist,margin)

st.plotly_chart(fig,use_container_width=True)