import streamlit as st

from core.robot_profiles import ROBOT_PROFILES

st.title("Robot Comparison")

r1 = st.selectbox("Robot A",list(ROBOT_PROFILES.keys()))
r2 = st.selectbox("Robot B",list(ROBOT_PROFILES.keys()))

st.write("Robot A specs:", ROBOT_PROFILES[r1])
st.write("Robot B specs:", ROBOT_PROFILES[r2])