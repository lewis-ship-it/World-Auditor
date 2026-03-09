import streamlit as st
import numpy as np
import plotly.graph_objects as go
import math


st.title("Map Path Speed Planner")

distance = st.slider("Route Length (m)",10,500,100)

friction = st.slider("Surface Friction",0.1,1.0,0.7)

points = np.linspace(0,distance,100)

speeds = []

for p in points:

    remaining = max(distance - p,1)

    vmax = math.sqrt(2 * friction * 9.81 * remaining)

    speeds.append(vmax)

fig = go.Figure()

fig.add_trace(go.Scatter(
    x=points,
    y=speeds,
    mode="lines",
    name="Max Safe Speed"
))

fig.update_layout(
    xaxis_title="Distance",
    yaxis_title="Speed (m/s)"
)

st.plotly_chart(fig,use_container_width=True)