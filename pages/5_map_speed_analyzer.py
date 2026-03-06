import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image

from alignment_core.physics.physics_engine import PhysicsEngine
from alignment_core.perception.map_segmentation import segment_track

st.title("🗺️ Map Speed Analyzer")

st.markdown("Analyze robot traversal speed across a map.")

friction = st.sidebar.slider("Surface Friction",0.1,1.5,0.8)

engine = PhysicsEngine(friction)

track_length = st.sidebar.number_input("Track Length (meters)",100,100000,5000)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:

    image = Image.open(uploaded_file)

    st.image(image)

    img = np.array(image)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    edges = cv2.Canny(gray,50,150)

    y,x = np.where(edges>0)

    points = np.column_stack((x,y))

    if len(points)>2000:
        points = points[::20]

    segments = segment_track(points)

    speeds = []

    for i in range(1,len(points)-1):

        p1 = points[i-1]
        p2 = points[i]
        p3 = points[i+1]

        a = np.linalg.norm(p2-p1)
        b = np.linalg.norm(p3-p2)
        c = np.linalg.norm(p3-p1)

        if a*b*c == 0:
            speeds.append(0)
            continue

        radius = (a*b*c)/np.sqrt(abs((a+b+c)*(b+c-a)*(c+a-b)*(a+b-c)))

        vmax = engine.max_corner_speed(radius)

        speeds.append(vmax)

    st.subheader("Speed Profile")

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            y=speeds,
            mode="lines"
        )
    )

    st.plotly_chart(fig)

    st.subheader("Track Segments")

    st.write({
        "straights":segments.count("straight"),
        "curves":segments.count("curve"),
        "hairpins":segments.count("hairpin")
    })