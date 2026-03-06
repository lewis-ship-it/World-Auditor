import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
import math
import time

st.title("🗺️ Map Speed Analyzer")

st.markdown("""
Upload a **track map** and enter the **real world track length**.

The system will:

• Convert the image into a track path  
• Estimate curvature along the route  
• Compute safe robot speeds  
• Calculate braking distances  
• Estimate total traversal time  
• Animate the robot moving along the path
""")

# -----------------------------
# SIDEBAR ROBOT SETTINGS
# -----------------------------

st.sidebar.header("Robot Physics")

mass = st.sidebar.number_input("Robot Mass (kg)", 1.0, 500.0, 40.0)

wheelbase = st.sidebar.number_input("Wheelbase (m)", 0.1, 3.0, 0.6)

friction = st.sidebar.slider("Surface Friction", 0.1, 1.2, 0.8)

max_brake = st.sidebar.number_input("Max Brake m/s²", 0.1, 20.0, 6.0)

max_accel = st.sidebar.number_input("Max Acceleration m/s²", 0.1, 20.0, 3.0)

# -----------------------------
# TRACK SETTINGS
# -----------------------------

st.sidebar.header("Track Settings")

track_length_meters = st.sidebar.number_input(
    "Total Track Length (meters)",
    min_value=1.0,
    max_value=10000.0,
    value=100.0
)

# -----------------------------
# MAP UPLOAD
# -----------------------------

uploaded_file = st.file_uploader("Upload Track Map Image")

if uploaded_file:

    image = Image.open(uploaded_file)
    image = image.convert("RGB")

    st.image(image, caption="Uploaded Track")

    img = np.array(image).astype("uint8")

    # Convert to grayscale safely
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # Edge detection
    edges = cv2.Canny(gray, 60, 120)

    st.subheader("Detected Track Edges")

    st.image(edges)

    # -----------------------------
    # EXTRACT TRACK POINTS
    # -----------------------------

    y_coords, x_coords = np.where(edges > 0)

    points = np.column_stack((x_coords, y_coords))

    if len(points) < 20:

        st.error("Track detection failed. Upload a clearer map.")
        st.stop()

    # Downsample to keep processing light
    points = points[::60]

    # Sort path left → right
    points = points[points[:, 0].argsort()]

    xs = points[:, 0]
    ys = points[:, 1]

    # -----------------------------
    # COMPUTE PIXEL PATH LENGTH
    # -----------------------------

    pixel_length = 0

    for i in range(len(points) - 1):

        p1 = points[i]
        p2 = points[i + 1]

        pixel_length += np.linalg.norm(p2 - p1)

    meters_per_pixel = track_length_meters / pixel_length

    st.success(f"Scale: {meters_per_pixel:.4f} meters per pixel")

    # -----------------------------
    # CURVATURE ANALYSIS
    # -----------------------------

    curvatures = []

    for i in range(1, len(points) - 1):

        p1 = points[i - 1]
        p2 = points[i]
        p3 = points[i + 1]

        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        if a * b * c == 0:
            curvatures.append(0)
            continue

        curvature = abs(
            4 * np.sqrt(
                max(
                    0,
                    (a + b - c)
                    * (a - b + c)
                    * (-a + b + c)
                    * (a + b + c)
                )
            )
            / (a * b * c)
        )

        curvatures.append(curvature)

    curvatures = np.array(curvatures)

    curvatures = curvatures / (curvatures.max() + 1e-6)

    # -----------------------------
    # SPEED MODEL
    # -----------------------------

    g = 9.81

    max_curve_speed = math.sqrt(friction * g * wheelbase)

    speeds = []

    for k in curvatures:

        speed = max_curve_speed * (1 - k)

        speed = max(speed, 0.5)

        speeds.append(speed)

    speeds = np.array(speeds)

    # -----------------------------
    # BRAKING DISTANCE
    # -----------------------------

    braking_distances = []

    for v in speeds:

        stop_dist = (v ** 2) / (2 * max_brake)

        braking_distances.append(stop_dist)

    braking_distances = np.array(braking_distances)

    # -----------------------------
    # DISTANCE ALONG TRACK
    # -----------------------------

    distances = []

    total = 0

    for i in range(len(points) - 1):

        segment = np.linalg.norm(points[i + 1] - points[i])

        segment_meters = segment * meters_per_pixel

        total += segment_meters

        distances.append(total)

    distances = np.array(distances)

    # -----------------------------
    # SPEED GRAPH
    # -----------------------------

    st.subheader("Speed Profile")

    fig = go.Figure()

    fig.add_trace(
        go.Scatter(
            x=distances,
            y=speeds,
            mode="lines",
            name="Speed m/s"
        )
    )

    fig.update_layout(
        xaxis_title="Distance Along Track (m)",
        yaxis_title="Speed (m/s)"
    )

    st.plotly_chart(fig, use_container_width=True)

    # -----------------------------
    # BRAKING GRAPH
    # -----------------------------

    st.subheader("Braking Distance")

    fig2 = go.Figure()

    fig2.add_trace(
        go.Scatter(
            x=distances,
            y=braking_distances,
            mode="lines",
            name="Braking Distance"
        )
    )

    fig2.update_layout(
        xaxis_title="Distance Along Track (m)",
        yaxis_title="Braking Distance (m)"
    )

    st.plotly_chart(fig2, use_container_width=True)

    # -----------------------------
    # ESTIMATED TRAVEL TIME
    # -----------------------------

    avg_speed = speeds.mean()

    total_time = track_length_meters / avg_speed

    st.metric("Estimated Traversal Time", f"{total_time:.2f} seconds")

    # -----------------------------
    # ROBOT ANIMATION
    # -----------------------------

    st.subheader("Robot Simulation")

    placeholder = st.empty()

    for i in range(len(xs)):

        fig_anim = go.Figure()

        fig_anim.add_trace(
            go.Scatter(
                x=xs,
                y=ys,
                mode="lines",
                line=dict(color="gray")
            )
        )

        fig_anim.add_trace(
            go.Scatter(
                x=[xs[i]],
                y=[ys[i]],
                mode="markers",
                marker=dict(size=12),
                name="Robot"
            )
        )

        fig_anim.update_layout(height=500, showlegend=False)

        placeholder.plotly_chart(fig_anim, use_container_width=True)

        time.sleep(0.015)