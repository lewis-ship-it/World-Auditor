import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image

st.set_page_config(page_title="Map Speed Analyzer", layout="wide")
st.title("🗺️ Map Path Extractor")

# Sidebar
threshold = st.sidebar.slider("AI Sensitivity", 0, 255, 120)
downsample = st.sidebar.slider("Path Smoothness", 1, 20, 8)

uploaded_file = st.file_uploader("Upload Track Map")

def extract_centerline(binary):

    # Clean image
    kernel = np.ones((5,5), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    # Find contours of track
    contours, _ = cv2.findContours(
        binary,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if len(contours) == 0:
        return None

    # Use largest contour
    contour = max(contours, key=cv2.contourArea)

    pts = contour[:,0,:]

    return pts

if uploaded_file:

    img = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img)

    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)

    _, binary = cv2.threshold(
        gray,
        threshold,
        255,
        cv2.THRESH_BINARY_INV
    )

    col1, col2 = st.columns(2)

    with col1:
        st.subheader("AI Vision")
        st.image(binary, use_container_width=True)

    path = extract_centerline(binary)

    if path is None:
        st.error("No track detected")
    else:

        path = path[::downsample]

        xs = path[:,0]
        ys = path[:,1]

        with col2:

            fig = go.Figure()

            fig.add_trace(
                go.Scatter(
                    x=xs,
                    y=ys,
                    mode="lines+markers",
                    line=dict(width=3),
                    marker=dict(size=4)
                )
            )

            fig.update_layout(
                images=[dict(
                    source=img,
                    xref="x",
                    yref="y",
                    x=0,
                    y=img_np.shape[0],
                    sizex=img_np.shape[1],
                    sizey=img_np.shape[0],
                    sizing="stretch",
                    opacity=0.4,
                    layer="below"
                )],
                xaxis=dict(visible=False),
                yaxis=dict(visible=False, scaleanchor="x"),
                template="plotly_dark",
                margin=dict(l=0,r=0,t=0,b=0)
            )

            st.plotly_chart(fig, use_container_width=True)

        st.success(f"Extracted {len(xs)} path points.")