import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
from sklearn.neighbors import NearestNeighbors

# --- THE NEW CONTOUR-BASED EXTRACTOR ---

def extract_clean_path(image_np, thresh_val):
    # 1. Convert to grayscale and blur to remove tiny speckles
    gray = cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    
    # 2. Thresholding
    _, binary = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY_INV)
    
    # 3. Find CONTOURS (Continuous shapes)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None

    # 4. Pick the LARGEST contour (the track) and ignore the rest (noise/text)
    main_track = max(contours, key=cv2.contourArea)
    
    # 5. Smooth the contour to remove "pixel jitter"
    epsilon = 0.005 * cv2.arcLength(main_track, True)
    approx_path = cv2.approxPolyDP(main_track, epsilon, True)
    
    # Reshape for our physics logic
    return approx_path.reshape(-1, 2)

# --- PHYSICS & UI ---

st.set_page_config(page_title="SafeBot Contour Auditor", layout="wide")
st.title("🗺️ Contour-Based Map Auditor")

st.sidebar.header("Processing Settings")
# This slider is key: move it until the "noise" disappears in the preview
thresh_val = st.sidebar.slider("Map Sensitivity (Threshold)", 0, 255, 127)
show_binary = st.sidebar.checkbox("Show AI's view of the track", value=False)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    
    # Show what the computer "sees" to help the user adjust the slider
    if show_binary:
        gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
        _, b_view = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)
        st.image(b_view, caption="AI's Binary View (Track should be solid White)", width=400)

    pts = extract_clean_path(img_np, thresh_val)
    
    if pts is not None and len(pts) > 5:
        # Since contours are already ordered, we don't even need the complex sort!
        xs, ys = pts[:, 0], pts[:, 1]
        
        # Visualize on Map
        map_fig = go.Figure()
        map_fig.add_trace(go.Scatter(
            x=xs, y=ys, mode='lines+markers',
            line=dict(color='cyan', width=3),
            marker=dict(size=4, color='white')
        ))
        
        map_fig.update_layout(
            images=[dict(source=img_pil, xref="x", yref="y", x=0, y=max(ys),
                         sizex=max(xs), sizey=max(ys),
                         sizing="stretch", opacity=0.5, layer="below")],
            xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
            template="plotly_dark", height=600
        )
        st.plotly_chart(map_fig, use_container_width=True)
        
        st.success(f"✅ Track successfully isolated with {len(pts)} key points.")
    else:
        st.error("No track detected. Try adjusting the 'Map Sensitivity' slider.")