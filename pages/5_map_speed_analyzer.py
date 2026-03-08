import streamlit as st
import numpy as np
import cv2
import plotly.graph_objects as go
from PIL import Image
from sklearn.neighbors import NearestNeighbors

# --- THE TRACING ENGINE ---

def trace_path_from_binary(binary_img):
    """Traces the center of the white shape found in the binary image."""
    # 1. Clean the binary image to ensure the track is solid
    kernel = np.ones((5,5), np.uint8)
    binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_CLOSE, kernel)
    
    # 2. Skeletonize to find the 'spine'
    skeleton = cv2.ximgproc.thinning(binary_img)
    
    # 3. Get all spine points
    y_indices, x_indices = np.where(skeleton > 0)
    if len(x_indices) < 10: return None
    points = np.column_stack((x_indices, y_indices))

    # 4. Sequential Trace: Instead of random sorting, we walk the path
    path = [points[0]]
    unused_points = points[1:].tolist()
    
    while unused_points and len(path) < 1000:
        last_pt = path[-1]
        # Find points within a reasonable 'step' distance
        nn = NearestNeighbors(n_neighbors=1).fit(unused_points)
        dist, idx = nn.kneighbors([last_pt])
        
        # If the next point is too far (>30px), we've reached the end of a segment
        if dist[0][0] > 30:
            break
            
        path.append(unused_points.pop(idx[0][0]))
        
    return np.array(path)

# --- UI & LOGIC ---

st.set_page_config(page_title="AI Trace Auditor", layout="wide")
st.title("🗺️ AI-View Path Tracer")

st.sidebar.header("AI Vision Tuning")
thresh_val = st.sidebar.slider("Sensitivity", 0, 255, 127)
track_len = st.sidebar.number_input("Track Length (m)", 1.0, 5000.0, 100.0)

uploaded_file = st.file_uploader("Upload Track Map")

if uploaded_file:
    img_pil = Image.open(uploaded_file).convert("RGB")
    img_np = np.array(img_pil)
    
    # Generate the "Perfect View"
    gray = cv2.cvtColor(img_np, cv2.COLOR_RGB2GRAY)
    _, binary = cv2.threshold(gray, thresh_val, 255, cv2.THRESH_BINARY_INV)
    
    # Show the Binary View so you can tune it
    st.subheader("AI Vision (The 'Perfect View')")
    st.image(binary, width=400, caption="Tune sensitivity until the track is a solid white line.")

    # TRACE THE MAP
    pts = trace_path_from_binary(binary)
    
    if pts is not None:
        xs, ys = pts[:, 0], pts[:, 1]
        
        # Draw the Interactive Map
        map_fig = go.Figure()
        
        # The Trace Line
        map_fig.add_trace(go.Scatter(
            x=xs, y=ys, mode='lines+markers',
            line=dict(color='cyan', width=4),
            marker=dict(size=4, color='white'),
            name="AI Trace"
        ))

        map_fig.update_layout(
            images=[dict(source=img_pil, xref="x", yref="y", x=0, y=max(ys),
                         sizex=max(xs), sizey=max(ys),
                         sizing="stretch", opacity=0.3, layer="below")],
            xaxis=dict(visible=False), yaxis=dict(visible=False, scaleanchor="x"),
            template="plotly_dark", height=600,
            margin=dict(l=0, r=0, t=0, b=0)
        )
        
        st.subheader("Interactive Audit Map")
        st.plotly_chart(map_fig, use_container_width=True)
        st.success(f"✅ AI successfully traced {len(pts)} points along the track spine.")
    else:
        st.error("Trace failed. Ensure the track is a continuous white line in the AI Vision view.")