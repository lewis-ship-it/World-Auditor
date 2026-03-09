import streamlit as st
import tempfile

from alignment_core.vision.physics_video_analyzer import PhysicsVideoAnalyzer

st.title("Video Robot Audit")

file = st.file_uploader(
    "Upload robot video",
    type=["mp4","mov","avi"]
)

if file:

    tmp = tempfile.NamedTemporaryFile(delete=False)
    tmp.write(file.read())

    analyzer = PhysicsVideoAnalyzer()

    velocities = analyzer.estimate_motion(tmp.name)

    st.subheader("Estimated Velocities")

    st.line_chart(velocities)

    st.write(f"Peak speed estimate: {max(velocities):.2f}")