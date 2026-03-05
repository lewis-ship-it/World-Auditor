import streamlit as st

st.set_page_config(
    page_title="SafeBot Physics Auditor",
    layout="wide"
)

st.title("🛡️ SafeBot Robotics Safety Platform")

st.markdown("""
Welcome to **SafeBot Auditor**.

This system analyzes whether a robot's plan violates physics constraints.

### Available Tools

• Simulator  
• Video Physics Audit  
• Robot Comparison  
• Map Route Planner  

Use the **left sidebar** to switch pages.
""")

st.divider()

col1, col2, col3 = st.columns(3)

with col1:
    st.subheader("Physics Simulator")
    st.write("Simulate robot braking, load, slope and friction.")

with col2:
    st.subheader("Video Safety Audit")
    st.write("Analyze real robot movement from uploaded footage.")

with col3:
    st.subheader("Path Planner")
    st.write("Upload maps and compute safe robot speeds.")