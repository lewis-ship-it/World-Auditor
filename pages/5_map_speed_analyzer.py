import streamlit as st

st.set_page_config(
    page_title="World Auditor",
    page_icon="🌍",
    layout="wide"
)

st.title("World Auditor Robotics Safety Platform")

st.markdown("""
Welcome to **World Auditor**.

This platform evaluates robotic systems for safety, navigation stability, and operational constraints.

### Available Tools

Use the sidebar to access modules:

• Robot Safety Audit  
• Route Optimizer  
• Map Speed Analyzer  
• Robot Visualization  
• Safety Reports

Each module operates independently to keep the system modular and scalable.
""")

st.info("Select a module from the sidebar to begin.")