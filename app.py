import streamlit as st
from datetime import datetime

# Import your engine
from alignment_core.engine import SafetyEngine, SafetyReport
from alignment_core.constraints import BrakingConstraint
from alignment_core.world_model import Agent, Environment, WorldState


st.set_page_config(page_title="AI Physics Commonsense Auditor", layout="wide")

st.title("AI Physics Commonsense Auditor")
st.markdown("Deterministic Safety Layer for AI Actions")


# -----------------------------
# Sidebar Mode Selection
# -----------------------------
mode = st.sidebar.selectbox(
    "Select Demo Mode",
    [
        "Interactive Physics Simulator",
        "Upload Image + Action",
        "Upload Video + Action",
        "Text Action Audit",
    ],
)

# -----------------------------
# MODE 1 — SIMULATOR
# -----------------------------
if mode == "Interactive Physics Simulator":

    st.header("Simulated Robot Action")

    col1, col2 = st.columns(2)

    with col1:
        velocity = st.slider("Velocity (m/s)", 0.0, 15.0, 5.0)
        max_deceleration = st.slider("Max Deceleration (m/s²)", 0.1, 10.0, 2.0)

    with col2:
        distance_to_obstacle = st.slider("Distance to Obstacle (m)", 0.5, 20.0, 4.0)

    if st.button("Propose Action"):

        agent = Agent(velocity=velocity, max_deceleration=max_deceleration)
        environment = Environment(distance_to_obstacle=distance_to_obstacle)
        world_state = WorldState(agent, environment)

        engine = SafetyEngine()
        engine.register_constraint(BrakingConstraint())

        results = engine.evaluate(world_state)
        report = SafetyReport(results)

        st.subheader("Safety Decision")

        if report.is_safe():
            st.success("ALLOW: Action is physically feasible.")
        else:
            st.error("BLOCK: Physics violation detected.")

        st.json(report.to_dict())

# -----------------------------
# MODE 2 — IMAGE + ACTION
# -----------------------------
elif mode == "Upload Image + Action":

    st.header("Image-Based Action Audit")

    uploaded_image = st.file_uploader("Upload Image", type=["jpg", "png", "jpeg"])
    action_text = st.text_area("Describe the AI's proposed action")

    if uploaded_image:
        st.image(uploaded_image, caption="Uploaded Image", use_column_width=True)

    if st.button("Audit Action"):

        st.subheader("Physics Analysis")

        if not action_text:
            st.warning("Please describe the proposed action.")
        else:
            # Simple rule-based analysis demo
            issues = []

            if "fast" in action_text.lower() or "high speed" in action_text.lower():
                issues.append("Potential braking feasibility risk.")

            if "heavy" in action_text.lower():
                issues.append("Load stability risk.")

            if "slope" in action_text.lower():
                issues.append("Slip or tipping risk on incline.")

            if issues:
                st.error("Physics Concerns Detected:")
                for issue in issues:
                    st.write(f"- {issue}")
            else:
                st.success("No obvious physics violations detected from description.")

# -----------------------------
# MODE 3 — VIDEO + ACTION
# -----------------------------
elif mode == "Upload Video + Action":

    st.header("Video-Based Action Audit")

    uploaded_video = st.file_uploader("Upload Video", type=["mp4", "mov"])
    action_text = st.text_area("Describe the AI's proposed action")

    if uploaded_video:
        st.video(uploaded_video)

    if st.button("Audit Video Action"):

        if not action_text:
            st.warning("Please describe the proposed action.")
        else:
            st.subheader("Physics Risk Assessment")

            risks = []

            if "corner" in action_text.lower():
                risks.append("High-speed turning may cause tipping.")

            if "gap" in action_text.lower():
                risks.append("Potential terrain traversal failure.")

            if "overload" in action_text.lower():
                risks.append("Exceeds load capacity.")

            if risks:
                st.error("Potential Violations:")
                for r in risks:
                    st.write(f"- {r}")
            else:
                st.success("No major risks detected from description.")

# -----------------------------
# MODE 4 — TEXT ONLY
# -----------------------------
elif mode == "Text Action Audit":

    st.header("Text-Based Physics Audit")

    action_description = st.text_area("Describe the proposed AI action")

    if st.button("Run Audit"):

        if not action_description:
            st.warning("Please enter an action.")
        else:

            st.subheader("Physics Analysis")

            # Very simple numeric parser demo
            import re

            velocity_match = re.search(r"(\d+)\s*m/s", action_description)
            distance_match = re.search(r"(\d+)\s*m", action_description)

            if velocity_match and distance_match:

                velocity = float(velocity_match.group(1))
                distance = float(distance_match.group(1))

                agent = Agent(velocity=velocity, max_deceleration=2.0)
                environment = Environment(distance_to_obstacle=distance)
                world_state = WorldState(agent, environment)

                engine = SafetyEngine()
                engine.register_constraint(BrakingConstraint())

                results = engine.evaluate(world_state)
                report = SafetyReport(results)

                if report.is_safe():
                    st.success("ALLOW: Physically feasible.")
                else:
                    st.error("BLOCK: Insufficient stopping distance.")

                st.json(report.to_dict())

            else:
                st.info("Could not parse numeric physics parameters. Provide values like '5 m/s' and '3 m'.")