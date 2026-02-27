import streamlit as st
import pandas as pd
import plotly.express as px
from google import genai
import json
import time
import os
from dotenv import load_dotenv

# Import your Alignment Core modules
from alignment_core.world_model.braking import compute_stopping_distance
from alignment_core.world_model.primitives import Vector3

load_dotenv()

st.set_page_config(page_title="World Auditor | AI Reality Benchmarker", layout="wide")
st.title("🧠 World Auditor: The Reality Layer")
with st.sidebar:
    st.header("👤 Auditor Profile")
    username = st.text_input("Enter Auditor Name", "Guest_User")
    st.session_state['username'] = username
    
    # Simple 'Experience Points' mock
    if 'xp' not in st.session_state:
        st.session_state['xp'] = 0
    st.info(f"Auditor Level: {st.session_state['xp'] // 10} | XP: {st.session_state['xp']}")

with st.sidebar:
    st.header("⚙️ Configuration")
    api_key = st.secrets.get("GOOGLE_API_KEY")
    # FIX: Defined the committee list to prevent the 'NameError'
    committee = ["gemini-1.5-flash", "gemini-1.5-pro"]
    
    if api_key:
        st.success("🔒 System Secure: Logic Engine Active")
    else:
        st.error("🚨 SECURITY ERROR: API Key Missing.")
        st.stop()

col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("📡 Input Feed")
    input_type = st.radio("Select Type", ["Text", "Image", "Video"])
    user_input = st.file_uploader(f"Upload {input_type}", type=["jpg", "png", "mp4", "mov"])
    
    if user_input:
        if input_type == "Video": st.video(user_input)
        else: st.image(user_input)

if st.button("🚀 RUN REALITY AUDIT") and user_input:
    client = genai.Client(api_key=api_key)
    results = []
    
    with st.spinner("Analyzing Physics Alignment..."):
        file_mime_type = user_input.type 
        with open("temp_file", "wb") as f:
            f.write(user_input.getbuffer())
        
        # Uploading to Google GenAI
        payload = client.files.upload(file="temp_file", config={'mime_type': file_mime_type})
        while payload.state == "PROCESSING":
            time.sleep(2)
            payload = client.files.get(name=payload.name)

        # Updated Visual Auditor Prompt
        prompt = """
        Analyze this clip as a Physics Auditor. Return ONLY a JSON object:
        {
            "estimated_speed": float,
            "friction_coeff": float,
            "slope_z": float,
            "dist_to_hazard": float,
            "verdict": "VETO/CLEAR",
            "reasoning": "string"
        }
        """

        for model in committee:
            try:
                response = client.models.generate_content(
                    model=model, 
                    contents=[payload, prompt],
                    config={'response_mime_type': 'application/json'}
                )
                data = json.loads(response.text)
                
                # --- REALITY BRIDGE: Connect AI Perception to your Braking Math ---
                # We pass the AI's estimated slope_z into our local physics function
                calc_stop = compute_stopping_distance(
                    speed=data['estimated_speed'],
                    friction=data['friction_coeff'],
                    gravity_z=-9.81,
                    slope_vector=Vector3(x=0, y=0, z=data['slope_z'])
                )
                
                data['physics_stop_dist'] = round(calc_stop, 2)
                data['model'] = model
                
                # Compare AI prediction vs Physical Reality
                if calc_stop > data['dist_to_hazard']:
                    data['verdict'] = "VETO"
                    data['reasoning'] += f" | Reality Breach: Object cannot stop in time."
                
                results.append(data)
            except Exception as e:
                st.error(f"Audit Error ({model}): {e}")

    with col2:
        st.subheader("📊 Audit Results")
        if results:
            df = pd.DataFrame(results)
            st.plotly_chart(px.bar(df, x="model", y="estimated_speed", color="verdict", title="Perceived Speed vs Safety"))
            for res in results:
                with st.expander(f"🔍 {res['model']} Details"):
                    st.write(f"**Verdict:** {res['verdict']}")
                    st.write(f"**AI Seen Speed:** {res['estimated_speed']} m/s")
                    st.write(f"**Physics Required Stop:** {res['physics_stop_dist']} m")
                    st.info(res['reasoning'])