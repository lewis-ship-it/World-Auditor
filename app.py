import streamlit as st
import pandas as pd
import plotly.express as px
from google import genai
import json
import time
import os
from dotenv import load_dotenv

# 1. Load the .env file immediately
load_dotenv()

# --- APP CONFIG ---
st.set_page_config(page_title="World Auditor | AI Reality Benchmarker", layout="wide")
st.title("🧠 World Auditor: The Reality Layer")
st.markdown("### Benchmarking AI Commonsense across Video, Image, and Text")

# --- SIDEBAR: Settings ---
# --- SIDEBAR: Settings ---
with st.sidebar:
    st.header("⚙️ Configuration")
    
    # Force the app to only use the back-end secret
    api_key = st.secrets.get("GOOGLE_API_KEY")
    
    if api_key:
        st.success("🔒 System Secure: Logic Engine Active")
    else:
        # This only shows if you forgot to add the key to Streamlit Settings
        st.error("🚨 SECURITY ERROR: API Key Missing from Secrets Manager.")
        st.info("Go to App Settings > Secrets to add your GOOGLE_API_KEY.")
        st.stop() # Stops the app from running without a key
# --- MAIN UI: Upload ---
col1, col2 = st.columns([1, 1])

with col1:
    st.subheader("📡 Input Feed")
    input_type = st.radio("Select Type", ["Text", "Image", "Video"])
    
    user_input = None
    if input_type == "Text":
        user_input = st.text_area("Enter Logic/Scenario", placeholder="A bird flies through glass without breaking it...")
    else:
        user_input = st.file_uploader(f"Upload {input_type}", type=["jpg", "png", "mp4", "mov"])
        if user_input:
            if input_type == "Image": st.image(user_input)
            else: st.video(user_input)

# --- EXECUTION ENGINE ---
if st.button("🚀 RUN REALITY AUDIT") and api_key:
    client = genai.Client(api_key=api_key)
    results = []
    
    with st.spinner("Uploading and processing..."):
        payload = None
        # --- FIXED UPLOAD LOGIC ---
        if input_type != "Text" and user_input is not None:
            # 1. Identify the mime type from the Streamlit uploader
            file_mime_type = user_input.type 
            
            # 2. Write the buffer to a temporary file
            with open("temp_file", "wb") as f:
                f.write(user_input.getbuffer())
            
            # 3. FIX: Use 'file' instead of 'path' and pass the mime_type in config
            # This matches the signature required by the current Google GenAI SDK
            payload = client.files.upload(
                file="temp_file", 
                config={'mime_type': file_mime_type}
            )
            
            while payload.state == "PROCESSING":
                time.sleep(2)
                payload = client.files.get(name=payload.name)

        # 4. FIX: Use the expanded prompt from auditor.py to get "reasoning" and "fix"
        prompt = """
        Act as the Supreme Arbiter of Reality. 
        Analyze the input for Physics, Bio, and Causal logic errors.
        Return ONLY a JSON object:
        {
            "verdict": "VETO/CLEAR", 
            "score": 0-100, 
            "reasoning": "Specifically what logic failed?", 
            "fix": "How would real-world physics handle this?"
        }
        """

        # Run Committee
        for model in committee:
            try:
                content_list = [payload if payload else user_input, prompt]
                response = client.models.generate_content(
                    model=model, 
                    contents=content_list,
                    config={'response_mime_type': 'application/json'}
                )
                data = json.loads(response.text)
                data['model'] = model
                results.append(data)
            except Exception as e:
                # Add retry logic for 429 quota errors just like in auditor.py
                if "429" in str(e):
                    st.warning(f"⚠️ {model} hit quota. Please wait a moment.")
                st.error(f"{model} Error: {e}")

    # --- RESULTS DASHBOARD ---
    with col2:
        st.subheader("📊 Audit Results")
        if results:
            df = pd.DataFrame(results)
            
            fig = px.bar(df, x="model", y="score", color="verdict", 
                         title="Consensus Scoreboard", color_discrete_map={"VETO": "red", "CLEAR": "green"})
            st.plotly_chart(fig, use_container_width=True)

            # 5. FIX: Display the expanded reasoning fields
            for res in results:
                with st.expander(f"🔍 {res['model']} Details"):
                    st.write(f"**Verdict:** {res['verdict']} ({res['score']}%)")
                    st.write(f"**Reasoning:** {res.get('reasoning', res.get('reason', 'N/A'))}")
                    st.write(f"**Correction:** {res.get('fix', 'N/A')}")
        else:
            st.warning("No data returned. Check API Key or Quota.")

elif not api_key:
    st.warning("Please enter your API Key in the sidebar or check your .env file.")