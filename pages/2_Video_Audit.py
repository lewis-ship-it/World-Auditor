import streamlit as st
import cv2
import tempfile

st.title("Video Robot Audit")

file = st.file_uploader(

    "Upload robot video",
    type=["mp4","mov","avi"]

)

if file:

    tfile = tempfile.NamedTemporaryFile(delete=False)
    tfile.write(file.read())

    cap = cv2.VideoCapture(tfile.name)

    ret, frame = cap.read()

    if ret:
        st.image(frame, channels="BGR")

    cap.release()

    st.info("Object tracking module will analyze robot motion.")