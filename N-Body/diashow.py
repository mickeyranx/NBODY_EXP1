import streamlit as st
import os
import time
from PIL import Image

# Get the directory of the current .py script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "diashow"))  # Normalize path for Windows/Linux
# Cache images from the subdirectory
@st.cache_data
def load_images():
    image_files = sorted([f for f in os.listdir(IMAGE_DIR) if f.endswith(".png")])
    return [Image.open(os.path.join(IMAGE_DIR, img)) for img in image_files]

# Load images only once
images = load_images()

# Initialize session state variables
if "index" not in st.session_state:
    st.session_state.index = 0
if "running" not in st.session_state:
    st.session_state.running = False
if "speed" not in st.session_state:
    st.session_state.speed = float(1.0) #seconds per slide

# Display the current image
st.image(images[st.session_state.index], use_column_width=True)

# Navigation and control buttons
col1, col2, col3 = st.columns(3)
with col1:
    if st.button("Previous") and st.session_state.index > 0:
        st.session_state.index -= 1
        st.rerun()
with col2:
    if st.button("Start Slideshow"):
        st.session_state.running = True
        st.rerun()
with col3:
    if st.button("Stop Slideshow"):
        st.session_state.running = False

# Speed control slider
st.session_state.speed = st.slider("Slideshow Speed (seconds per slide)", float(0.5), float(5.0), st.session_state.speed)

# Automatic Slideshow Logic
if st.session_state.running:
    time.sleep(st.session_state.speed)  # Wait before changing slide
    st.session_state.index = (st.session_state.index + 1) % len(images)  # Loop slideshow
    st.rerun()
