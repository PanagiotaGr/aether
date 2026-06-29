"""Minimal dashboard placeholder for saved AETHER runs."""

from __future__ import annotations

import json
from pathlib import Path

import streamlit as st


st.set_page_config(page_title="AETHER Dashboard", layout="wide")
st.title("AETHER Simulation Dashboard")
st.write("Load saved JSON summaries from experiments and compare route statistics.")

uploaded = st.file_uploader("Upload run_summary.json", type=["json"])
if uploaded is not None:
    data = json.loads(uploaded.read().decode("utf-8"))
    st.json(data)
else:
    example = Path("experiments/grid_baseline/run_summary.json")
    st.info(f"No file uploaded. Expected example path: {example}")
