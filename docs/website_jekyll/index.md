---
layout: splash
title: Robotic Hand-Wrist Orthosis
permalink: /
header:
  overlay_image: /assets/images/hero/hand1.jpg
  overlay_filter: 0.35
  actions:
    - label: "Install now"
      url: "https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/releases/latest"
      class: "btn btn--primary"
excerpt: >
  An open-source platform that brings **software, hardware, and 3D printing** from benchtop to research-grade experiments quickly.
feature_row:
  - image_path: /assets/images/tiles/assemble.png
    alt: "Assemble your own"
    title: "Assemble Your Own"
    excerpt: "Full BOM, prints, torque specs, wiring, safety & QA."
    url: "/assembly/"
    btn_label: "Open guide"
    btn_class: "btn--info"
  - image_path: /assets/images/tiles/examples.png
    alt: "Examples"
    title: "Examples"
    excerpt: "From gesture control to ROS2 teleop and EMG pipelines."
    url: "/examples/"
    btn_label: "Browse examples"
    btn_class: "btn--info"
  - image_path: /assets/images/tiles/sdk.png
    alt: "API & SDK"
    title: "API & SDK"
    excerpt: "Doxygen reference + Python utilities for real-time control."
    url: "/python-api/"
    btn_label: "Open API"
    btn_class: "btn--info"
---


<!-- Hero slideshow config (valid JSON; each string uses relative_url) -->
<div
  class="hero-rotator"
  data-images='[
    "{{ "/assets/images/hero/hand1.jpg" | relative_url }}",
    "{{ "/assets/images/hero/hand2.jpg" | relative_url }}",
    "{{ "/assets/images/hero/hand3.jpg" | relative_url }}",
    "{{ "/assets/images/hero/exo_hand_wrist2.png" | relative_url }}"
  ]'
  data-interval="4500"
  data-fade="900">
</div>


{% include feature_row %}
