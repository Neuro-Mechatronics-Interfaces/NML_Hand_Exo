---
layout: splash
title: Tutorials
permalink: /tutorials/
header:
  overlay_image: /assets/images/hero/hero.jpg
  overlay_filter: 0.25
  caption: "Step-by-step guides to get you moving fast."
feature_row:
  - image_path: /assets/images/tutorials/first-motion.png
    alt: "First Motion"
    title: "First Motion"
    excerpt: "Flash firmware, home axes, and execute your first gesture."
    url: "{{ '/tutorials/first-motion/' | relative_url }}"
    btn_label: "Start"
    btn_class: "btn--primary"
  - image_path: /assets/images/tutorials/live-emg.png
    alt: "Live EMG Classification"
    title: "Live EMG Classification"
    excerpt: "Stream 250 ms windows from Intan/Open Ephys and classify in real time."
    url: "{{ '/tutorials/live-emg/' | relative_url }}"
    btn_label: "Open"
    btn_class: "btn--primary"
  - image_path: /assets/images/tutorials/ros2-teleop.png
    alt: "ROS2 Teleop"
    title: "ROS2 Teleop"
    excerpt: "Joystick control with collision checks using MoveIt."
    url: "{{ '/tutorials/ros2-teleop/' | relative_url }}"
    btn_label: "Coming soon"
    btn_class: "btn--disabled"
feature_row2:
  - image_path: /assets/images/tutorials/force-plate.png
    alt: "Force Plate Calibration"
    title: "Force Plate Calibration"
    excerpt: "Zero, span, and verify using reference weights."
    url: "{{ '/tutorials/force-plate/' | relative_url }}"
    btn_label: "Coming soon"
    btn_class: "btn--disabled"
---

<div class="notice--primary">
Follow these focused walkthroughs to go from <strong>flash → motion → control</strong>.
Each tutorial includes prerequisites, commands you can copy-paste, and expected results with screenshots.
</div>

{% include feature_row %}
{% include feature_row id="feature_row2" %}
