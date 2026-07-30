#!/usr/bin/env python3
"""
============================================================
  GENERATE OFFICIAL ROBO CUP RESCUE MAZE VICTIM MARKERS
  Generates clean, high-contrast, official H, S, U letter images
  for testing vision & OCR detection.
============================================================
"""

import cv2
import numpy as np
import os

output_dir = r"c:\Users\AMAN\Desktop\DPS\victim_markers"
os.makedirs(output_dir, exist_ok=True)

# 400x400 standard resolution
SIZE = 400
FONT = cv2.FONT_HERSHEY_SIMPLEX
SCALE = 8.0
THICKNESS = 18

letters = ['H', 'S', 'U']

generated_files = []

for letter in letters:
    # 1. Black letter on White background (Standard Victim)
    img_bg_white = np.full((SIZE, SIZE, 3), 255, dtype=np.uint8)
    (w, h), baseline = cv2.getTextSize(letter, FONT, SCALE, THICKNESS)
    x = (SIZE - w) // 2
    y = (SIZE + h) // 2
    cv2.putText(img_bg_white, letter, (x, y), FONT, SCALE, (0, 0, 0), THICKNESS, cv2.LINE_AA)
    
    path_white = os.path.join(output_dir, f"victim_{letter}_standard.png")
    cv2.imwrite(path_white, img_bg_white)
    generated_files.append(path_white)

    # 2. White letter on Black background (Reverse Victim)
    img_bg_black = np.full((SIZE, SIZE, 3), 0, dtype=np.uint8)
    cv2.putText(img_bg_black, letter, (x, y), FONT, SCALE, (255, 255, 255), THICKNESS, cv2.LINE_AA)
    
    path_black = os.path.join(output_dir, f"victim_{letter}_reverse.png")
    cv2.imwrite(path_black, img_bg_black)
    generated_files.append(path_black)

print("Generated official victim markers successfully:")
for f in generated_files:
    print(f"  - {f}")
