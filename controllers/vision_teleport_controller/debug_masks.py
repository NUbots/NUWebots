#!/usr/bin/env python3
"""
Debug script to examine mask image contents and color values.
"""

import os
import glob
from PIL import Image
import numpy as np
import re

def analyze_mask(mask_path):
    """Analyze a mask image to understand its color distribution."""
    try:
        # Open the mask image
        img = Image.open(mask_path)
        print(f"\nAnalyzing: {os.path.basename(mask_path)}")
        print(f"Mode: {img.mode}")
        print(f"Size: {img.size}")
        
        # Convert to RGB if not already
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        # Convert to numpy array
        img_array = np.array(img)
        print(f"Shape: {img_array.shape}")
        
        # Get unique colors
        unique_colors = np.unique(img_array.reshape(-1, 3), axis=0)
        print(f"Unique colors found: {len(unique_colors)}")
        
        for i, color in enumerate(unique_colors):
            r, g, b = color
            pixel_count = np.sum((img_array == color).all(axis=2))
            print(f"  Color {i+1}: RGB({r}, {g}, {b}) - {pixel_count} pixels")
            
            # Check if this looks like red
            if r > g and r > b and r > 100:
                print(f"    ^ This looks RED!")
        
        return unique_colors
        
    except Exception as e:
        print(f"Error analyzing {mask_path}: {e}")
        return None

def main():
    data_mono_dir = "/home/angy-mei/NUWebots/controllers/vision_teleport_controller/data/data_mono"
    
    # Get a few mask files to analyze
    mask_files = glob.glob(os.path.join(data_mono_dir, "mask*.png"))
    mask_files.sort()
    
    # Analyze first 5 files
    for mask_file in mask_files[:5]:
        colors = analyze_mask(mask_file)
        print("-" * 50)

if __name__ == "__main__":
    main()
