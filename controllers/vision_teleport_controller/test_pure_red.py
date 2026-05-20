#!/usr/bin/env python3
"""
TEST version - Check for PURE red pixels (255, 0, 0) only.
Test on first 20 files.
"""

import os
import shutil
import glob
from PIL import Image
import numpy as np
import re

def has_red_pixels(mask_path, red_threshold=1):
    """
    Check if mask image contains PURE red pixels (255, 0, 0).
    """
    try:
        # Open the mask image
        img = Image.open(mask_path)
        
        # Convert to RGB if not already
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        # Convert to numpy array
        img_array = np.array(img)
        
        # Only check for PURE red pixels (255, 0, 0)
        pure_red = (img_array[:, :, 0] == 255) & \
                   (img_array[:, :, 1] == 0) & \
                   (img_array[:, :, 2] == 0)
        
        # Count pure red pixels
        red_pixel_count = np.sum(pure_red)
        
        print(f"  Pure red pixels (255,0,0): {red_pixel_count}")
        return red_pixel_count >= red_threshold
        
    except Exception as e:
        print(f"Error processing {mask_path}: {e}")
        return False

def extract_number(filename):
    """Extract the 7-digit number from filename."""
    match = re.search(r'(\d{7})', filename)
    return match.group(1) if match else None

def main():
    # Set up paths
    data_mono_dir = "/home/angy-mei/NUWebots/controllers/vision_teleport_controller/data/data_mono"
    
    # Get all mask files and sort them - TEST FIRST 20
    mask_files = glob.glob(os.path.join(data_mono_dir, "mask*.png"))
    mask_files.sort()
    mask_files = mask_files[:20]  # Test first 20
    
    print(f"Testing pure red detection on {len(mask_files)} files")
    
    pure_red_count = 0
    
    for mask_file in mask_files:
        mask_basename = os.path.basename(mask_file)
        number = extract_number(mask_basename)
        
        print(f"\nChecking {mask_basename}")
        
        if has_red_pixels(mask_file):
            pure_red_count += 1
            print(f"  ✓ HAS pure red pixels")
        else:
            print(f"  ✗ No pure red pixels")
    
    print(f"\nTest Results:")
    print(f"Files with pure red pixels: {pure_red_count} out of {len(mask_files)}")

if __name__ == "__main__":
    main()
