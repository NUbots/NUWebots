#!/usr/bin/env python3
"""
TEST version - Script to filter mono vision data by detecting red pixels in mask images.
Only processes first 10 files for testing.
"""

import os
import shutil
import glob
from PIL import Image
import numpy as np
import re

def has_red_pixels(mask_path, red_threshold=10):
    """
    Check if mask image contains red pixels.
    
    Args:
        mask_path: Path to the mask image
        red_threshold: Minimum number of red pixels to consider as containing red
        
    Returns:
        bool: True if red pixels are found, False otherwise
    """
    try:
        # Open the mask image
        img = Image.open(mask_path)
        
        # Convert to RGB if not already
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        # Convert to numpy array
        img_array = np.array(img)
        
        # Define red color range (you may need to adjust these values)
        # Red pixels typically have high R value, low G and B values
        red_mask = (img_array[:, :, 0] > 200) & \
                   (img_array[:, :, 1] < 100) & \
                   (img_array[:, :, 2] < 100)
        
        # Count red pixels
        red_pixel_count = np.sum(red_mask)
        
        print(f"  Red pixels found: {red_pixel_count}")
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
    data_filtered_dir = "/home/angy-mei/NUWebots/controllers/vision_teleport_controller/data/data_filtered_test"
    
    # Create filtered directory if it doesn't exist
    os.makedirs(data_filtered_dir, exist_ok=True)
    
    # Get all mask files and sort them - ONLY PROCESS FIRST 10 FOR TESTING
    mask_files = glob.glob(os.path.join(data_mono_dir, "mask*.png"))
    mask_files.sort()
    mask_files = mask_files[:10]  # LIMIT TO FIRST 10 FILES
    
    print(f"Testing with {len(mask_files)} mask files")
    
    filtered_count = 0
    
    for mask_file in mask_files:
        # Extract the number from the mask filename
        mask_basename = os.path.basename(mask_file)
        number = extract_number(mask_basename)
        
        print(f"\nProcessing {mask_basename} (number: {number})")
        
        if not number:
            print(f"Could not extract number from {mask_basename}")
            continue
        
        # Check if mask contains red pixels
        if has_red_pixels(mask_file):
            filtered_count += 1
            
            # Generate new sequential filenames
            new_number = f"{filtered_count:07d}"
            
            # Define source and destination paths
            source_image = os.path.join(data_mono_dir, f"image{number}.jpg")
            source_lens = os.path.join(data_mono_dir, f"lens{number}.yaml")
            
            dest_mask = os.path.join(data_filtered_dir, f"mask{new_number}.png")
            dest_image = os.path.join(data_filtered_dir, f"image{new_number}.jpg")
            dest_lens = os.path.join(data_filtered_dir, f"lens{new_number}.yaml")
            
            # COPY files (don't remove originals in test)
            try:
                shutil.copy2(mask_file, dest_mask)
                if os.path.exists(source_image):
                    shutil.copy2(source_image, dest_image)
                else:
                    print(f"Warning: {source_image} not found")
                    
                if os.path.exists(source_lens):
                    shutil.copy2(source_lens, dest_lens)
                else:
                    print(f"Warning: {source_lens} not found")
                
                print(f"COPIED set {number} -> {new_number} (contains red pixels)")
                
            except Exception as e:
                print(f"Error copying files for {number}: {e}")
        else:
            print(f"No red pixels found in {mask_basename}")
    
    print(f"\nTest complete!")
    print(f"Total files with red pixels: {filtered_count}")
    print(f"Test files copied to: {data_filtered_dir}")

if __name__ == "__main__":
    main()
