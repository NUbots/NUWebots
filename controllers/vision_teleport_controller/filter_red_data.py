#!/usr/bin/env python3
"""
Script to filter mono vision data by detecting red pixels in mask images.
Moves corresponding image, mask, and lens files to data_filtered directory
with sequential numbering.
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
    
    Args:
        mask_path: Path to the mask image
        red_threshold: Minimum number of pure red pixels to consider as containing red
        
    Returns:
        bool: True if pure red pixels are found, False otherwise
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
    data_filtered_dir = "/home/angy-mei/NUWebots/controllers/vision_teleport_controller/data/data_filtered_pure_red"
    
    # Create filtered directory if it doesn't exist
    os.makedirs(data_filtered_dir, exist_ok=True)
    
    # Get all mask files and sort them
    mask_files = glob.glob(os.path.join(data_mono_dir, "mask*.png"))
    mask_files.sort()
    
    print(f"Found {len(mask_files)} mask files to process")
    
    filtered_count = 0
    
    for mask_file in mask_files:
        # Extract the number from the mask filename
        mask_basename = os.path.basename(mask_file)
        number = extract_number(mask_basename)
        
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
            
            # Copy files (don't move - keep originals)
            try:
                # Copy files to filtered directory
                shutil.copy2(mask_file, dest_mask)
                if os.path.exists(source_image):
                    shutil.copy2(source_image, dest_image)
                else:
                    print(f"Warning: {source_image} not found")
                    
                if os.path.exists(source_lens):
                    shutil.copy2(source_lens, dest_lens)
                else:
                    print(f"Warning: {source_lens} not found")
                
                # DON'T remove original files - keep them
                print(f"Copied set {number} -> {new_number} (contains pure red pixels)")
                
            except Exception as e:
                print(f"Error copying files for {number}: {e}")
        else:
            # Optionally print progress for files without red pixels
            if int(number) % 1000 == 0:  # Print every 1000th file
                print(f"Processed {number} (no pure red pixels)")
    
    print(f"\nFiltering complete!")
    print(f"Total files with red pixels: {filtered_count}")
    print(f"Files copied to: {data_filtered_dir}")

if __name__ == "__main__":
    main()
