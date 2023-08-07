import os
import xml.etree.ElementTree as ET
import pymeshlab

# Parse URDF file
tree = ET.parse('robot.urdf')
root = tree.getroot()

# Get all mesh filenames from URDF
mesh_files = []
for mesh in root.findall(".//mesh"):
    filename = mesh.attrib['filename']

    # Remove 'package:///' prefix
    clean_filename = filename.replace('package:///', '')
    mesh.set('filename', clean_filename)

    mesh_files.append(clean_filename)

# Save modified URDF
tree.write('robot.urdf')

# Simplify each STL file referenced in URDF
for file in mesh_files:
    # Load mesh
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(file)

    # Simplify mesh
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=10000)

    # Save simplified mesh
    ms.save_current_mesh(file)

# Delete all STL files not in URDF
for file in os.listdir('.'):
    if file.endswith('.stl') and file not in mesh_files:
        os.remove(file)

# Delete all .part files
for file in os.listdir('.'):
    if file.endswith('.part'):
        os.remove(file)
