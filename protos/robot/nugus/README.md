This contains the URDF and proto file description and all the mesh files for the NUgus robot model generated automatically via the [onshape-to-robot](https://onshape-to-robot.readthedocs.io/en/latest/) tool. The onshape CAD model of the NUgus robot can be viewed [here](https://cad.onshape.com/documents/7a93aa6f68a144f3337682ca/w/21ea40de0160dd9b6d40225b/e/beac31a00af2912b29fa71ac?renderMode=0&uiState=646d68c25eec491896d25b02).

## Instructions

### Pre-requisites

Before you begin, make sure you have Python installed on your machine. If not, you can download it from [python.org](https://www.python.org/).

### Installing Dependencies

The following packages are required to run the code:

- `numpy`
- `scipy`
- `pymeshlab`
- `urdf2webots`
- `onshape-to-robot`

To install these dependencies, you can use pip (Python's package installer):

```bash
pip install numpy scipy pymeshlab urdf2webots onshape-to-robot
```

### Generating the URDF and proto file
1. Run `onshape-to-robot .` in this directory
