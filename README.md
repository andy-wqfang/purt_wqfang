# Minimum Snap Trajectory Generation
- Paper: [Minimum snap trajectory generation and control for quadrotors](https://ieeexplore.ieee.org/document/5980409)
- Library: [Minsnap-trajectories](https://pypi.org/project/minsnap-trajectories/)
- Experiment code: `example.ipynb`
- **TODO**: implement optimized time allocation using code in `large_scale`.

# Large Scale Trajectory Optimizer
- Paper: [Generating Large-Scale Trajectories Efficiently using Double Descriptions of Polynomials](https://ieeexplore.ieee.org/document/9561585)
- Codebase: [ZJU-FAST-Lab/large_scale_traj_optimizer](https://github.com/ZJU-FAST-Lab/large_scale_traj_optimizer/tree/main)
- Usage: 
    - Follow [README.md](https://github.com/ZJU-FAST-Lab/large_scale_traj_optimizer/blob/main/README.md) in the above link for instructions on the original codebase
    - You may need to install `eigen3`
        - `sudo apt install libeigen3-dev`
    - `new_mywp.cpp`: compile naively. This code generates `min_jerk_traj.csv` and `min_jerk_traj.csv`, trajectories under minimum jerk and minimum snap constraints respectively. 
    - `trajectory_visual.py`: this script plots in 3D either of the above two trajectories. Change the trajectory loaded at line 7 `df = pd.read_csv("min_jerk_traj.csv")` and relevant labels to plot the other trajectory.

# Path Planning Study
- Course: **Modern Robotics, Course 4: Robot Motion Planning and Control** on Coursera

# Usage:
1. Create virtual environment
```
python3 -m venv env
```
2. On Unix or MacOS, use `source env/bin/activate`. On Windows, use `.\env\Scripts\activate`
3. Install packages
```
pip install -r requirements.txt
```