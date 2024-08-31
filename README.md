# Gait Controller
The gait controller simulation is based on the paper "Simple Bipeds Robot Model Exhibiting Speed-Dependent Gait Transition." This Python package includes:

- Dynamics simulation based on numerical calculation of the equation of motion
- Animation plot for the bipedal model
- Data analysis of gait behavior
- Controller based on Raibert's controller
- Our proposed virtual leg model and state transition

## Main Modules
The top directory `./` contains three main modules:

| Module                     | Function                                    |
|----------------------------|---------------------------------------------|
| `simulate_and_make_data.py`| Main programs for simulation                |
| `make_video.py`            | Animation video                             |
| `make_figure_energy.py`    | Analysis notebook                           |

## Submodules
Module files are located in the `./src` directory. They include the following `.py` files:

| Module                   | Function                                      |
|--------------------------|-----------------------------------------------|
| `config.py`              | Configuration file for setting parameters, gains, etc. |
| `equation_of_motion.py`  | Numerical simulation of the equation of motion|
| `plot_for_animation.py`  | Animation plot                                |
| `utils.py`               | Utilities, definition of functions            |
| `variables.py`           | Variables management                          |
| `controller.py`          | Implementation of raibert controller          |


## Requirements
To build the grasp simulation, the following are required:

1. numpy
2. matplotlib (This is necessary for animation.)

```bash
$ sudo python3 -m pip install -U pip
$ pip3 install numpy matplotlib
```

## Setup
```bash
$ git clone git@github.com:HirofumiShin/gait_transition_model.git
$ cd gait_transition_model
$ mkdir data
```


## Making Simulation Data and Paper Graphs
```bash
$ python3 simulate_and_make_data.py --mode low-to-high
$ python3 simulate_and_make_data.py --mode high-to-low
```
Running both scripts generates data for Figures 5, 7 and 8.  
Then, `analysis/analysis_gait_transition.ipynb` and `analysis/analysis_stability.ipynb` analysis the data and creates graphs for these figures.

## Making a Supplementary Video
```bash
$ python3 simulate_and_make_data.py --mode low-to-low
```
This script also generates data. Then, run
```bash
$ python3 make_video.py
```
to create a video file `./video/transition_video.mp4`.

## Note
The following option shows an animation during simulation:
```bash
$ python3 simulate_and_make_data.py --mode low-to-high --animation
$ python3 simulate_and_make_data.py --mode high-to-low --animation
```
Furthermore,
```bash
$ python3 simulate_and_make_data.py -h
```
displays the option modes as:
```
options:
  -h, --help       show this help message and exit
  --mode MODE      Simulation mode selection: low-to-high, high-to-low, low-to-low, one-target
  --target TARGET  Target speed selection: 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, etc.
  --animation      Enable animation: True, False
```


## LICENSE

MIT