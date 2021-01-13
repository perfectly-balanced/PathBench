# PathBench: Benchmarking Platform for Classic and Learned Path Planning Algorithms
[![coverage report](https://codecov.io/gh/perfectly-balanced/PathBench/coverage.svg?branch=master)](https://codecov.io/gh/perfectly-balanced/PathBench/?branch=master)

PathBench is a motion planning platform used to develop, assess, compare and visualise the performance and behaviour of both classic and machine learning-based robot path planners in a two- or three-dimensional space.

## Quick Start

**Python 3.8.5** officially supported, though older (and newer) versions should work for the most part.

The following installation and run instructions have been used for running PathBench on **Ubuntu 18 and 20**.

### Installing dependencies

```bash
pip3 install -r requirements.txt
```

Optional dependency is `ompl` with installation not covered here, although installation is detected automatically if both `ompl` and its Python bindings are installed.
There are some extra dependencies needed for testing, and are detailed [here](#testing).

### Simulator Visualiser Usage
```bash
python3 src/main.py -v
```
Note, the main script can be run from any working directory as PathBench does not use relative paths internally, however this document will specify commands as if run from the root directory for clarity.

| Key               	| Action                                                            	|
|-------------------	|-------------------------------------------------------------------	|
| escape            	| Exit the simulator                                                	|
| mouse left click  	| Moves agent to mouse location                                     	|
| mouse right click 	| Moves goal to mouse location                                          |                    
| arrow keys, PgUp, PgDn| Move agent (goal when Alt down) along the X-Y-Z plane                 |
| w a s d               | Rotate orbital camera around map                                      |
| t                 	| Find the path between the agent and goal                            	|
| x                 	| Pause/Resume path finding (animations required)                      	|
| p                 	| Take screenshot                                                    	|
| o                 	| Take top-view high resolution screenshot of the map                	|
| c 	                | Toggle visibility of simulator config window                          |
| v 	                | Toggle visibility of view editor window                               |
| m                 	| Toggle map between Sparse and Dense                               	|
| i                     | Toggle visibility of debug overlay                                    |

Note, screenshots are placed in `data/screenshots/`.

### Other components
Example usage:
```bash
# Run trainer
python3 src/run_trainer.py --num_maps 10000 --full-train --model LSTM
# Run generator
python3 src/main.py --generator --num-maps 100 --generator-type house --dims 3
# Run analyzer
python3 src/main.py --analyzer --algorithms "A*" Dijkstra VIN --include-all-builtin-maps
```
Note `A*` is in quotes to prevent glob expansion by the shell.

You can specify multiple components to be run in order: for example, running the trainer and then visualising the resulting trained ML algorithm:
```bash
python3 src/main.py --trainer --visualiser
```

### More Options
Run 
```bash
python3 src/main.py --help
```
to get a full list of all available options for each of the generator, analyser, trainer, and visualiser.

## Testing

**Dependencies**

```bash
pip3 install -r tests/requirements.txt
sudo apt-get install scrot
```

### Headless Testing

**Dependencies**

```bash
sudo apt-get install x11vnc xvfb xtightvncviewer
```

**Example Usage**

1. Running an **individual** graphics test (labyrinth with A* in this case):
```bash
python3 tests/test_graphics/test_labyrinth_A.py --spawn-display --view-display
```

2. Running **all** tests:
```bash
python3 tests/run_tests.py --spawn-display --view-display
```

- Specifying `--view-display` will internally execute `vncviewer`. This will result in a white dialog popup. Press `Enter` when it appears, and the interactive view will subsequently appear. However, please wait approximately a second before pressing `Enter`, otherwise the viewer will be created before the view server has had time to initialise, which will prevent launching the interactive view (i.e. no-op).
- Specifying `--spawn-display` for `run_tests.py` will launch an isolated virtual display for each test as they have side-effects, which would cause failures on the CI. As a result, when specifying `--view-display`, the viewer will keep relaunching itself for each graphics test.
- For more usage details specify the help flag `-h`.

Note, to view a screenshot of the screen, execute the following:

```bash
xwud -in /var/tmp/Xvfb_screen0
```

## PathBench

### **Simulator**

This section is responsible for environment interactions and algorithm visualisation.
It provides custom collision detection systems and a graphics framework (Panda3D) for rendering the internal
state of the algorithms in 2D or 3D. In PathBench, a simulation can be run headlessly or with graphics.
<br />
<img src="./readme_files/potential_field_2d.png" width="130"/>
<img src="./readme_files/RRT_star_long_wall.png" width="130"/>
<img src="./readme_files/screenshot_180.png" width="130" height="130"/>
<img src="./readme_files/A_2D.png" width="130" height="129"/>
<img src="./readme_files/RRT_labyrinth.png" width="134" height="133"/>
<br />
<img src="./readme_files/screenshot_185.png" width="200" height="200"/>
<img src="./readme_files/screenshot_159.png" width="210" height="195"/>
<img src="./readme_files/A_3D_cube.png" width="163" height="154"/>
<img src="./readme_files/screenshot_177.png" width="182" height="196"/>


The GUI main window consists of several components - a Simulator Configuration, View Editor and a Debug Overlay.
<p class="aligncenter">
    <img src="./readme_files/visualiser_full.png" alt="PathBench Main Window" width="450" align="center" />
</p>

The Simulator Configuration window is used to make a selection between different maps and algorithms to analyse, 
set goal and start positions and change animation settings. A map is then initialised on pressing "Update".


The View Editor window is used to customise the map, e.g. change colours of entities or set transparency level. Those 
modifications could also be saved as one of the six states using the "Save" button, and the changes can be 
reverted at any point using "Restore". The 1-6 state buttons provide the freedom to easily toggle between different
custom states.


The Debug Overlay at the top left of the main window provides information about the selected map and algorithm, 
the entity positions, and the current state of the simulator (Initialising, Initialised, Running, Done, Terminated).

To run and use the simulator see [Quick Start](#quick-start).

<br />

### **Generator**
This section is responsible for generating and labelling the training data used to train
the Machine Learning models.

It has several options to change the generation algorithm and vary hyperparameters to that algorithm, as well as specifying number of dimensions and size of the maps. They are saved to a directory corresponding to `data/maps/[algorithm]_[number](_3D)/[0..number].json`, and can then be used for training, analysing, or for use in the visualiser.

### **Trainer**
This section is a class wrapper over the third party Machine Learning libraries. It
provides a generic training pipeline based on the holdout method and standardised access to the
training data. It is a wrapper over the Pytorch python package for machine learning and provides a generic training pipeline for path planning models based on the holdout method and standardized access to the training data generated by the Generator.

Each learned model in PathBench3D (LSTM, CAE, Combined LSTM+CAE and also VIN) inherits from the `MLModel` class, which trains its models using a standard holdout method. The models are trained on the data labels generated by the Generator class, using data received from the A* algorithm. Each model contains metadata about what labels it needs. For example, LSTM trains on raycast information and angle to the goal, whereas VIN just uses the map and path. 

### **Analyzer**
The final section manages the statistical measures used in the practical assessment of
the algorithms. Custom metrics can be defined as well as graphical displays for visual interpretations.

The current metrics are:
 - Average Path Deviation
 - Success Rate
 - Average Time
 - Average Steps
 - Average Distance
 - Average Distance from Goal
 - Average Original Distance from Goal
 - Average Trajectory Smoothness
 - Average Obstacle Clearance
 - Average Search Space
 - Maximum Memory

 These are saved to `pbtest.csv`, and plotted on bar charts and violin plots using `matplotlib` as below: 

 <img src="./readme_files/analyzer_results1.png" width=750>

 <img src="./readme_files/analyzer_results3.png" width=750>


### **ROS Real-time Extension**

This extension provides real-time support for visualisation, coordination
and interaction with a physical robot.

- For a basic demonstration of interacting with `RosMap` see [here](src/ros/basic/README.md).
- For the fully functional 2D ROS Real-time Extension see [here](src/ros/advanced/README.md).

**Example Trajectory**
</br>
<img src="./readme_files/gazebo_ros.png" width="750">

This shows a screenshot of the ROS extension running with Gazebo emulating a real robot (turtlebot3).

### **Architecture High Overiew**

<img src="./readme_files/sim_high_overview.png" width="300"/>

### **Platform Architecture**

<img src="./readme_files/architecture_full.jpg" width="750"/>

### **Infrastructure**

The MainRunner component is the main entry point of the platform and it coordinates all other
sections. The MainRunner takes a master Configuration component as input which represents
the main inflexion point of the platform. It describes which section(s) (Simulator, Generator,
Trainer, Analyser) should be used and how.

The Services component is a bag of Service components which is injected into all platform classes
in order to maintain global access to the core libraries. A Service component is created for most
external libraries to encapsulate their APIs and provide useful helper functions. Moreover, by
making use of the Adapter Pattern we can easily switch third party libraries, if needed, and the
code becomes more test friendly. Finally, the Services container can be mocked together with all
its Service components, thus avoiding rendering, file writing and useless printing.

The Simulator was build by following the Model-View-Controller (MVC) pattern. The
Model represents the logic part, the View renders the Model and the Controller handles
the input from the keyboard and mouse, and calls the appropriate functions from the associated
Model. 

The EventManager component is a communication service which allows the Model to
update the View as there is no direct connection between them (from Model to View, the other
way is).

The Debug component is a printing service which augments printing messages with different
decorators such as time-stamp and routes the messages to a specified IO stream or standard out.
It also provides a range of debugging/printing modes: None (no information), Basic (only basic
information), Low (somewhat verbose), Medium (quite verbose), High (all information).

The Torch service is not an actual wrapper around pytorch, but instead it defines some constants
such as the initial random seed and the training device (CPU/CUDA).

The Resources service is the persistent storage system. It is a container of Directory components
which represent an interface over the actual filesystem directories. It provides safe interaction with
the filesystem and a range of utility directories: Cache (temporary storage used for speeding up second
runs), Screenshots, Maps (stores all user defined and generated maps), Images (stores images which
can be converted to internal 2D maps), Algorithms (stores trained machine learning models), Training
Data (stores training data for machine learning models). The main serialisation tool is dill which is
a wrapper around pickle with lambda serialisation capabilities, but JSON support has been added, and
this should be preferred when possible. Custom serialisation is also allowed
such as tensor serialisation provided by pytorch.

The AlgorithmRunner manages the algorithm session which contains the Algorithm, BasicTesting
and Map. The AlgorithmRunner launches a separate daemon thread that is controlled
by a condition variable. When writing an Algorithm, special key frames can be defined
(e.g. when the trace is produced) to create animations. Key frames release the synchronisation
variable for a brief period and then acquire it again, thus querying new rendering jobs.
Animation is done automatically by monitoring of the trace and display components returned by
`set_display_info()`, with the visualiser updated during the keyframe call.

The Utilities section provides a series of helper methods and classes: Maps (holds in-memory
user defined Map components), Point, Size, Progress (progress bar), Timer, MapProcessing
(feature extractor used mainly in ML sections).

<!-- [Overleaf link](https://www.overleaf.com/project/5c006f75ebc04119dbfb3c90)-->
