<h1 align="center">Tiled evidential occupancy grid map (EOGM)</h1>

## 📝 Description

This repository contains a ROS node that fuses multiple evidential occupancy grid maps (EOGM) into a single map and publishes it on a topic in the form of an octomap. For performance reasons, the fusion is done using AVX2 instructions. This is a part of the robotic project of the 3rd year of engineering school at INSA Rouen Normandie. 

## ✅ Requirements

- ROS Melodic
- Ubuntu 18.04 LTS
- AVX2 compatible CPU (Intel Haswell or newer)

## 🛠️ Compilation

To build the project, clone the repository and compile it using `catkin_make`:

```bash
catkin_make install
```

## 🚀 Usage

First, start the ROS core:

```bash
roscore
```

Then, launch the node:

```bash
source devel/setup.bash
rosrun map_fusion map_fusion_node
```

## 📖 Documentation

To generate the documentation, run:

```bash
source devel/setup.bash
rosdoc_lite src/map_fusion
```

The documentation will be generated in the `doc` folder.

## 📜 License

Authors:
- Dimitri TIMOZ
- [Alix ANNERAUD](alix.anneraud.fr)

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
