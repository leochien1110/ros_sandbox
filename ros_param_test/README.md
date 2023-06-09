# Sensor Fusion

This ROS package integrates yolov7, SORT and ROS streaming together to achieve the forward collision warning for ADAS usages.
vivotek_16: 192.168.11.16
![forward collision](res/output.gif)

## Installation

This package is built and tested on **Ubuntu 20.04 LTS** and **ROS Noetic** with **Python 3.8**.

### Python Packages
```bash
pip install scikit-image filterpy open3d
```

### ROS Workspace and Packages

1. Clone the repository, yolov7 and detection_msg package:
    ```bash
    cd <ros_workspace>/src      # DONOT use captial letter for your workspace path
    git clone --recurse-submodules https://github.com/linkernetworks/modelt_fcw.git
    cd modelt_fcw/src/yolov7
    pip install -r requirements.txt     # install the requirements for yolov7
    # TODO clone redis_publisher
    ```
1. Download the yolov7 weight

    [`yolov7.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt) [`yolov7x.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7x.pt) [`yolov7-w6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-w6.pt) [`yolov7-e6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6.pt) [`yolov7-d6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-d6.pt) [`yolov7-e6e.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6e.pt)
1. Make sure lv-ros-msgs is read. If not:
    ```bash
    cd <ros_workspace>/src
    git clone git@github.com:linkernetworks/lv-ros-msgs.git
    ```
1. Install [Blickfeld API](https://docs.blickfeld.com/cube/latest/external/blickfeld-scanner-lib/install.html)
1. Install ROS dependencies
    ```bash
    sudo apt install ros-noetic-tf2-msgs ros-noetic-tf2-sensor-msgs gpsd gpsd-clients libgps-dev 
    ```
1. Build the ROS package
    ```bash
    cd <ros_workspace>
    catkin build modelt_fcw patchworkpp event_msgs detection_msgs
    # for kaohsiung modelt project
    catkin build sensor_calib modelt_fcw velo2cam_calibration acsc_calibration blickfeld_driver gscam bev360_calibration gpsd_client rtsp_ros_driver patchworkpp redis_publisher
    ```
### Setup Triton Server and Model Conversion
Follow the [instruction from yolov7 deployment](https://github.com/WongKinYiu/yolov7/blob/main/deploy/triton-inference-server/README.md).

#### Linker Triton Server Setup:
TODO

### Remote Install
```bash

```
## Run ROS nodes
Source the built workspace before executing nodes
```bash
cd <ros_workspace>
source devel/setup.bash
```
or put this line in `~/.bashrc`
```bash
echo "source ~/<ros_workspace>/devel/setup.bash" >> ~/.bashrc
```

To run the codes, launch sensor drivers or rosbag

1. Object Detection
This node runs yolov7 with COCO pre-train dataset that comes with 80 categories. 
    ```bash
    roslaunch modelt_fcw detect.launch
    ```
1. Object Tracking
    ```bash
    roslaunch modelt_fcw track.launch
    ```
1. Forward Collisin Warning
    ```bash
    roslaunch modelt_fcw sensor_fusion_torch.launch
    ```
    * If you run the sensor streaming, set `sim_time` to false.
    * If you run rosbag, set `sim_time` to true and use `--clock` to play bag.
### Triton Client Node
Launch the triton server docker container:
```bash
cd <tirton_docker_ws>

docker run --gpus all --rm --ipc=host --shm-size=1g --ulimit memlock=-1 --ulimit stack=67108864 -p8000:8000 -p8001:8001 -p8002:8002 -v$(pwd)/triton-deploy/models:/models triton_test tritonserver --model-repository=/models --strict-model-config=false --log-verbose 1
```
Make sure the models(`YoloV7`,`YoloPV2`,`YoloX`,`Segformer`) are available under `$(pwd)/triton-deploy/models` directory. 

Should be:
```bash
triton-deploy/
└── models
    ├── segformer
    │   ├── 1
    │   │   └── model.onnx
    │   └── config.pbtxt
    ├── yolopv2
    │   ├── 1
    │   │   ├── ckpt
    │   │   │   └── yolopv2.pt
    │   │   ├── model.py
    │   │   ├── __pycache__
    │   │   │   └── model.cpython-38.pyc
    │   │   ├── tracker
    │   │   │   ├── __pycache__
    │   │   │   │   └── sort.cpython-38.pyc
    │   │   │   └── sort.py
    │   │   └── utils
    │   │       ├── __init__.py
    │   │       ├── __pycache__
    │   │       │   ├── __init__.cpython-38.pyc
    │   │       │   └── utils.cpython-38.pyc
    │   │       └── utils.py
    │   └── config.pbtxt
    ├── yolov7
    │   ├── 1
    │   │   └── model.plan
    │   └── config.pbtxt
    └── yolox
        ├── 1
        │   ├── best_ckpt.pth
        │   ├── config.py
        │   ├── model.py
        │   └── __pycache__
        │       ├── config.cpython-38.pyc
        │       └── model.cpython-38.pyc
        └── config.pbtxt
```
It will take a while(~1 min) to fully load models into the triton container. After Triton Server is up, you should be able to check the models availability in the terminal:
```bash
+-----------+---------+--------+
| Model     | Version | Status |
+-----------+---------+--------+
| segformer | 1       | READY  |
| yolopv2   | 1       | READY  |
| yolov7    | 1       | READY  |
| yolox     | 1       | READY  |
+-----------+---------+--------+

```

#### ROS Triton Client Example
* Sensor Fusion (YoloV7 object detection + pointcloud projection + 2D tracking + 3D tracking)
    ```bash
    roslaunch modelt_fcw sensor_fusion_yolov7.launch
    ```
* Sensor Fusion (YoloV7 object detection + YoloPV2 lane detection + segformer bus stop detection + pointcloud projection + 2D tracking + 3D tracking)
    prerequest: image stream + pointcloud2
    ```bash
    roslaunch modelt_fcw sensor_fusion_pv2seg.launch
    ```
* Bird's eye view parking space detection - YoloX
    prerequest: bird's eye view stream
    ```bash
    roslaunch modelt_fcw triton_yolox_detect.launch
    ```

### Tests
To test the custom built message,
1. Launch the ROS master in one terminal:
    ```bash
    roscore
    ```
1. In a new terminal, launch the fake event publisher node:
    ```bash
    rosrun modelt_fcw <>_event_pub.py
    ```
1. There are three ways to check published message:
    1. CLI:
        ```bash
        rostopic echo /smart_city/red_lane_event /smart_city/bus_stop_event
        ```
    1. ROS GUI:
        ```bash
        rqt
        ```
    1. Python script:
        ```bash
        rosrun modelt_fcw <>_event_sub.py
        ```

# Linker Repo Template

## Docker Setup

Linkers may use this template repo as cornerstone of new project.

## How?

Click [here](https://help.github.com/en/github/creating-cloning-and-archiving-repositories/creating-a-repository-from-a-template
) for a tutorial on how to create a repository from such a template.

## Development Reminder:

Besides main code itself and tests, you may need to take care the follows

0. Put main code in package and rename `package_template` to your-package-name

1. setup.cfg:

    change `package_template` to your main code folder's name

2. setup.py

    change necessary informations, like **name** and **install_requires**, etc.

3. Makefile:

    Folders for formatting

4. `requirements.txt` and `requirements-dev.txt`

    We assume repo is to be `pip-installable`.

    In `requirements.txt`, only contains essential packages, e.g. `numpy`.

    Other packages help developing, like tests, debug, format, we put in `requirements-dev.txt`, e.g. `pytest`, `fire`.

### One-command setup

We offer a script(`setup.sh`) to help setup new repository, if a `package-name` is given then it will replace all `package_template` with the name, otherwise it will use the directory name by default. This script will also cleanup `README.md`, like it will remove lines from start till line `# package_template`.

```sh
./setup.sh [package-name]
```

Run `setup.sh`, it will do all setup works, git add/log and remove itself. Then you could push to remote repository. If it is not what you expected, `git reset --hard HEAD^` will get all stuff back. You can try it like,

```sh
cp -r template_python_project tpp
cd tpp
./setup.sh
```

# package_template

## Summary

Some utilities.

- logging
- time

## Installation

### Requirement

> Python >= 3.6

### Local install

Clone this repo and install packages in the virtual environment of your choice.
```shell
git clone https://github.com/linkernetworks/repo-name.git
cd repo-name
make install
```

### Remote install

This is a private repo, so you would need a credential for pip-installation. Replace **`ACCOUNT_NAME`** and **`ACCESS_TOKEN`** of your own. Also, add **`TAG`** of certain release/tag of this package.



```shell
pip install git+https://LinkernetworksDev:${GITTOKEN}@github.com/linkernetworks/repo-name.git@{TAG}

```
You can get `GITTOKEN` via

```
export GITTOKEN=$(docker run --rm  mcr.microsoft.com/azure-cli /bin/bash -c 'az login --output none && echo $(az keyvault secret show --name "git" --vault-name "linker-machine-users" --query "value" | xargs )' || echo  "Token not found.")  && echo "FIND token: $GITTOKEN"
```

## Examples

1. Use `export LINKER_LOG_LEVEL=INFO` to control/open logger
2. Save log to file via assigning log dir: `export LINKER_LOG_TO_DIR=/tmp`
3. `LINKER_LOG_LEVEL=INFO python <script>`

## Tests

> Add as more tests as possible for each module

Install first,

```sh
make install
# or
pip install .
# or
pip install -r requirements.txt
python setup.py install
```

### Packages for developing

```sh
make install-dev
# or
pip install -r requirements-dev.txt
```

### Run tests

```sh
pytest
```

## Version

Users are able to setup package version within `VERSION`. By default, after installation package automatically records current git HEAD sha and commit time. To get them from package like,

```python
import package_template
package_template.__version__ # get version
package_template.__git_version__ # git HEAD branch:sha
package_template.__commit_time__ # git HEAD commit time
```


## Download models
[`yolov7.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt) [`yolov7x.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7x.pt) [`yolov7-w6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-w6.pt) [`yolov7-e6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6.pt) [`yolov7-d6.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-d6.pt) [`yolov7-e6e.pt`](https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-e6e.pt)