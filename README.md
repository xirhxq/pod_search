# pod_search

ROS package for pod-related works in inspection task of MBZIRC2024 demostration phase by Fly Eagle.

### Installation

1. Into the `src` dirtectory in your ROS workspace.

    If you want to implement `pod_search` in a existing workspace.
    ```bash
    cd <YOUR_WORKSPACE_DIR>/src
    ```

    Or you can make a new workspace
    ```bash
    mkdir -p <YOUR_WORKSPACE_DIR>/src && cd <YOUR_WORKSPACE_DIR>/src
    catkin_init_workspace
    ```
2. Clone the repository.
    ```bash
    git clone git@github.com:xirhxq/pod_search.git
    ```

3. Install the requirements
    ```bash
    cd pod_search
    pip3 install -r requirements.txt
    ```

### Files

#### Headers

- `AutoTra.py`: Calculate searching trajectory of pod in a rectangle area ahead.

- ~~`Classifier.py`~~: **(decaprecated)**

- `DataLogger.py`: Log datas into .csv format.

- `LocatingEKF.py`: EKF for locating target vessel or USV.

- `PID.py`: Simple PID.

- `PodAngles.py`: Struct for pod state, including pitch & yaw & hfov & maxrate & laser on/off.

- `PodParas.py`: Parameters for pod controlling.

- `QuaternionBuffer.py`: Buffering geometry_msgs/Quaternion messages for a certain time delay.

- `SearchPoint.py`: Struct for UAV hovering points, including xyz position & yaw & max hovering time.

- `TimeBuffer.py`: Buffering std_msgs/Float32 messages for a certain time delay.

- `Utils.py`: Commonly used colors. 

#### Nodes

- `PodComm.py`: Communication with pod.

- `PodInput.py`: Simple script to manually control pod angles.

- `PodSearch.py`: **Main script**. Pod searching in inspection task.

- ~~`Transformer.py`~~: **(decaprecated)**