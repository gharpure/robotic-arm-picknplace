
## Download

  ```
  git clone https://github.com/gharpure/robotic-arm-picknplace.git
  ```

## Installation

After downloading all the files, build using `catkin build`

  ```
  cd robotic-arm-picknplace
  source /opt/ros/noetic/setup.bash
  catkin build
  source devel/setup.bash
  ```
Give execution permission to the scripts

  ```
  chmod +x spawner/spawner_1.py
  chmod +x spawner/spawner_2.py

  chmod +x src/ur5/ur5_gazebo/scripts/send_joints.py
  chmod +x src/ur5/ur5_gazebo/scripts/vision.py
  ```

## Usage

Launch the gazebo world

  ```
  roslaunch ur5_gazebo ur5_world.launch
  ```
  ![image](https://user-images.githubusercontent.com/87241534/225561672-ef9ea265-6eed-4fdd-9b03-70f3b5a5bfa3.png)

Unpause the simulation. Launch vision script

  ```
  rosrun ur5_gazebo vision.py
  ```
  ![image](https://user-images.githubusercontent.com/87241534/225561781-76d73818-b798-4065-ac72-863f870484f3.png)

You can spawn random legos in random position using the `spawner_1` or `spawner_2` script

  ```
  ./spawner/spawner_1.py # Spawn only one random block at the time
  ```
  ![image](https://user-images.githubusercontent.com/87241534/225561865-9c7496b2-ef3f-4b8c-89cc-c8095e1c187c.png)

  ```
  ./spawner/spawner_2.py # Spawn all 11 blocks at the same time
  ```
  ![image](https://user-images.githubusercontent.com/87241534/225561908-4303118c-4e6c-41ab-9539-e4c9e19512bb.png)

After you spawned lego blocks you can tell the `ur5` to pick them up

  ```
  rosrun ur5_gazebo send_joints.py
  ```
![image](https://user-images.githubusercontent.com/87241534/225562113-6310fc62-21c0-42ea-9f3d-befd97332889.png)
