# regulated_pure_pursuit_controller
Regulated Pure Pursuit Controller for ROS. Applicable for both [move_base](http://wiki.ros.org/move_base) and [move_base_flex](http://wiki.ros.org/move_base_flex). Tested with ROS-Noetic.

## Update 
Implement [vector_pure_pursuit](https://github.com/blackcoffeerobotics/vector_pursuit_controller). To use it, set the parameter `use_vector_pure_pursuit` to `true` and follow it's [Configuration Guide](https://docs.ros.org/en/humble/p/vector_pursuit_controller/)

# Installation
* Install move_base_flex
    ```sh
    sudo apt install ros-noetic-mbf-costmap-nav
    ```
* Clone code
    ```sh
    cd your_ws/src
    git clone https://github.com/datledoan/regulated_pure_pursuit_controller_ros.git
    catkin build
    ```
* See its [Configuration Guide Page](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)

# Demo

|   regulated_pure_pursuit  |   vector_pure_pursuit |
| --- | --- |
| <video src="https://github.com/user-attachments/assets/12fc6db3-852a-41f5-ba8a-dff95c00e0d3">  | <video src="https://github.com/user-attachments/assets/ad69bfca-6a1c-4576-814b-242fe073b841">  |
# Reference
- [nav2_regulated_pure_pursuit_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)
- [regulated_pure_pursuit_controller](https://github.com/JohnTGZ/regulated_pure_pursuit_controller)
- [vector_pure_pursuit](https://github.com/blackcoffeerobotics/vector_pursuit_controller)
- S. Macenski, S. Singh, F. Martin, J. Gines, [**Regulated Pure Pursuit for Robot Path Tracking**](https://arxiv.org/abs/2305.20026). Autonomous Robots, 2023.

```bibtex
@article{macenski2023regulated,
      title={Regulated Pure Pursuit for Robot Path Tracking}, 
      author={Steve Macenski and Shrijit Singh and Francisco Martin and Jonatan Gines},
      year={2023},
      journal = {Autonomous Robots}
}
```
