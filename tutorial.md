# Webots ROS2 Tutorial: Subscriber and Publisher Setup

## Subscriber  
### Understanding ROS 2 Topics  
Refer to the [ROS 2 Foxy Documentation](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

### Steps in Terminals  

**Terminal 1:**  
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch webots_ros2_epuck robot_launch.py  # Opens ePuck in Webots
```

**Terminal 2:**  
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic list  # Displays list of topics
ros2 topic echo /tof  # Echos data from the /tof topic
# Press Ctrl+C to stop
ros2 topic info /tof  # Shows topic info (publisher and subscriber count)
ros2 interface show sensor_msgs/msg/Range  # Displays message structure
```

---

### Creating a Subscriber Node  

**Steps:**  
1. Open a new terminal.  
2. Refer to the code from the [ROS 2 Foxy Publisher and Subscriber Tutorial](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html).  
3. Save the `.py` file and modify `setup.py`.  
4. Build the package:  
   ```bash
   colcon build --packages-select <package_name>
   ```

**Run the Subscriber Node:**  
```bash
ros2 run webots_ros2_epuck <subscriber.py name>
```

**New Terminal:**  
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run webots_ros2_epuck  # Press double tab to see available nodes
ros2 run webots_ros2_epuck publisher_vel  # Runs the ePuck, sensor values change upon obstacle detection
```

---

## Publisher Node Setup  

**Create `publisher_vel.py`:**  
Place this file under `webots_ros2_epuck` in your VSCode workspace.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PublisherVel(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.1
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher_obj = PublisherVel()
    rclpy.spin(publisher_obj)
    publisher_obj.destroy_node()  # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Modify `setup.py`:**  
Add this line:  
```python
entry_points={
    'console_scripts': [
        'publisher_vel = webots_ros2_epuck.publisher_vel:main'
    ],
}
```

**Build and Run:**  
```bash
colcon build --packages-select webots_ros2_epuck
source install/setup.bash
ros2 run webots_ros2_epuck publisher_vel  # This will move the robot
```

--- 

This guide walks you through setting up ROS 2 topics, creating a subscriber, and running a publisher node in the Webots ROS2 environment.
