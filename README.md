# Action Service and Goal Service

This project contains two ROS nodes:
1. **Action Service**: A node that controls the robot's motion by processing action goals.
2. **Goal Service**: A node that provides the coordinates of the last goal sent by the user.

---

## **Features**

### **Action Service**
- Moves the robot to a target position based on received goals.
- Uses a state machine with the following states:
  1. **Fix Yaw**: Adjusts the robot's orientation towards the target.
  2. **Go Straight Ahead**: Moves the robot forward towards the target.
  3. **Done**: Stops the robot when the target is reached.
- Dynamically updates the goal parameters during runtime.

### **Goal Service**
- Provides the coordinates of the last target set by the user.
- Allows external nodes or users to query the most recent goal.

---

## **Dependencies**

Both nodes require the following ROS packages:
- `rospy`: ROS Python client library.
- `geometry_msgs`: To handle position and velocity messages (`Twist` and `Point`).
- `nav_msgs`: For odometry messages.
- `sensor_msgs`: For LaserScan (if needed).
- `tf`: For quaternion and yaw transformations.
- `std_srvs`: For service communication.

---

## **Setup**

1. Clone the repository into your ROS workspace:
   ```bash
   cd ~/my_ws/src
   git clone <repository-url>
