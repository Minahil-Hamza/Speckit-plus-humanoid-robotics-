// Structured book content organized into modules and chapters
const bookContent = {
    title: "Physical AI & Robotics: A Comprehensive Guide",
    description: "Master Physical AI, Robotics, ROS2, Digital Twins, AI Robot Brains, and Vision-Language-Action models",
    modules: [
        {
            id: "module-1",
            title: "Module 1: Introduction to Physical AI and Robotics",
            description: "Foundational concepts in Physical AI and Robotics",
            chapters: [
                {
                    id: "chapter-1-1",
                    title: "Chapter 1: What is Physical AI?",
                    content: `# What is Physical AI?

Physical AI refers to artificial intelligence systems that interact with and operate in the physical world through robotics. Unlike traditional AI that processes data in isolated environments, Physical AI combines AI algorithms with physical embodiment to enable machines to perceive, understand, and manipulate their environment.

## Key Characteristics

**Embodiment**: Physical AI systems have a physical form (robots) that allows them to interact with the real world.

**Perception**: They use sensors (cameras, LiDAR, tactile sensors) to gather information about their environment.

**Action**: They can manipulate objects and navigate through spaces using actuators and motors.

**Learning**: They improve performance through experience and interaction with the physical world.

## Applications

- Autonomous robots in warehouses and factories
- Humanoid robots for assistance and social interaction
- Self-driving vehicles
- Surgical robots in healthcare
- Agricultural robots for farming automation

Physical AI is crucial for creating intelligent machines that can work alongside humans in the real world, adapting to dynamic and unpredictable environments.`,
                    readingTime: "10 min",
                    keywords: ["physical ai", "embodied ai", "robotics", "autonomous systems"]
                },
                {
                    id: "chapter-1-2",
                    title: "Chapter 2: Overview of Robotics",
                    content: `# Overview of Robotics

Robotics is the interdisciplinary field that combines mechanical engineering, electrical engineering, and computer science to design, build, and program robots.

## Core Components

**Mechanical Structure**: The physical body, joints, and links that form the robot's skeleton.

**Sensors**: Devices that gather information about the environment (cameras, LiDAR, IMUs, force sensors).

**Actuators**: Motors and mechanisms that create movement.

**Control Systems**: Software and algorithms that coordinate robot behavior.

**Power Systems**: Batteries or power supplies that provide energy.

## Types of Robots

**Industrial Robots**: Fixed manipulators for manufacturing (welding, assembly, painting).

**Mobile Robots**: Wheeled or legged platforms that can navigate (delivery robots, autonomous vehicles).

**Humanoid Robots**: Robots with human-like appearance and capabilities.

**Aerial Robots**: Drones and flying robots for inspection and delivery.

**Underwater Robots**: Submersibles for ocean exploration and maintenance.

**Collaborative Robots (Cobots)**: Designed to work safely alongside humans.

## Why Robotics Matters

Robotics enables automation of dangerous, repetitive, or precision-critical tasks. As AI capabilities advance, robots become increasingly capable of handling complex, unstructured environments.`,
                    readingTime: "12 min",
                    keywords: ["robotics", "robot types", "sensors", "actuators"]
                },
                {
                    id: "chapter-1-3",
                    title: "Chapter 3: History and Evolution",
                    content: `# History and Evolution of Robotics

## Early Beginnings (1950s-1970s)

**1954**: George Devol invented the first programmable robot, Unimate.

**1961**: Unimate was installed in a General Motors plant, beginning industrial robotics.

**1970s**: Stanford Cart demonstrated early autonomous navigation.

## Classical Robotics Era (1980s-2000s)

**1980s**: Introduction of robot programming languages and vision systems.

**1990s**: Development of behavior-based robotics and reactive systems.

**2000s**: ROS (Robot Operating System) was created, standardizing robot software development.

## Modern AI-Powered Robotics (2010s-Present)

**2010s**: Deep learning revolutionized computer vision and perception.

**2015**: Boston Dynamics demonstrated advanced legged robots (Atlas, Spot).

**2020s**: Emergence of VLA (Vision-Language-Action) models enabling robots to understand natural language commands.

**2023-Present**: Foundation models and large language models integrated into robotics, enabling more intelligent and adaptable behavior.

## Key Milestones

- **Industrial Automation**: Transformed manufacturing efficiency
- **Space Exploration**: Mars rovers demonstrating autonomous operation
- **Autonomous Vehicles**: Self-driving cars becoming reality
- **Humanoid Robots**: Advancing toward general-purpose assistance
- **AI Integration**: Modern robots combining perception, reasoning, and action

The field continues to evolve rapidly as AI capabilities advance and hardware becomes more sophisticated and affordable.`,
                    readingTime: "15 min",
                    keywords: ["robotics history", "robot evolution", "milestones"]
                }
            ]
        },
        {
            id: "module-2",
            title: "Module 2: ROS2 - Robot Operating System",
            description: "Master ROS2, the modern framework for robot software development",
            chapters: [
                {
                    id: "chapter-2-1",
                    title: "Chapter 1: Introduction to ROS2",
                    content: `# Introduction to ROS2

ROS2 (Robot Operating System 2) is an open-source framework for building robot software. It provides tools, libraries, and conventions for creating complex robot applications.

## Why ROS2?

**Not Actually an OS**: Despite the name, ROS2 is a middleware framework running on top of Linux, Windows, or macOS.

**Improved from ROS1**: ROS2 addresses limitations of the original ROS with better real-time performance, security, and multi-robot support.

**Industry Standard**: Used by universities, research labs, and companies worldwide.

## Key Features

**Distributed System**: Nodes communicate across networks, enabling multi-robot systems.

**Real-Time Support**: Deterministic communication for safety-critical applications.

**Security**: Built-in authentication and encryption (DDS-Security).

**Multi-Platform**: Runs on Linux, Windows, macOS, and embedded systems.

**Language Support**: Python, C++, and other language bindings available.

## Core Concepts

**Nodes**: Independent processes that perform specific tasks.

**Topics**: Named channels for asynchronous message passing (publish/subscribe).

**Services**: Synchronous request/reply communication.

**Actions**: Asynchronous goal-oriented tasks with feedback.

**Parameters**: Configuration values that can be changed at runtime.

ROS2 provides the foundation for building modular, maintainable robot software that scales from simple prototypes to production systems.`,
                    readingTime: "10 min",
                    keywords: ["ros2", "robot operating system", "middleware"]
                },
                {
                    id: "chapter-2-2",
                    title: "Chapter 2: ROS2 Architecture",
                    content: `# ROS2 Architecture

Understanding ROS2's architecture is essential for building robust robot applications.

## Communication Layer (DDS)

**Data Distribution Service (DDS)**: The underlying communication protocol that enables real-time, peer-to-peer data exchange.

**Quality of Service (QoS)**: Configure reliability, durability, and other communication properties.

**Discovery**: Automatic detection of nodes and topics on the network.

## Node Architecture

**Single Purpose**: Each node should have one clear responsibility.

**Composition**: Nodes can be combined into single processes for efficiency.

**Lifecycle**: Managed node states (Unconfigured, Inactive, Active, Finalized).

## Communication Patterns

**Topics (Pub/Sub)**:
- Many-to-many asynchronous communication
- Fire-and-forget messaging
- Best for sensor data, status updates

**Services (Req/Rep)**:
- One-to-one synchronous communication
- Blocks until response received
- Best for configuration, queries

**Actions**:
- Asynchronous goal-based tasks
- Provides feedback and can be cancelled
- Best for long-running operations (navigation, manipulation)

## Package Structure

\`\`\`
my_robot_package/
├── package.xml          # Package metadata
├── CMakeLists.txt       # Build configuration (C++)
├── setup.py             # Build configuration (Python)
├── resource/
├── launch/              # Launch files
├── config/              # Configuration files
├── src/                 # Source code
└── include/             # Header files (C++)
\`\`\`

## Build System (colcon)

**Workspace**: Directory containing multiple packages.

**Build**: Compile packages and generate executables.

**Install**: Create deployment-ready binaries.

**Source**: Setup environment to use built packages.

Understanding this architecture enables you to design scalable, maintainable robot systems.`,
                    readingTime: "15 min",
                    keywords: ["ros2 architecture", "dds", "nodes", "topics"]
                },
                {
                    id: "chapter-2-3",
                    title: "Chapter 3: Creating Your First ROS2 Package",
                    content: `# Creating Your First ROS2 Package

Let's build a simple ROS2 package to understand the basics.

## Setup Workspace

\`\`\`bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create package
ros2 pkg create --build-type ament_python my_first_package
\`\`\`

## Publisher Node (Python)

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
\`\`\`

## Subscriber Node (Python)

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
\`\`\`

## Building and Running

\`\`\`bash
# Build workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash

# Run publisher (terminal 1)
ros2 run my_first_package publisher

# Run subscriber (terminal 2)
ros2 run my_first_package subscriber
\`\`\`

## Verification Commands

\`\`\`bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# View topic messages
ros2 topic echo /my_topic

# View node info
ros2 node info /simple_publisher
\`\`\`

This example demonstrates the fundamental ROS2 publish-subscribe pattern. You can extend this to create more complex robot behaviors.`,
                    readingTime: "20 min",
                    keywords: ["ros2 package", "publisher", "subscriber", "tutorial"]
                }
            ]
        },
        {
            id: "module-3",
            title: "Module 3: Robot Simulation and Digital Twins",
            description: "Learn simulation tools including Gazebo, Isaac Sim, and Digital Twin concepts",
            chapters: [
                {
                    id: "chapter-3-1",
                    title: "Chapter 1: Introduction to Robot Simulation",
                    content: `# Introduction to Robot Simulation

Robot simulation enables testing and development in virtual environments before deploying to physical hardware.

## Why Simulation?

**Cost Reduction**: Test without risking expensive hardware.

**Safety**: Validate dangerous scenarios without risk to people or equipment.

**Speed**: Iterate faster than real-world testing allows.

**Reproducibility**: Create consistent test conditions.

**Accessibility**: Develop without needing physical robots.

## Popular Simulators

**Gazebo**: Open-source, physics-based, ROS-integrated simulator.

**Isaac Sim**: NVIDIA's photorealistic simulator with GPU acceleration.

**Unity**: Game engine adapted for robotics with ML-Agents.

**Webots**: Professional robot simulator with comprehensive robot models.

**CoppeliaSim (V-REP)**: Versatile simulator for research and education.

## Key Simulation Components

**Physics Engine**: Simulates gravity, collisions, friction (ODE, Bullet, PhysX).

**Rendering**: Visualizes the scene (OpenGL, Vulkan, ray tracing).

**Sensor Models**: Simulates cameras, LiDAR, IMU, GPS, etc.

**Environment**: Models of worlds, buildings, obstacles.

**Robot Models**: URDF/SDF descriptions of robot geometry and dynamics.

## Sim-to-Real Gap

**Challenge**: Behaviors working in simulation may fail on real robots.

**Causes**: Perfect sensors, simplified physics, lack of real-world noise.

**Solutions**:
- Domain randomization
- Realistic sensor noise
- Accurate physics parameters
- Real-world validation

Simulation is an essential tool in modern robotics development, bridging the gap between algorithm design and real-world deployment.`,
                    readingTime: "12 min",
                    keywords: ["simulation", "gazebo", "isaac sim", "digital twin"]
                },
                {
                    id: "chapter-3-2",
                    title: "Chapter 2: Gazebo Simulator",
                    content: `# Gazebo Simulator

Gazebo is the most widely used open-source robot simulator, tightly integrated with ROS.

## Gazebo Features

**3D Visualization**: Realistic rendering of robots and environments.

**Physics Simulation**: ODE, Bullet, Simbody, or DART physics engines.

**Sensor Simulation**: Cameras, depth sensors, LiDAR, IMU, GPS.

**Robot Plugins**: Custom behavior through C++ plugins.

**ROS Integration**: Seamless connection with ROS/ROS2.

## Gazebo Architecture

**Server (gzserver)**: Runs physics simulation and sensor generation.

**Client (gzclient)**: Provides visualization interface.

**World Files**: SDF (Simulation Description Format) defines environments.

**Model Files**: SDF/URDF defines robot structure.

## Creating a Simple World

\`\`\`xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1 1 1</size></box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
\`\`\`

## Launching Gazebo with ROS2

\`\`\`python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'my_world.world'],
            output='screen'
        )
    ])
\`\`\`

## Common Gazebo Sensors

**Camera**: RGB images
**Depth Camera**: RGB-D data (like Kinect)
**LiDAR**: 2D/3D point clouds
**IMU**: Acceleration and angular velocity
**GPS**: Global positioning
**Contact**: Collision detection

## Performance Tips

- Use headless mode (gzserver only) for faster simulation
- Adjust real-time factor for speed vs. accuracy
- Reduce rendering quality when visualization isn't needed
- Use simpler physics engines for less critical scenarios

Gazebo provides a powerful, free platform for robot development and testing.`,
                    readingTime: "18 min",
                    keywords: ["gazebo", "simulation", "sdf", "robot models"]
                },
                {
                    id: "chapter-3-3",
                    title: "Chapter 3: NVIDIA Isaac Sim and Digital Twins",
                    content: `# NVIDIA Isaac Sim and Digital Twins

Isaac Sim is NVIDIA's cutting-edge robotics simulator built on Omniverse, offering photorealistic rendering and GPU-accelerated physics.

## Isaac Sim Features

**Photorealistic Rendering**: Ray-traced graphics for realistic scenes.

**GPU Acceleration**: Fast physics simulation using NVIDIA PhysX 5.

**AI Integration**: Built-in support for training and deploying AI models.

**ROS/ROS2 Bridge**: Native integration with ROS ecosystem.

**Synthetic Data Generation**: Create labeled datasets for ML training.

**Multi-Robot Simulation**: Efficiently simulate fleets of robots.

## Digital Twin Concept

A Digital Twin is a virtual replica of a physical system that mirrors its behavior, state, and environment.

**Benefits**:
- Test algorithms safely before deployment
- Predict maintenance needs
- Optimize performance
- Train AI models with synthetic data
- Validate system behavior

**Components**:
1. **Physical Asset**: The real robot
2. **Virtual Model**: Accurate simulation in Isaac Sim
3. **Data Connection**: Bidirectional data flow
4. **Analytics**: Monitoring and optimization

## Isaac Sim Workflow

\`\`\`python
# Basic Isaac Sim Python example
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot

# Create world
world = World()

# Add robot
robot = Robot(prim_path="/World/my_robot")

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
\`\`\`

## Integration with Isaac ROS

Isaac ROS provides GPU-accelerated ROS2 packages:
- **Isaac ROS Visual SLAM**: Real-time SLAM
- **Isaac ROS Object Detection**: AI-powered perception
- **Isaac ROS Depth Segmentation**: Scene understanding

## Synthetic Data Generation

Generate labeled datasets for training:
- Object detection annotations
- Semantic segmentation masks
- Depth maps
- Instance segmentation

## Use Cases

**Warehouse Automation**: Test navigation in complex environments
**Manipulation**: Validate grasp planning and execution
**Multi-Robot Systems**: Coordinate robot fleets
**Autonomous Vehicles**: Simulate driving scenarios
**Human-Robot Interaction**: Test safe collaboration

Isaac Sim and Digital Twins represent the future of robot development, enabling faster iteration and more reliable deployments.`,
                    readingTime: "20 min",
                    keywords: ["isaac sim", "digital twin", "nvidia", "omniverse"]
                }
            ]
        },
        {
            id: "module-4",
            title: "Module 4: Robot Perception and Computer Vision",
            description: "Master computer vision, sensor processing, and AI-powered perception",
            chapters: [
                {
                    id: "chapter-4-1",
                    title: "Chapter 1: Computer Vision Fundamentals",
                    content: `# Computer Vision Fundamentals

Computer vision enables robots to extract meaningful information from visual data.

## Core Concepts

**Image Representation**: Pixels arranged in grids with color/intensity values.

**Color Spaces**: RGB, HSV, YUV for different processing needs.

**Image Processing**: Filtering, edge detection, morphological operations.

**Feature Extraction**: Identifying distinctive points, edges, regions.

**Object Recognition**: Classifying and localizing objects in images.

## Classical Computer Vision

**Edge Detection**: Sobel, Canny algorithms find boundaries.

**Feature Detectors**: SIFT, SURF, ORB identify keypoints.

**Template Matching**: Find patterns in images.

**Optical Flow**: Track motion between frames.

## Deep Learning for Vision

**Convolutional Neural Networks (CNNs)**: Process spatial data efficiently.

**Object Detection**: YOLO, R-CNN, SSD for real-time detection.

**Semantic Segmentation**: Label every pixel (FCN, U-Net, DeepLab).

**Instance Segmentation**: Separate individual objects (Mask R-CNN).

## OpenCV in Robotics

\`\`\`python
import cv2
import numpy as np

# Read image
image = cv2.imread('robot_camera.jpg')

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Edge detection
edges = cv2.Canny(gray, 100, 200)

# Object detection with cascade classifier
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
faces = face_cascade.detectMultiScale(gray, 1.3, 5)

# Draw rectangles around detected objects
for (x, y, w, h) in faces:
    cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 2)
\`\`\`

## ROS2 Image Processing

\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.br = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, 'bgr8')

        # Process image
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image
        ros_image = self.br.cv2_to_imgmsg(edges, encoding='mono8')
        self.publisher.publish(ros_image)
\`\`\`

## Applications in Robotics

**Navigation**: Obstacle detection, path planning
**Manipulation**: Object recognition, grasp pose estimation
**Inspection**: Quality control, defect detection
**Tracking**: Following objects or people
**Localization**: Visual odometry, landmark detection

Computer vision is fundamental to enabling robots to understand and interact with their environment.`,
                    readingTime: "15 min",
                    keywords: ["computer vision", "opencv", "image processing", "perception"]
                },
                {
                    id: "chapter-4-2",
                    title: "Chapter 2: 3D Perception and Point Clouds",
                    content: `# 3D Perception and Point Clouds

3D perception enables robots to understand depth and spatial relationships.

## Depth Sensing Technologies

**Stereo Vision**: Two cameras triangulate depth (like human eyes).

**Time-of-Flight (ToF)**: Measures light travel time to calculate distance.

**Structured Light**: Projects patterns and measures distortion.

**LiDAR**: Uses laser pulses for precise distance measurements.

## Point Cloud Representation

A point cloud is a collection of 3D points representing object surfaces and scenes.

**Point Structure**: Each point has (x, y, z) coordinates and optional color/intensity.

**Formats**: PCD (Point Cloud Data), PLY, LAS.

**Size**: Can contain millions of points requiring efficient processing.

## Point Cloud Library (PCL)

PCL is the standard library for point cloud processing in robotics.

\`\`\`cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Create point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

// Downsample with voxel grid
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setInputCloud(cloud);
voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
voxel_filter.filter(*cloud_filtered);
\`\`\`

## Common Operations

**Filtering**:
- Voxel grid downsampling
- Statistical outlier removal
- Pass-through filtering

**Segmentation**:
- Plane detection (RANSAC)
- Cluster extraction
- Region growing

**Registration**:
- ICP (Iterative Closest Point)
- Feature-based alignment
- Global registration

**Feature Extraction**:
- Normal estimation
- FPFH descriptors
- VFH descriptors

## ROS2 Point Cloud Processing

\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.cloud_callback, 10)

    def cloud_callback(self, msg):
        # Convert ROS PointCloud2 to Python
        points = list(pc2.read_points(msg, field_names=("x", "y", "z")))

        # Process points
        for point in points:
            x, y, z = point
            # Filter or process points
            if z < 2.0:  # Only points closer than 2m
                # Do something with point
                pass
\`\`\`

## Applications

**Object Detection**: Identify and locate objects in 3D space.

**Obstacle Avoidance**: Detect collision risks.

**Mapping**: Build 3D maps of environments (SLAM).

**Bin Picking**: Locate objects in cluttered bins.

**Scene Understanding**: Comprehend 3D spatial relationships.

3D perception is essential for robots operating in complex, dynamic environments.`,
                    readingTime: "18 min",
                    keywords: ["point cloud", "3d perception", "lidar", "pcl", "depth sensing"]
                },
                {
                    id: "chapter-4-3",
                    title: "Chapter 3: Advanced Perception Techniques",
                    content: `# Advanced Perception Techniques

Modern robotics leverages advanced AI-powered perception for robust scene understanding.

## Object Detection and Recognition

**YOLO (You Only Look Once)**: Real-time object detection.

\`\`\`python
from ultralytics import YOLO
import cv2

# Load model
model = YOLO('yolov8n.pt')

# Run inference
results = model('robot_view.jpg')

# Process results
for result in results:
    boxes = result.boxes
    for box in boxes:
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0]
        print(f"Detected {model.names[class_id]} with {confidence:.2f} confidence")
\`\`\`

**R-CNN Family**: Region-based detection for higher accuracy.

**SSD (Single Shot Detector)**: Balance speed and accuracy.

## Semantic Segmentation

Classify each pixel in an image.

**DeepLab**: Atrous convolution for semantic segmentation.

**U-Net**: Encoder-decoder architecture for dense prediction.

**Mask R-CNN**: Instance segmentation (separate individual objects).

\`\`\`python
import torch
from torchvision.models.segmentation import deeplabv3_resnet101

# Load pre-trained model
model = deeplabv3_resnet101(pretrained=True)
model.eval()

# Process image
input_tensor = preprocess(image)
output = model(input_tensor)['out'][0]
predictions = output.argmax(0)
\`\`\`

## Depth Estimation

Estimate depth from single images or stereo pairs.

**Monocular Depth Estimation**:
- MiDaS (Mixed Data Sampling)
- DPT (Dense Prediction Transformer)
- ZoeDepth

\`\`\`python
import torch
from transformers import DPTImageProcessor, DPTForDepthEstimation

processor = DPTImageProcessor.from_pretrained("Intel/dpt-large")
model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")

# Prepare image
inputs = processor(images=image, return_tensors="pt")

# Predict depth
with torch.no_grad():
    outputs = model(**inputs)
    predicted_depth = outputs.predicted_depth
\`\`\`

**Stereo Vision**: Triangulate depth from two cameras.

## Visual SLAM

Simultaneous Localization and Mapping using cameras.

**ORB-SLAM3**: Feature-based SLAM for monocular, stereo, and RGB-D cameras.

**LSD-SLAM**: Direct method using photometric error.

**RTAB-Map**: RGB-D SLAM with loop closure detection.

## Sensor Fusion

Combine multiple sensors for robust perception.

**Extended Kalman Filter (EKF)**:
\`\`\`python
class RobotEKF:
    def __init__(self):
        self.state = np.zeros(6)  # [x, y, theta, vx, vy, vtheta]
        self.covariance = np.eye(6)

    def predict(self, control, dt):
        # Prediction step
        self.state = self.motion_model(self.state, control, dt)
        self.covariance = self.predict_covariance(dt)

    def update(self, measurement, sensor_type):
        # Update step
        innovation = measurement - self.observation_model(self.state)
        kalman_gain = self.compute_kalman_gain(sensor_type)
        self.state += kalman_gain @ innovation
        self.covariance = self.update_covariance(kalman_gain)
\`\`\`

**Particle Filter**: Non-parametric state estimation.

**Sensor Fusion Architectures**:
- Early fusion (combine raw data)
- Late fusion (combine processed results)
- Deep fusion (learn fusion with neural networks)

## Attention Mechanisms

Focus on relevant image regions.

**Visual Attention**: Learn which parts of image matter most.

**Transformer-based**: Vision Transformers (ViT) for image understanding.

\`\`\`python
from transformers import ViTForImageClassification

model = ViTForImageClassification.from_pretrained('google/vit-base-patch16-224')

# Process with attention
outputs = model(pixel_values, output_attentions=True)
attentions = outputs.attentions  # Attention weights for each layer
\`\`\`

## Active Perception

Robots actively control sensors to improve perception.

**Strategies**:
- Move camera for better viewpoint
- Adjust focus or zoom
- Scan environment systematically
- Explore uncertain areas

## Multi-Modal Perception

Combine vision with other modalities.

**Vision + Touch**: Tactile sensors for manipulation.

**Vision + Audio**: Sound source localization.

**Vision + LiDAR**: Robust outdoor perception.

Advanced perception techniques enable robots to understand complex, dynamic environments with high reliability.`,
                    readingTime: "20 min",
                    keywords: ["yolo", "semantic segmentation", "depth estimation", "sensor fusion", "slam"]
                }
            ]
        },
        {
            id: "module-5",
            title: "Module 5: AI Robot Brains and VLA Models",
            description: "Explore modern AI approaches including Vision-Language-Action models",
            chapters: [
                {
                    id: "chapter-5-1",
                    title: "Chapter 1: The AI Robot Brain",
                    content: `# The AI Robot Brain

The AI Robot Brain represents the integration of perception, reasoning, and action in robotic systems.

## Architecture Overview

**Perception Layer**: Processes sensor data (vision, touch, audio) to understand the environment.

**Reasoning Layer**: Makes decisions based on goals, context, and learned knowledge.

**Action Layer**: Generates motor commands to achieve desired behaviors.

**Memory Systems**: Stores experiences, skills, and world knowledge.

## Traditional Approach

**Sense-Plan-Act Cycle**:
1. Sense: Gather sensor data
2. Plan: Generate action sequence
3. Act: Execute plan
4. Repeat

**Limitations**: Brittle, slow, difficult to handle uncertainty.

## Modern AI Approach

**End-to-End Learning**: Neural networks map directly from sensors to actions.

**Foundation Models**: Large pre-trained models adapted for robotics tasks.

**Multimodal Integration**: Combine vision, language, and proprioception.

**Continuous Learning**: Improve through interaction and experience.

## Cognitive Architectures

**Behavior Trees**: Hierarchical task decomposition.

**Subsumption Architecture**: Layered reactive behaviors.

**Hybrid Deliberative/Reactive**: Combine planning with reactive responses.

**Neural Architectures**: Deep networks for decision-making.

## Key Capabilities

**Perception**: Object recognition, scene understanding, state estimation.

**Prediction**: Anticipate future states and consequences.

**Planning**: Generate action sequences to achieve goals.

**Learning**: Acquire new skills and adapt to new situations.

**Reasoning**: Understand cause and effect, solve novel problems.

## Example: Robot Manipulation Brain

\`\`\`python
class RobotBrain:
    def __init__(self):
        self.vision_model = VisionEncoder()
        self.language_model = LanguageEncoder()
        self.policy_network = PolicyNetwork()
        self.memory = ExperienceBuffer()

    def perceive(self, image, language_instruction):
        # Process visual input
        visual_features = self.vision_model(image)

        # Process language input
        language_features = self.language_model(language_instruction)

        # Combine multimodal features
        combined = torch.cat([visual_features, language_features])
        return combined

    def decide(self, state):
        # Generate action from policy
        action = self.policy_network(state)
        return action

    def learn(self, experience):
        # Update policy from experience
        self.memory.store(experience)
        if len(self.memory) > batch_size:
            batch = self.memory.sample()
            self.update_policy(batch)
\`\`\`

The AI Robot Brain combines multiple AI technologies to enable intelligent, adaptive behavior in complex environments.`,
                    readingTime: "15 min",
                    keywords: ["ai robot brain", "cognitive architecture", "perception", "planning"]
                },
                {
                    id: "chapter-5-2",
                    title: "Chapter 2: Vision-Language-Action (VLA) Models",
                    content: `# Vision-Language-Action (VLA) Models

VLA models represent a breakthrough in robotics by combining visual perception, natural language understanding, and robotic control.

## What are VLA Models?

VLA models are neural networks that:
- **See**: Process visual input (images, videos)
- **Understand**: Interpret natural language instructions
- **Act**: Generate robot control commands

**Key Innovation**: Direct mapping from vision + language → actions without intermediate symbolic representations.

## Architecture

**Vision Encoder**: Processes images (often ViT - Vision Transformer).

**Language Encoder**: Processes text instructions (often BERT, T5, or GPT variants).

**Cross-Modal Fusion**: Combines vision and language representations.

**Action Decoder**: Generates robot actions (joint angles, gripper commands).

## Training Approaches

**Imitation Learning**: Learn from human demonstrations.

**Reinforcement Learning**: Learn through trial and error with rewards.

**Pre-training**: Use large vision-language datasets, then fine-tune for robotics.

**Co-training**: Simultaneously train on vision-language and robotics tasks.

## Example VLA Models

**RT-1 (Robotics Transformer 1)**: Google's model trained on 130k robot demonstrations.

**RT-2 (Robotics Transformer 2)**: Combines VLA with vision-language models like PaLI-X.

**PaLM-E**: 562B parameter model integrating language and embodied reasoning.

**CLIP-based Models**: Use CLIP for vision-language grounding in robotics.

## Sample Implementation

\`\`\`python
import torch
import torch.nn as nn

class VLAModel(nn.Module):
    def __init__(self):
        super().__init__()
        # Vision encoder
        self.vision_encoder = VisionTransformer(img_size=224, patch_size=16)

        # Language encoder
        self.language_encoder = BERTModel.from_pretrained('bert-base-uncased')

        # Cross-attention fusion
        self.cross_attention = nn.MultiheadAttention(embed_dim=768, num_heads=12)

        # Action decoder
        self.action_decoder = nn.Sequential(
            nn.Linear(768, 512),
            nn.ReLU(),
            nn.Linear(512, 7)  # 7-DOF robot arm
        )

    def forward(self, image, text):
        # Encode image
        visual_features = self.vision_encoder(image)

        # Encode text
        language_features = self.language_encoder(text)

        # Fuse modalities
        fused, _ = self.cross_attention(
            visual_features, language_features, language_features
        )

        # Generate action
        action = self.action_decoder(fused.mean(dim=1))
        return action
\`\`\`

## Capabilities

**Natural Language Control**: "Pick up the red block and place it in the blue box"

**Generalization**: Transfer to new objects and environments

**Complex Tasks**: Multi-step manipulation and navigation

**Few-Shot Learning**: Adapt to new tasks with minimal examples

## Challenges

**Data Requirements**: Need large datasets of robot demonstrations

**Sim-to-Real Gap**: Transferring from simulation to real robots

**Safety**: Ensuring reliable operation in unstructured environments

**Computational Cost**: Large models require significant compute

VLA models represent the cutting edge of robot learning, enabling more natural and capable human-robot interaction.`,
                    readingTime: "20 min",
                    keywords: ["vla", "vision language action", "transformer", "multimodal"]
                },
                {
                    id: "chapter-5-3",
                    title: "Chapter 3: Foundation Models in Robotics",
                    content: `# Foundation Models in Robotics

Foundation models are large-scale pre-trained AI models adapted for robotic tasks, bringing powerful capabilities to physical AI.

## What are Foundation Models?

**Definition**: Large neural networks pre-trained on diverse datasets that can be fine-tuned for specific tasks.

**Characteristics**:
- Billions of parameters
- Pre-trained on internet-scale data
- General-purpose capabilities
- Transfer learning to new domains

## Vision Foundation Models

**CLIP (Contrastive Language-Image Pre-training)**: Joint vision-language understanding.

\`\`\`python
import torch
import clip
from PIL import Image

# Load CLIP model
device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

# Encode image
image = preprocess(Image.open("robot_scene.jpg")).unsqueeze(0).to(device)
image_features = model.encode_image(image)

# Encode text
text_options = ["a red cube", "a blue ball", "a green cylinder"]
text = clip.tokenize(text_options).to(device)
text_features = model.encode_text(text)

# Calculate similarity
similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
print(f"Object detected: {text_options[similarity.argmax()]}")
\`\`\`

**DINOv2**: Self-supervised vision features.

**SAM (Segment Anything Model)**: Universal segmentation.

## Language Foundation Models

**GPT Family**: Text generation and understanding.

**LLaMA**: Open-source large language model.

**PaLM**: Pathways Language Model by Google.

**Robot Integration**:
\`\`\`python
from transformers import AutoTokenizer, AutoModelForCausalLM

# Load LLM
tokenizer = AutoTokenizer.from_pretrained("meta-llama/Llama-2-7b-chat-hf")
model = AutoModelForCausalLM.from_pretrained("meta-llama/Llama-2-7b-chat-hf")

# Task decomposition
prompt = """Break down this task into robot actions:
'Go to the kitchen, pick up the red mug, and bring it to the living room table.'"""

inputs = tokenizer(prompt, return_tensors="pt")
outputs = model.generate(**inputs, max_length=200)
plan = tokenizer.decode(outputs[0])
print(plan)
\`\`\`

## Multimodal Foundation Models

**GPT-4V**: Vision and language understanding.

**Gemini**: Google's multimodal model.

**LLaVA**: Large Language and Vision Assistant.

**RT-2**: Robotics Transformer with vision-language pre-training.

## Code as Policies

Use foundation models to generate robot code.

\`\`\`python
class CodeAsPolicy:
    def __init__(self, llm_model):
        self.llm = llm_model
        self.robot_api = RobotAPI()

    def execute_instruction(self, natural_language_command):
        prompt = f"""
Generate Python code using the robot API to execute: {natural_language_command}

Available functions:
- robot.move_to(x, y, z)
- robot.grasp_object(object_name)
- robot.place_object(location)
- robot.open_gripper()
- robot.close_gripper()

Code:
"""
        # Generate code with LLM
        generated_code = self.llm.generate(prompt)

        # Execute generated code
        exec(generated_code, {"robot": self.robot_api})

# Usage
policy = CodeAsPolicy(llm_model)
policy.execute_instruction("pick up the blue block and stack it on the red block")
\`\`\`

## Embodied AI with Foundation Models

**PaLM-E (Embodied)**: 562B parameters combining PaLM with robotic control.

**RT-X**: Cross-embodiment data for robot learning.

**OpenVLA**: Open-source VLA using foundation models.

\`\`\`python
from transformers import AutoModel

# Load embodied AI model
model = AutoModel.from_pretrained("openvla/openvla-7b")

# Process multimodal input
robot_observation = {
    "image": camera_image,
    "language": "pick up the apple",
    "proprioception": robot_joint_states
}

# Generate action
action = model(robot_observation)
robot.execute_action(action)
\`\`\`

## Prompt Engineering for Robotics

Crafting effective prompts for robot control.

**Chain-of-Thought Prompting**:
\`\`\`
Prompt: "To pick up the cup:
1. First, identify the cup location
2. Move arm above the cup
3. Lower arm to grasp height
4. Close gripper
5. Lift cup upward

Now execute step 1..."
\`\`\`

**Few-Shot Learning**:
\`\`\`
Example 1: "move left" → robot.move(-0.1, 0, 0)
Example 2: "move forward" → robot.move(0, 0.1, 0)
Example 3: "grasp" → robot.close_gripper()

Task: "move right"
\`\`\`

## Fine-Tuning for Robotics

Adapt foundation models to robot-specific tasks.

**Low-Rank Adaptation (LoRA)**:
\`\`\`python
from peft import LoraConfig, get_peft_model

# Configure LoRA
lora_config = LoraConfig(
    r=8,  # Low-rank dimension
    lora_alpha=32,
    target_modules=["q_proj", "v_proj"],
    lora_dropout=0.1
)

# Apply LoRA to model
model = get_peft_model(base_model, lora_config)

# Train on robot demonstrations
trainer.train(robot_dataset)
\`\`\`

## Challenges and Limitations

**Computational Requirements**: Large models need significant compute.

**Latency**: Real-time control requires fast inference.

**Grounding**: Connecting abstract concepts to physical actions.

**Safety**: Unpredictable generations require safety constraints.

**Data Efficiency**: Limited robot data compared to internet text.

## Future Directions

**Embodied Foundation Models**: Models pre-trained specifically for robotics.

**Efficient Architectures**: Smaller models with comparable performance.

**Online Learning**: Continual adaptation during deployment.

**Multi-Robot Foundation Models**: Models that transfer across robot types.

Foundation models are transforming robotics by bringing powerful pre-trained capabilities to physical AI systems.`,
                    readingTime: "22 min",
                    keywords: ["foundation models", "clip", "llm", "gpt", "palm-e", "code as policy"]
                }
            ]
        },
        {
            id: "module-6",
            title: "Module 6: Navigation and Path Planning",
            description: "Learn autonomous navigation using Nav2 and path planning algorithms",
            chapters: [
                {
                    id: "chapter-6-1",
                    title: "Chapter 1: Navigation Fundamentals",
                    content: `# Navigation Fundamentals

Robot navigation is the ability to move from one location to another while avoiding obstacles.

## Navigation Components

**Localization**: Knowing where the robot is.

**Mapping**: Building a representation of the environment.

**Path Planning**: Determining the route to the goal.

**Motion Control**: Following the planned path.

**Obstacle Avoidance**: Reacting to dynamic obstacles.

## Localization Methods

**Odometry**: Track wheel rotation to estimate position (accumulates error).

**GPS**: Global positioning (outdoor only, limited accuracy).

**Visual Odometry**: Estimate motion from camera images.

**LiDAR-based**: Use laser scans for position estimation.

**SLAM**: Simultaneously build map and localize within it.

## Mapping Representations

**Occupancy Grid**: 2D grid where each cell is free/occupied/unknown.

**Topological Map**: Nodes (places) connected by edges (paths).

**Metric Map**: Precise geometric representation.

**Semantic Map**: Include object identities and relationships.

## Path Planning Algorithms

**Dijkstra's Algorithm**: Finds shortest path by exploring systematically.

**A\* Search**: Uses heuristics to guide search toward goal.

**RRT (Rapidly-exploring Random Tree)**: Probabilistic sampling for high-dimensional spaces.

**Dynamic Window Approach (DWA)**: Local planning considering robot dynamics.

**TEB (Timed Elastic Band)**: Optimizes trajectory considering time and obstacles.

## Obstacle Avoidance

**Static Obstacles**: Pre-mapped, don't move.

**Dynamic Obstacles**: Moving objects (people, other robots).

**Costmap**: Assigns costs to areas based on obstacles and clearance.

**Local Planner**: Reactive avoidance while following global path.

## Example: Simple Path Planning

\`\`\`python
import numpy as np
from queue import PriorityQueue

def a_star(start, goal, grid):
    """A* path planning on occupancy grid"""
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}

    while not open_set.empty():
        current = open_set.get()[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, grid):
            tentative_g = g_score[current] + distance(current, neighbor)

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                open_set.put((f_score, neighbor))

    return None  # No path found
\`\`\`

Navigation is essential for mobile robots to operate autonomously in real-world environments.`,
                    readingTime: "18 min",
                    keywords: ["navigation", "path planning", "slam", "localization"]
                },
                {
                    id: "chapter-6-2",
                    title: "Chapter 2: ROS2 Nav2 Stack",
                    content: `# ROS2 Nav2 Stack

Nav2 (Navigation2) is the complete navigation system for ROS2, providing all components needed for autonomous navigation.

## Nav2 Architecture

**Behavior Trees**: Coordinate navigation behaviors.

**BT Navigator**: Execute behavior tree-based navigation logic.

**Planner Server**: Global path planning.

**Controller Server**: Local trajectory following.

**Smoother Server**: Smooth planned paths.

**Recovery Server**: Handle navigation failures.

**Waypoint Follower**: Navigate through multiple waypoints.

**Lifecycle Manager**: Manage node lifecycles.

## Core Components

**Costmaps**:
- Global costmap (full map)
- Local costmap (around robot)
- Layers: static map, obstacles, inflation

**Planners**:
- NavFn (Dijkstra)
- Theta\* Planner
- Smac Planner (2D, Hybrid-A\*, Lattice)

**Controllers**:
- DWB (Dynamic Window Approach)
- TEB (Timed Elastic Band)
- MPPI (Model Predictive Path Integral)
- Regulated Pure Pursuit

## Configuration Example

\`\`\`yaml
# controller.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_theta: -3.2
\`\`\`

## Launch Nav2

\`\`\`python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            'nav2_bringup/launch/navigation_launch.py',
            launch_arguments={
                'params_file': 'config/nav2_params.yaml',
                'use_sim_time': 'true'
            }.items()
        )
    ])
\`\`\`

## Sending Navigation Goals

\`\`\`python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.navigator = BasicNavigator()

    def navigate_to_pose(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = np.sin(theta / 2)
        goal_pose.pose.orientation.w = np.cos(theta / 2)

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')
\`\`\`

## Behavior Trees

Nav2 uses behavior trees for complex navigation logic:

\`\`\`xml
<root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <ComputePathToPose goal="{goal}"/>
            <FollowPath path="{path}"/>
        </Sequence>
    </BehaviorTree>
</root>
\`\`\`

## Tuning Tips

**Controller Tuning**: Adjust speeds and accelerations for smooth motion.

**Costmap Configuration**: Balance safety (inflation) vs. navigation in tight spaces.

**Recovery Behaviors**: Set appropriate timeouts and retry strategies.

**Planner Selection**: Choose based on environment (open vs. cluttered).

Nav2 provides a production-ready navigation system that scales from simple to complex autonomous navigation scenarios.`,
                    readingTime: "22 min",
                    keywords: ["nav2", "navigation stack", "ros2 navigation", "autonomous navigation"]
                },
                {
                    id: "chapter-6-3",
                    title: "Chapter 3: SLAM - Simultaneous Localization and Mapping",
                    content: `# SLAM - Simultaneous Localization and Mapping

SLAM enables robots to build maps of unknown environments while simultaneously tracking their location within those maps.

## The SLAM Problem

**Challenge**: The chicken-and-egg problem of robotics.
- Need map to localize
- Need localization to build map
- Must solve both simultaneously

**Mathematical Formulation**:
- Estimate robot poses: x₁, x₂, ..., xₜ
- Estimate map: m
- Given: sensor observations z₁, z₂, ..., zₜ and control inputs u₁, u₂, ..., uₜ

## Types of SLAM

**2D SLAM**: Build 2D occupancy grid maps (for mobile robots).

**3D SLAM**: Build 3D point cloud or mesh maps (for aerial, underwater robots).

**Visual SLAM**: Use cameras as primary sensor.

**LiDAR SLAM**: Use laser rangefinders.

**RGB-D SLAM**: Use depth cameras (Kinect, RealSense).

## Key SLAM Approaches

**Filter-Based SLAM**:
- Extended Kalman Filter (EKF-SLAM)
- Particle Filter (FastSLAM)
- Incremental updates
- Efficient for real-time operation

**Graph-Based SLAM**:
- Pose graph optimization
- Better for large-scale environments
- Loop closure detection and correction

## EKF-SLAM Example

\`\`\`python
import numpy as np

class EKFSLAM:
    def __init__(self):
        # State: [robot_x, robot_y, robot_theta, landmark1_x, landmark1_y, ...]
        self.state = np.zeros(3)  # Initially just robot pose
        self.covariance = np.eye(3) * 0.1

        # Process and measurement noise
        self.R = np.diag([0.1, 0.1, 0.05])  # Motion noise
        self.Q = np.diag([0.1, 0.1])  # Observation noise

    def predict(self, velocity, angular_velocity, dt):
        """Prediction step - robot motion"""
        x, y, theta = self.state[:3]

        # Update robot pose
        self.state[0] += velocity * np.cos(theta) * dt
        self.state[1] += velocity * np.sin(theta) * dt
        self.state[2] += angular_velocity * dt

        # Jacobian of motion model
        G = self.compute_motion_jacobian(velocity, theta, dt)

        # Update covariance
        self.covariance = G @ self.covariance @ G.T + self.R

    def update(self, landmark_observations):
        """Update step - sensor measurements"""
        for obs in landmark_observations:
            landmark_id = obs['id']
            measured_range = obs['range']
            measured_bearing = obs['bearing']

            if self.is_new_landmark(landmark_id):
                self.add_landmark(landmark_id, measured_range, measured_bearing)
            else:
                self.update_landmark(landmark_id, measured_range, measured_bearing)

    def add_landmark(self, landmark_id, r, phi):
        """Add newly observed landmark to map"""
        x, y, theta = self.state[:3]

        # Initialize landmark position
        lx = x + r * np.cos(theta + phi)
        ly = y + r * np.sin(theta + phi)

        # Extend state and covariance
        self.state = np.append(self.state, [lx, ly])
        # ... extend covariance matrix

    def update_landmark(self, landmark_id, measured_r, measured_phi):
        """Update belief with landmark observation"""
        # Predicted observation
        predicted_r, predicted_phi = self.predict_observation(landmark_id)

        # Innovation
        innovation = np.array([
            measured_r - predicted_r,
            self.normalize_angle(measured_phi - predicted_phi)
        ])

        # Kalman gain
        H = self.compute_observation_jacobian(landmark_id)
        S = H @ self.covariance @ H.T + self.Q
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # State update
        self.state += K @ innovation

        # Covariance update
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance
\`\`\`

## Graph-Based SLAM

Represent SLAM as a graph optimization problem.

\`\`\`python
import g2o

class GraphSLAM:
    def __init__(self):
        self.optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE2(g2o.LinearSolverCholmodSE2())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        self.optimizer.set_algorithm(solver)

        self.vertex_count = 0

    def add_robot_pose(self, pose):
        """Add robot pose as vertex"""
        v = g2o.VertexSE2()
        v.set_id(self.vertex_count)
        v.set_estimate(pose)

        if self.vertex_count == 0:
            v.set_fixed(True)  # Fix first pose

        self.optimizer.add_vertex(v)
        self.vertex_count += 1
        return v.id()

    def add_odometry_edge(self, from_id, to_id, measurement, information):
        """Add odometry constraint between poses"""
        edge = g2o.EdgeSE2()
        edge.set_vertex(0, self.optimizer.vertex(from_id))
        edge.set_vertex(1, self.optimizer.vertex(to_id))
        edge.set_measurement(measurement)
        edge.set_information(information)
        self.optimizer.add_edge(edge)

    def add_loop_closure(self, from_id, to_id, measurement, information):
        """Add loop closure constraint"""
        edge = g2o.EdgeSE2()
        edge.set_vertex(0, self.optimizer.vertex(from_id))
        edge.set_vertex(1, self.optimizer.vertex(to_id))
        edge.set_measurement(measurement)
        edge.set_information(information)
        self.optimizer.add_edge(edge)

    def optimize(self, iterations=10):
        """Optimize the pose graph"""
        self.optimizer.initialize_optimization()
        self.optimizer.optimize(iterations)
\`\`\`

## Loop Closure Detection

Recognize when robot returns to previously visited location.

**Methods**:
- **Appearance-based**: Compare current view with past views (Bag of Words, DBoW2)
- **Geometric**: Match point cloud features
- **Learning-based**: Neural network descriptors (NetVLAD)

\`\`\`python
import cv2
from sklearn.neighbors import NearestNeighbors

class LoopClosureDetector:
    def __init__(self):
        self.keyframe_descriptors = []
        self.keyframe_poses = []
        self.orb = cv2.ORB_create(nfeatures=1000)

    def add_keyframe(self, image, pose):
        """Add new keyframe to database"""
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        self.keyframe_descriptors.append(descriptors)
        self.keyframe_poses.append(pose)

    def detect_loop_closure(self, current_image, threshold=0.7):
        """Check if current location matches previous keyframe"""
        kp, desc = self.orb.detectAndCompute(current_image, None)

        best_match_id = None
        best_match_score = 0

        for i, kf_desc in enumerate(self.keyframe_descriptors):
            # Match descriptors
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(desc, kf_desc)

            # Compute matching score
            score = len(matches) / max(len(desc), len(kf_desc))

            if score > threshold and score > best_match_score:
                best_match_score = score
                best_match_id = i

        return best_match_id, best_match_score
\`\`\`

## ROS2 SLAM Packages

**slam_toolbox**: 2D LiDAR SLAM with loop closure.

\`\`\`bash
ros2 launch slam_toolbox online_async_launch.py
\`\`\`

**cartographer**: Google's SLAM solution for 2D/3D.

**rtabmap_ros**: RGB-D SLAM with visual odometry.

**ORB_SLAM3**: State-of-the-art visual SLAM.

## SLAM Configuration Example

\`\`\`yaml
# slam_toolbox configuration
slam_toolbox:
  ros__parameters:
    # Solver parameters
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT

    # Scan matcher
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.5
    minimum_travel_heading: 0.5

    # Loop closure
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
\`\`\`

## SLAM Challenges

**Scale Drift**: Accumulated error in large environments.

**Dynamic Objects**: Moving objects confuse mapping.

**Loop Closure**: Detecting revisited locations reliably.

**Computational Cost**: Real-time performance requirements.

**Perceptual Aliasing**: Different places look similar.

## Advanced SLAM Topics

**Multi-Robot SLAM**: Multiple robots collaboratively build map.

**Semantic SLAM**: Include object labels in maps.

**Dense SLAM**: Build detailed 3D reconstructions.

**Lifelong SLAM**: Maintain maps over extended periods.

SLAM is fundamental to autonomous robot operation, enabling robots to navigate and understand previously unknown environments.`,
                    readingTime: "25 min",
                    keywords: ["slam", "mapping", "localization", "loop closure", "graph slam", "ekf slam"]
                }
            ]
        },
        {
            id: "module-7",
            title: "Module 7: Robot Manipulation",
            description: "Master robot arm control, MoveIt, and manipulation strategies",
            chapters: [
                {
                    id: "chapter-7-1",
                    title: "Chapter 1: Kinematics and Dynamics",
                    content: `# Kinematics and Dynamics

Understanding robot arm motion requires knowledge of kinematics and dynamics.

## Forward Kinematics

Calculates end-effector position from joint angles.

**Given**: Joint angles θ₁, θ₂, ..., θₙ
**Find**: End-effector pose (position + orientation)

**Denavit-Hartenberg (DH) Parameters**: Standard representation of robot geometry.

\`\`\`python
import numpy as np

def forward_kinematics_2dof(theta1, theta2, l1, l2):
    """Forward kinematics for 2-link arm"""
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
    return x, y
\`\`\`

## Inverse Kinematics

Calculates joint angles needed to reach desired end-effector pose.

**Given**: Desired end-effector pose
**Find**: Joint angles θ₁, θ₂, ..., θₙ

**Challenges**:
- Multiple solutions (ambiguity)
- No solution (unreachable)
- Singularities (loss of degrees of freedom)

**Solutions**:
- Analytical (closed-form equations)
- Numerical (iterative optimization)
- Learning-based (neural networks)

\`\`\`python
def inverse_kinematics_2dof(x, y, l1, l2):
    """Inverse kinematics for 2-link arm"""
    # Calculate using law of cosines
    c2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    if abs(c2) > 1:
        return None  # No solution

    # Two possible solutions (elbow up/down)
    theta2_1 = np.arccos(c2)
    theta2_2 = -np.arccos(c2)

    k1 = l1 + l2 * np.cos(theta2_1)
    k2 = l2 * np.sin(theta2_1)
    theta1_1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return theta1_1, theta2_1  # Return elbow-up solution
\`\`\`

## Robot Dynamics

Studies forces and torques required for motion.

**Forward Dynamics**: Given torques, find accelerations.

**Inverse Dynamics**: Given desired motion, find required torques.

**Equations of Motion**: Describe how forces create motion (Newton-Euler, Lagrangian).

## Jacobian Matrix

Relates joint velocities to end-effector velocities.

**Uses**:
- Velocity control
- Force control
- Singularity analysis
- Manipulability analysis

\`\`\`python
def compute_jacobian_2dof(theta1, theta2, l1, l2):
    """Jacobian for 2-link arm"""
    J = np.array([
        [-l1*np.sin(theta1) - l2*np.sin(theta1+theta2), -l2*np.sin(theta1+theta2)],
        [l1*np.cos(theta1) + l2*np.cos(theta1+theta2), l2*np.cos(theta1+theta2)]
    ])
    return J
\`\`\`

## Workspace Analysis

**Reachable Workspace**: All positions the end-effector can reach.

**Dexterous Workspace**: Positions reachable with any orientation.

**Singularities**: Configurations where robot loses degrees of freedom.

Understanding kinematics and dynamics is fundamental to effective robot manipulation control.`,
                    readingTime: "20 min",
                    keywords: ["kinematics", "inverse kinematics", "dynamics", "jacobian"]
                },
                {
                    id: "chapter-7-2",
                    title: "Chapter 2: MoveIt Motion Planning",
                    content: `# MoveIt Motion Planning

MoveIt is the most widely used motion planning framework for ROS, providing comprehensive manipulation capabilities.

## MoveIt Features

**Motion Planning**: Generate collision-free trajectories.

**Inverse Kinematics**: Multiple solver options (KDL, TracIK, TRAC-IK).

**Collision Checking**: Fast collision detection.

**3D Perception**: Integration with depth sensors.

**Grasping**: Grasp pose generation.

**Benchmarking**: Compare planner performance.

## Architecture

**Move Group**: Central node coordinating all components.

**Planning Scene**: Maintains world state and collision objects.

**Planning Pipeline**: Chain planners and adapters.

**Trajectory Execution**: Sends commands to robot controllers.

**Sensor Integration**: Updates planning scene from cameras/depth sensors.

## Motion Planning Algorithms

**Sampling-Based**:
- RRT (Rapidly-exploring Random Tree)
- RRT-Connect
- PRM (Probabilistic Roadmap)
- KPIECE

**Optimization-Based**:
- CHOMP (Covariant Hamiltonian Optimization)
- TrajOpt
- STOMP (Stochastic Trajectory Optimization)

**Search-Based**:
- A\* variants for discrete spaces

## Example: Planning to Pose

\`\`\`python
import moveit_commander
import geometry_msgs.msg

# Initialize MoveIt
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Set pose goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4
pose_goal.orientation.w = 1.0

group.set_pose_target(pose_goal)

# Plan and execute
plan = group.go(wait=True)
group.stop()
group.clear_pose_targets()
\`\`\`

## Collision Objects

\`\`\`python
# Add box obstacle
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "base_link"
box_pose.pose.position.x = 0.5
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.25

scene.add_box("box1", box_pose, size=(0.1, 0.1, 0.5))
\`\`\`

## Cartesian Path Planning

\`\`\`python
waypoints = []

# Start with current pose
wpose = group.get_current_pose().pose

# Move in straight line
wpose.position.z -= 0.1
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += 0.2
waypoints.append(copy.deepcopy(wpose))

# Plan Cartesian path
(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    0.01,  # eef_step (1cm)
    0.0    # jump_threshold (disabled)
)

# Execute if path complete
if fraction == 1.0:
    group.execute(plan, wait=True)
\`\`\`

## Grasp Planning

\`\`\`python
from moveit_msgs.msg import Grasp

# Define grasp pose
grasp = Grasp()
grasp.grasp_pose.header.frame_id = "base_link"
grasp.grasp_pose.pose.position.x = 0.5
grasp.grasp_pose.pose.position.y = 0.0
grasp.grasp_pose.pose.position.z = 0.2

# Pre-grasp approach
grasp.pre_grasp_approach.direction.header.frame_id = "tool_link"
grasp.pre_grasp_approach.direction.vector.z = 1.0
grasp.pre_grasp_approach.min_distance = 0.05
grasp.pre_grasp_approach.desired_distance = 0.1

# Execute grasp
group.pick("object_name", [grasp])
\`\`\`

## Configuration Tips

**Planning Time**: Balance between speed and quality.

**Planner Choice**: RRTConnect for speed, CHOMP for smoothness.

**Collision Padding**: Add safety margin around robot.

**Joint Limits**: Respect velocity and acceleration constraints.

MoveIt provides powerful, production-ready manipulation planning for complex robot arms.`,
                    readingTime: "25 min",
                    keywords: ["moveit", "motion planning", "manipulation", "grasp planning"]
                },
                {
                    id: "chapter-7-3",
                    title: "Chapter 3: Grippers and Grasp Planning",
                    content: `# Grippers and Grasp Planning

Effective manipulation requires both capable end-effectors (grippers) and intelligent grasp planning.

## Types of Grippers

**Parallel Jaw Grippers**: Two parallel fingers for simple grasping.

\`\`\`
Advantages:
- Simple control
- Reliable
- Cost-effective

Limitations:
- Limited adaptability
- Size constraints
\`\`\`

**Multi-Fingered Hands**: Three or more fingers for dexterous manipulation.

**Suction Grippers**: Vacuum-based for flat/smooth objects.

**Magnetic Grippers**: For ferromagnetic materials.

**Soft Grippers**: Compliant materials that conform to objects.

**Anthropomorphic Hands**: Human-like hands (5 fingers, many DOF).

## Gripper Selection Criteria

**Payload**: Maximum weight to lift.

**Size Range**: Min/max object dimensions.

**Material Compatibility**: What surfaces can be grasped.

**Speed**: Open/close cycle time.

**Precision**: Positioning accuracy.

**Durability**: Robustness in harsh environments.

## Grasp Types

**Power Grasp**: Entire hand wraps around object (strength).

**Precision Grasp**: Fingertips hold object (dexterity).

**Pinch Grasp**: Two fingers oppose (small objects).

**Tripod Grasp**: Three contact points (stability).

**Wrap Grasp**: Fingers wrap around cylindrical objects.

## Grasp Quality Metrics

**Force Closure**: Can resist arbitrary external forces.

**Form Closure**: Geometric constraints prevent motion.

**Grasp Wrench Space**: Set of forces/torques gripper can apply.

**Ferrari-Canny Metric**: Minimum force to disturb grasp.

\`\`\`python
import numpy as np

def compute_grasp_quality(contact_points, contact_normals, friction_coeff):
    """
    Compute grasp quality using force closure analysis

    Args:
        contact_points: Nx3 array of contact positions
        contact_normals: Nx3 array of contact normals
        friction_coeff: Friction coefficient

    Returns:
        quality_score: Grasp quality metric
    """
    n_contacts = len(contact_points)

    # Build grasp matrix
    G = []
    for i in range(n_contacts):
        point = contact_points[i]
        normal = contact_normals[i]

        # Force component
        force_part = normal

        # Torque component (r × n)
        torque_part = np.cross(point, normal)

        # Combine into grasp matrix column
        G.append(np.concatenate([force_part, torque_part]))

    G = np.array(G).T

    # Check force closure (rank should be 6 for 3D)
    rank = np.linalg.matrix_rank(G)

    # Compute quality metric
    if rank < 6:
        return 0.0  # No force closure

    # Compute minimum singular value (robustness)
    singular_values = np.linalg.svd(G)[1]
    quality = np.min(singular_values)

    return quality
\`\`\`

## Grasp Planning Approaches

**Analytical Methods**: Use geometric models and heuristics.

**Sampling-Based**: Generate and evaluate many candidate grasps.

**Learning-Based**: Train neural networks on grasp datasets.

**Simulation-Based**: Test grasps in physics simulation.

## Analytical Grasp Planning

\`\`\`python
class AnalyticalGraspPlanner:
    def __init__(self, gripper_width=0.08):
        self.gripper_width = gripper_width

    def plan_parallel_jaw_grasp(self, object_point_cloud):
        """
        Plan grasp for parallel jaw gripper
        """
        # Find object principal axes
        pca = self.compute_pca(object_point_cloud)
        principal_axes = pca.components_

        # Find grasp axis (typically narrowest dimension)
        object_dimensions = self.compute_dimensions(object_point_cloud, principal_axes)
        grasp_axis_idx = np.argmin(object_dimensions)
        grasp_axis = principal_axes[grasp_axis_idx]

        # Find grasp center (object centroid)
        grasp_center = np.mean(object_point_cloud, axis=0)

        # Compute approach direction (along grasp axis)
        approach_direction = grasp_axis

        # Generate grasp pose
        grasp_pose = self.create_grasp_pose(
            grasp_center,
            approach_direction,
            grasp_axis
        )

        return grasp_pose

    def create_grasp_pose(self, position, approach, grasp_axis):
        """Create 6D grasp pose from position and directions"""
        # Build rotation matrix
        z_axis = approach / np.linalg.norm(approach)
        x_axis = grasp_axis / np.linalg.norm(grasp_axis)
        y_axis = np.cross(z_axis, x_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])

        return {
            'position': position,
            'orientation': rotation
        }
\`\`\`

## Learning-Based Grasp Planning

**GraspNet**: Deep learning for 6-DOF grasp pose detection.

**GPD (Grasp Pose Detection)**: CNN for grasp quality prediction.

**Contact-GraspNet**: Predict contact points for grasping.

\`\`\`python
import torch
import torch.nn as nn

class GraspQualityNet(nn.Module):
    """Neural network to predict grasp quality"""

    def __init__(self):
        super().__init__()

        # Point cloud encoder
        self.point_encoder = nn.Sequential(
            nn.Conv1d(3, 64, 1),
            nn.ReLU(),
            nn.Conv1d(64, 128, 1),
            nn.ReLU(),
            nn.Conv1d(128, 256, 1),
        )

        # Grasp pose encoder
        self.pose_encoder = nn.Sequential(
            nn.Linear(7, 64),  # position (3) + quaternion (4)
            nn.ReLU(),
            nn.Linear(64, 128),
        )

        # Quality predictor
        self.quality_head = nn.Sequential(
            nn.Linear(256 + 128, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
            nn.Sigmoid()  # Quality score [0, 1]
        )

    def forward(self, point_cloud, grasp_pose):
        # Encode point cloud
        pc_features = self.point_encoder(point_cloud.transpose(1, 2))
        pc_features = torch.max(pc_features, dim=2)[0]  # Max pooling

        # Encode grasp pose
        pose_features = self.pose_encoder(grasp_pose)

        # Combine and predict quality
        combined = torch.cat([pc_features, pose_features], dim=1)
        quality = self.quality_head(combined)

        return quality
\`\`\`

## Grasp Synthesis Pipeline

**Step 1: Object Segmentation**: Isolate target object from scene.

**Step 2: Grasp Candidate Generation**: Create many potential grasps.

**Step 3: Grasp Ranking**: Evaluate and rank candidates.

**Step 4: Collision Checking**: Verify gripper won't collide.

**Step 5: Reachability Check**: Ensure arm can reach grasp pose.

**Step 6: Execution**: Execute highest-ranked feasible grasp.

## ROS2 Grasp Planning

\`\`\`python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import numpy as np

class GraspPlanningNode(Node):
    def __init__(self):
        super().__init__('grasp_planner')

        # Subscribe to point cloud
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        # Publish grasp poses
        self.grasp_pub = self.create_publisher(
            PoseStamped,
            '/grasp_pose',
            10
        )

        self.grasp_planner = AnalyticalGraspPlanner()

    def pointcloud_callback(self, msg):
        # Convert ROS PointCloud2 to numpy
        points = self.pointcloud2_to_array(msg)

        # Segment object (simplified)
        object_points = self.segment_object(points)

        # Plan grasp
        grasp_pose = self.grasp_planner.plan_parallel_jaw_grasp(object_points)

        # Publish grasp
        grasp_msg = self.create_pose_msg(grasp_pose)
        self.grasp_pub.publish(grasp_msg)

        self.get_logger().info('Published grasp pose')
\`\`\`

## Advanced Grasp Planning Topics

**In-Hand Manipulation**: Reposition object within gripper.

**Multi-Object Grasping**: Grasp multiple objects simultaneously.

**Deformable Object Grasping**: Handle soft/flexible objects.

**Bin Picking**: Grasp from cluttered bins.

**Task-Oriented Grasping**: Choose grasps based on subsequent task.

## Gripper Control

**Position Control**: Specify finger positions.

**Force Control**: Specify gripping force.

**Impedance Control**: Combine position and force.

\`\`\`python
class GripperController:
    def __init__(self):
        self.max_force = 50.0  # Newtons
        self.max_width = 0.08  # meters

    def grasp(self, target_width=0.0, force=20.0):
        """Execute grasp with force control"""
        # Close gripper to target width
        self.move_to_width(target_width)

        # Apply force
        current_force = 0.0
        while current_force < force:
            current_force = self.get_gripper_force()
            if current_force > force * 0.9:
                break
            self.apply_force(force - current_force)

        return self.is_object_grasped()

    def release(self):
        """Open gripper to release object"""
        self.move_to_width(self.max_width)
\`\`\`

## Sim-to-Real Transfer

Test grasps in simulation before real execution.

**Physics Simulation**: Use realistic friction, compliance models.

**Domain Randomization**: Vary object properties during training.

**Real-World Validation**: Test on physical hardware.

Effective grasp planning combines mechanical design, geometric reasoning, and learning-based optimization to enable reliable manipulation.`,
                    readingTime: "23 min",
                    keywords: ["grippers", "grasp planning", "manipulation", "force closure", "grasp quality"]
                }
            ]
        },
        {
            id: "module-8",
            title: "Module 8: Advanced Topics",
            description: "Explore cutting-edge topics in robotics and AI",
            chapters: [
                {
                    id: "chapter-8-1",
                    title: "Chapter 1: Reinforcement Learning for Robotics",
                    content: `# Reinforcement Learning for Robotics

Reinforcement Learning (RL) enables robots to learn optimal behaviors through trial and error.

## RL Fundamentals

**Agent**: The robot learning to act.

**Environment**: The world the robot interacts with.

**State**: Current situation (sensor readings, joint angles).

**Action**: Robot's choice (motor commands).

**Reward**: Feedback signal indicating desirability of outcome.

**Policy**: Strategy mapping states to actions.

## Key Algorithms

**Q-Learning**: Learn value of state-action pairs.

**Deep Q-Networks (DQN)**: Use neural networks for Q-values.

**Policy Gradient**: Directly optimize policy.

**Actor-Critic**: Combine value and policy learning.

**PPO (Proximal Policy Optimization)**: Stable policy gradient method.

**SAC (Soft Actor-Critic)**: Maximum entropy RL for continuous control.

**TD3 (Twin Delayed DDPG)**: Stable continuous control.

## Example: Simple RL Training

\`\`\`python
import gym
import numpy as np
from stable_baselines3 import PPO

# Create environment
env = gym.make('RobotReacher-v0')

# Create agent
model = PPO('MlpPolicy', env, verbose=1)

# Train
model.learn(total_timesteps=100000)

# Test
obs = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
\`\`\`

## Sim-to-Real Transfer

**Domain Randomization**: Vary simulation parameters during training.

**System Identification**: Measure real robot parameters to match simulation.

**Fine-tuning**: Continue learning on real robot.

**Residual Learning**: Learn correction on top of simulation policy.

## Challenges

**Sample Efficiency**: Real robots can't train for millions of steps.

**Safety**: Exploration can damage robot or environment.

**Reward Engineering**: Designing good reward functions is difficult.

**Sim-to-Real Gap**: Simulation doesn't perfectly match reality.

## Applications

**Manipulation**: Learn grasping and in-hand manipulation.

**Locomotion**: Train walking and running gaits.

**Navigation**: Learn obstacle avoidance policies.

**Multi-Robot Coordination**: Learn collaborative behaviors.

RL is a powerful tool for learning complex robot behaviors that are difficult to program manually.`,
                    readingTime: "18 min",
                    keywords: ["reinforcement learning", "rl", "ppo", "robot learning"]
                },
                {
                    id: "chapter-8-2",
                    title: "Chapter 2: Multi-Robot Systems",
                    content: `# Multi-Robot Systems

Multi-robot systems coordinate multiple robots to achieve tasks beyond single-robot capabilities.

## Motivations

**Robustness**: System continues if one robot fails.

**Efficiency**: Parallelize tasks across multiple robots.

**Coverage**: Monitor or work in larger areas.

**Capabilities**: Combine different robot types for complex tasks.

## Coordination Strategies

**Centralized**: Single controller directs all robots.
- Pro: Optimal coordination
- Con: Single point of failure, communication bottleneck

**Decentralized**: Each robot makes independent decisions.
- Pro: Robust, scalable
- Con: Suboptimal, coordination challenges

**Distributed**: Robots coordinate through peer communication.
- Pro: Balance of efficiency and robustness
- Con: Communication complexity

## Communication

**Topics**: Shared information (positions, goals, discoveries).

**Broadcast**: All robots receive all messages.

**Targeted**: Messages sent to specific robots.

**Consensus**: Robots agree on shared values.

## Task Allocation

**Market-Based**: Robots bid on tasks.

**Priority-Based**: Assign by robot capabilities and availability.

**Learning-Based**: Learn task allocation policies.

## Example: Multi-Robot ROS2

\`\`\`python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MultiRobotCoordinator(Node):
    def __init__(self):
        super().__init__('coordinator')

        # Subscribe to all robot positions
        self.robot_positions = {}
        for robot_id in range(5):
            self.create_subscription(
                PoseStamped,
                f'/robot_{robot_id}/pose',
                lambda msg, id=robot_id: self.update_position(id, msg),
                10
            )

        # Publishers for robot goals
        self.goal_publishers = {}
        for robot_id in range(5):
            self.goal_publishers[robot_id] = self.create_publisher(
                PoseStamped,
                f'/robot_{robot_id}/goal',
                10
            )

    def update_position(self, robot_id, msg):
        self.robot_positions[robot_id] = msg
        self.coordinate_robots()

    def coordinate_robots(self):
        # Implement coordination logic
        if len(self.robot_positions) == 5:
            # All robots reporting, assign tasks
            self.assign_coverage_tasks()
\`\`\`

## Formation Control

Maintain geometric arrangements while moving.

**Leader-Follower**: Follow designated leader.

**Behavioral**: Combine attraction, repulsion, and goal-seeking.

**Virtual Structure**: Robots maintain positions in virtual formation.

## Swarm Robotics

Many simple robots exhibit complex collective behavior.

**Principles**:
- Local sensing and communication
- Simple individual rules
- Emergent global behavior
- Robustness through redundancy

**Applications**:
- Environmental monitoring
- Search and rescue
- Agriculture
- Construction

## Multi-Robot SLAM

Build shared maps using multiple robots.

**Challenges**:
- Data association (which observations match)
- Loop closure (recognizing previously visited places)
- Merging local maps

**Approaches**:
- Centralized fusion
- Distributed consensus
- Relative localization

Multi-robot systems enable capabilities beyond single robots but introduce coordination and communication challenges.`,
                    readingTime: "20 min",
                    keywords: ["multi-robot", "swarm robotics", "coordination", "formation control"]
                },
                {
                    id: "chapter-8-3",
                    title: "Chapter 3: Human-Robot Collaboration",
                    content: `# Human-Robot Collaboration

Human-robot collaboration (HRC) enables robots to work safely and effectively alongside humans.

## Collaboration Levels

**Coexistence**: Humans and robots in same space, no interaction.

**Cooperation**: Share workspace but separate tasks.

**Collaboration**: Work together on same task with coordination.

**Responsive Collaboration**: Robot adapts to human behavior in real-time.

## Safety Considerations

**Physical Safety**: Prevent collisions and injuries.

**Cognitive Safety**: Prevent confusion, surprise, or stress.

**Predictability**: Robot behavior should be understandable.

**Compliance**: Soft/compliant motion when near humans.

## Safety Standards

**ISO 10218**: Industrial robot safety.

**ISO/TS 15066**: Collaborative robot requirements.

**ISO 13482**: Personal care robot safety.

## Safety Mechanisms

**Force Limiting**: Reduce speed or stop when contact detected.

**Workspace Monitoring**: Track human positions with sensors.

**Hand Guiding**: Allow human to physically guide robot.

**Speed and Separation Monitoring**: Maintain safe distances.

## Intent Recognition

Understanding human goals and actions.

**Approaches**:
- Gesture recognition
- Gaze tracking
- Motion prediction
- Natural language understanding
- Brain-computer interfaces

\`\`\`python
class IntentRecognizer(Node):
    def __init__(self):
        super().__init__('intent_recognizer')

        # Subscribe to human pose
        self.create_subscription(
            PoseArray, '/human_tracker/poses',
            self.pose_callback, 10
        )

        # Subscribe to voice commands
        self.create_subscription(
            String, '/speech/recognized',
            self.speech_callback, 10
        )

    def pose_callback(self, msg):
        # Analyze gestures
        if self.detect_pointing_gesture(msg):
            target = self.get_pointing_target(msg)
            self.publish_intent('point_to', target)

    def speech_callback(self, msg):
        # Parse natural language
        intent = self.parse_command(msg.data)
        self.publish_intent(intent.type, intent.params)
\`\`\`

## Shared Control

Human and robot share control authority.

**Modes**:
- **Teleoperation**: Human full control
- **Supervisory**: Human high-level, robot low-level
- **Trading**: Control switches between human and robot
- **Shared**: Continuous blending of commands

## Communication Modalities

**Robot → Human**:
- Visual displays/LEDs
- Speech synthesis
- Gestures and movements
- Haptic feedback

**Human → Robot**:
- Voice commands
- Gestures
- Touch/buttons
- Brain signals

## Trust Calibration

Ensuring appropriate human trust in robot capabilities.

**Overtrust**: Human relies on robot beyond capabilities (dangerous).

**Undertrust**: Human doesn't utilize capable robot (inefficient).

**Calibration Strategies**:
- Transparent limitations
- Consistent performance
- Explanation of decisions
- Graceful failure handling

## Application Examples

**Manufacturing**: Collaborative assembly, material handling.

**Healthcare**: Surgical assistance, rehabilitation, elder care.

**Service**: Delivery, cleaning, guidance.

**Education**: Tutoring, teaching assistance.

**Exploration**: Search and rescue, space exploration.

Effective human-robot collaboration requires technical capabilities, safety mechanisms, and consideration of human factors.`,
                    readingTime: "22 min",
                    keywords: ["human robot interaction", "collaboration", "safety", "cobots"]
                }
            ]
        }
    ]
};

module.exports = bookContent;
