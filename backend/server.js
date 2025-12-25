const express = require('express');
const cors = require('cors');
const dotenv =require('dotenv');
const Anthropic = require('@anthropic-ai/sdk');
const { GoogleGenerativeAI } = require('@google/generative-ai');
const OpenAI = require('openai');
const fs = require('fs');
const path = require('path');

// Redirect console output to a log file
const logStream = fs.createWriteStream(path.join(__dirname, 'translation.log'), { flags: 'a' });
const logToFile = (message) => {
  logStream.write(`${new Date().toISOString()} - ${message}\n`);
  process.stdout.write(`${new Date().toISOString()} - ${message}\n`);
};
console.log = (message, ...optionalParams) => {
    logToFile(message);
};
console.error = (message, ...optionalParams) => {
    logToFile(`ERROR: ${message}`);
};

// Load environment variables
dotenv.config();

// Initialize Express app
const app = express();

// API key setup
const CLAUDE_API_KEY = process.env.CLAUDE_API_KEY;
const GEMINI_API_KEY = process.env.GEMINI_API_KEY;
const OPENAI_API_KEY = process.env.OPENAI_API_KEY || process.env.openai_api_key || process.env.openai_api; // Support different naming conventions

if (!CLAUDE_API_KEY) {
    console.log("CLAUDE_API_KEY is not set in the .env file. Using fallback responses.");
}

if (!GEMINI_API_KEY) {
    console.log("GEMINI_API_KEY is not set in the .env file. Using fallback responses.");
}

if (!OPENAI_API_KEY) {
    console.log("OPENAI_API_KEY is not set in the .env file. Using fallback responses.");
}

const anthropic = CLAUDE_API_KEY ? new Anthropic({ apiKey: CLAUDE_API_KEY }) : null;
const gemini = GEMINI_API_KEY ? new GoogleGenerativeAI(GEMINI_API_KEY) : null;
const openai = OPENAI_API_KEY ? new OpenAI({ apiKey: OPENAI_API_KEY }) : null;

// let ragPipeline = null;
// if (GEMINI_API_KEY) {
//     try {
//         ragPipeline = require('./rag_pipeline');
//         ragPipeline.initializeRagPipeline()
//             .then(() => {
//                 console.log("RAG pipeline initialized successfully");
//             })
//             .catch(error => {
//                 console.error("Failed to initialize RAG pipeline:", error);
//             });
//     } catch (error) {
//         console.log("RAG Pipeline not available:
// ", error.message);
//     }
// }

// Import local RAG system (doesn't require external APIs)
// try {
//     const localRAG = require('./local_rag');
//     global.localRAG = localRAG; // Make available globally to handle any scope issues
// } catch (error) {
//     console.log("Local RAG system not available:", error.message);
//     // Create a mock localRAG to prevent failures
//     global.localRAG = {
//         initializeLocalRAG: async () => { console.log("Local RAG not available, using fallback"); return Promise.resolve(); },
//         retrieveRelevantDocuments: (query, k) => { console.log("Local RAG not available"); return []; }
//     };
// }

// Middleware
app.use(express.json());
app.use(cors({
  origin: function (origin, callback) {
    // Allow requests with no origin (like mobile apps or curl requests)
    if (!origin) return callback(null, true);

    // Allow all localhost origins
    if (origin && origin.startsWith('http://localhost')) {
      return callback(null, true);
    }

    const allowedOrigins = [
      'http://localhost:3000',
      'https://physical-ai-robotics-chi.vercel.app',
      'https://physical-ai-robotics-minahil-hamzas-projects.vercel.app',
      'https://physical-ai-robotics-lmpl2ni7e-minahil-hamzas-projects.vercel.app',
      process.env.FRONTEND_URL
    ].filter(Boolean);

    // Allow all Vercel frontend deployments
    if (origin.includes('frontend') && origin.includes('minahil-hamzas-projects.vercel.app')) {
      return callback(null, true);
    }

    // Allow all book-hackathone frontend deployments
    if (origin.includes('book-hackathone') && origin.includes('minahil-hamzas-projects.vercel.app')) {
      return callback(null, true);
    }

    if (allowedOrigins.indexOf(origin) !== -1) {
      callback(null, true);
    } else {
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true
}));

console.log('Server initialized successfully without database');

// Routes
app.use('/api/auth', require('./routes/auth'));
app.use('/api/users', require('./routes/users'));

// Enhanced fallback knowledge base for when API is unavailable
const knowledgeBase = {
    'physical ai': 'Physical AI refers to artificial intelligence systems that interact with and operate in the physical world through robotics. It combines AI algorithms, computer vision, sensor processing, and robotic control to enable machines to perceive, understand, and manipulate their environment. Physical AI is crucial for autonomous robots, humanoids, industrial automation, and embodied AI systems. It represents the integration of AI with the physical world, allowing machines to learn from interaction with real environments.',
    'robotics': 'Robotics is the field of engineering and computer science focused on designing, building, and programming robots. It combines mechanical engineering, electrical engineering, and computer science to create machines that can perform tasks autonomously or semi-autonomously. Modern robotics incorporates AI, sensors, actuators, control systems, and often connects to cloud services for enhanced capabilities.',
    'ros2': 'ROS2 (Robot Operating System 2) is the next generation of ROS, a flexible framework for writing robot software. It provides services including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. ROS2 offers improved security, real-time support, and better scalability compared to the original ROS.',
    'digital twin': 'A Digital Twin is a virtual representation of a physical robot that mirrors its behavior, performance, and environment. It allows for testing, simulation, and optimization before deployment on actual hardware. Digital twins enable engineers to validate robot behaviors, test scenarios safely, and predict maintenance needs.',
    'gazebo': 'Gazebo is a powerful 3D robotics simulator that provides realistic physics simulation, sensor simulation, and supports ROS integration. It\'s commonly used for testing robots in various environments before physical deployment. Gazebo allows testing of navigation, perception, and control algorithms without hardware risks.',
    'isaac sim': 'NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse, providing photorealistic rendering, accurate physics simulation, and support for AI model training. It\'s particularly powerful for developing AI-powered robots. It integrates with Isaac ROS for seamless development workflows.',
    'nav2': 'Nav2 (Navigation2) is the ROS2 navigation stack that provides autonomous navigation capabilities for mobile robots. It includes path planning, obstacle avoidance, localization, and mapping features. Nav2 enables robots to navigate in dynamic environments safely.',
    'urdf': 'URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot\'s physical structure, joints, links, sensors, and visual/collision properties. URDF files specify how robot parts connect and move.',
    'vla': 'Vision-Language-Action (VLA) models combine computer vision, natural language processing, and robotic control to enable robots to understand visual scenes, interpret language commands, and execute appropriate actions. VLA models represent a major advance in embodied AI.',
    'isaac ros': 'Isaac ROS provides GPU-accelerated packages for ROS2, offering high-performance perception and AI capabilities for robotics applications using NVIDIA hardware. It bridges the gap between GPU computing and robotic systems.',
    'simulation': 'Robot simulation allows testing and developing robotic systems in virtual environments before deploying to real hardware. Common simulators include Gazebo, Isaac Sim, and Unity. Simulation helps reduce costs, speeds up development, and enables testing dangerous scenarios safely.',
    'ai robot brain': 'The AI Robot Brain refers to the cognitive and decision-making capabilities of a robot, combining perception (sensors, vision), processing (AI models, planning algorithms), and action (motor control, manipulation). It enables robots to understand their environment, make decisions, and execute tasks autonomously.',
    'humanoid robotics': 'Humanoid robots are robots with human-like form and capabilities. They typically have a head, torso, two arms, and two legs. Humanoid robots aim to interact naturally with human environments and can be used for research, assistance, and social interaction.',
    'embodied ai': 'Embodied AI refers to artificial intelligence that interacts with the physical world through a body (robot). Unlike traditional AI that processes data, embodied AI learns through interaction with its environment. This leads to more grounded and practical intelligence.',
    'computer vision': 'Computer vision in robotics enables robots to interpret and understand visual information from cameras and other imaging sensors. It\'s crucial for navigation, object recognition, manipulation, and environmental understanding.',
    'machine learning': 'Machine learning in robotics involves training algorithms to recognize patterns, make predictions, and improve performance through experience. Common techniques include deep learning, reinforcement learning, and supervised learning.',
    'motion planning': 'Motion planning involves determining a sequence of movements to accomplish a task while avoiding obstacles. It\'s crucial for robot navigation and manipulation tasks.',
    'sensor fusion': 'Sensor fusion combines data from multiple sensors to create more accurate and reliable perception of the environment. This includes cameras, lidar, IMUs, and other sensing modalities.',
    'control theory': 'Control theory in robotics involves mathematical models to govern robot behavior, ensuring stability, accuracy, and response characteristics. It\'s fundamental for achieving desired robot motions.',
    'kinematics': 'Kinematics is the study of motion without considering forces. Forward kinematics calculates end-effector position from joint angles, while inverse kinematics determines joint angles to achieve desired positions.',
    'dynamics': 'Robot dynamics studies the relationship between forces acting on a robot mechanism and the resulting motion. It considers mass, inertia, friction, and other physical properties.',
    'manipulation': 'Robotic manipulation involves controlling robot arms and grippers to interact with objects. This includes grasping, lifting, moving, and assembling objects.',
    'locomotion': 'Locomotion in robotics refers to the methods by which robots move, including walking, rolling, crawling, or flying depending on the robot design.',
    'path planning': 'Path planning algorithms determine optimal routes for robots to travel while avoiding obstacles and respecting environmental constraints.',
    'slam': 'SLAM (Simultaneous Localization and Mapping) allows robots to build maps of unknown environments while simultaneously tracking their location within those maps.',
    'reinforcement learning': 'Reinforcement learning in robotics uses trial-and-error approaches where robots learn optimal behaviors through rewards and penalties.',
    'deep learning': 'Deep learning in robotics applies neural networks with multiple layers to enable robots to learn complex behaviors, recognize objects, and make decisions.',
    'autonomous': 'Autonomous robots can operate independently without continuous human intervention. They perceive their environment, make decisions, and execute actions towards goals.',
    'human robot interaction': 'Human-robot interaction focuses on designing robots that can communicate and collaborate effectively with humans in shared environments.',
    'robot operating system': 'Robot Operating System (ROS) provides libraries and tools to help software developers create robot applications. It includes hardware abstraction, device drivers, libraries, visualizers, and more.',
    'point cloud': 'A point cloud is a collection of data points in 3D space, typically generated by 3D scanners or depth sensors, used for mapping and object recognition.',
    'lidar': 'LiDAR (Light Detection and Ranging) sensors create precise distance measurements using laser light, commonly used for mapping and navigation in robotics.',
    'imu': 'An IMU (Inertial Measurement Unit) measures acceleration, rotation rates, and sometimes magnetic fields to estimate orientation and motion.',
    'actuator': 'An actuator is a component that moves or controls a mechanism or system, converting energy into physical motion for robot joints or other mechanisms.',
    'end effector': 'An end effector is the device at the end of a robot arm designed to interact with the environment, such as grippers, tools, or sensors.',
    'forward kinematics': 'Forward kinematics calculates the position and orientation of the robot\'s end effector based on known joint angles.',
    'inverse kinematics': 'Inverse kinematics determines the joint angles required to achieve a desired position and orientation of the robot\'s end effector.',
    'workspace': 'A robot\'s workspace is the total volume of space that a robot\'s end-effector can reach.',
    'degrees of freedom': 'Degrees of freedom refer to the number of independent movements a robot can make, corresponding to the number of joints or joint types.',
    'ros package': 'A ROS package is the basic unit of organizing software in ROS, containing nodes, libraries, data sets, configuration files, and other resources.',
    'ros node': 'A ROS node is a process that performs computation in the ROS system, and multiple nodes communicate with each other using topics, services, and actions.',
    'ros topic': 'A ROS topic is a named bus over which nodes exchange messages in a publish/subscribe communication pattern.',
    'ros service': 'A ROS service provides a request/reply communication pattern between nodes.',
    'gripper': 'A gripper is a device that grasps and manipulates objects, usually located at the end of a robot arm.',
    'navigation': 'Robot navigation is the process of determining and following a path from a source to a destination in an environment.',
    'mapping': 'Mapping in robotics is the creation of representations of the environment, typically used for navigation and spatial understanding.',
    'localization': 'Localization is the process by which a robot determines its position and orientation in a known or unknown environment.',
    'obstacle avoidance': 'Obstacle avoidance enables robots to detect and navigate around obstacles in their path while reaching their destination.',
    'trajectory': 'A trajectory is a time-ordered sequence of position, velocity, and acceleration commands that guide robot motion.',
    'control loop': 'A control loop is a continuous cycle of sensing, processing, and actuating to maintain desired robot behavior or achieve goals.',
    'feedback control': 'Feedback control adjusts robot actions based on sensor measurements to reduce errors and achieve desired behaviors.',
    'proportional integral derivative controller': 'PID controllers are feedback controllers that adjust robot behavior based on proportional, integral, and derivative terms to minimize error.',
    'torque': 'Torque is rotational force applied to robot joints to produce motion.',
    'force control': 'Force control regulates the forces applied by robots during interaction with the environment or objects.',
    'impedance control': 'Impedance control regulates robot behavior by defining dynamic relationships between forces and motion during contact tasks.',
    'ros launch': 'ROS launch files allow starting multiple nodes with configuration parameters at once.',
    'tf transform': 'TF (Transform) in ROS manages coordinate frame transformations over time.',
    'actionlib': 'Actionlib provides tools for managing long-duration tasks in ROS with feedback and status updates.',
    'moveit': 'MoveIt is a ROS-based motion planning framework for robot manipulation and movement.',
    'rviz': 'RViz is the 3D visualization tool for ROS that displays robot models, sensor data, and other information.',
    'gazebo simulation': 'Gazebo simulation provides realistic physics and sensor simulation for testing robots before deployment.',
    'physics engine': 'A physics engine computes physical interactions in simulation to mimic real-world behavior.',
    'collision detection': 'Collision detection algorithms identify when robot components or the environment intersect.',
    'articulated robot': 'An articulated robot has multiple rigid segments connected by joints allowing relative motion.',
    'serial robot': 'A serial robot has links connected sequentially from base to end effector.',
    'parallel robot': 'A parallel robot has multiple kinematic chains connecting the base to the end effector.',
    'mobile robot': 'A mobile robot can move freely in its environment, unlike stationary manipulator robots.',
    'wheeled robot': 'A wheeled robot uses wheels for locomotion, common for ground-based mobile robots.',
    'legged robot': 'A legged robot uses legs for locomotion, mimicking biological movement patterns.',
    'walking robot': 'A walking robot uses legged locomotion with periodic foot contact with the ground.',
    'bipedal': 'Bipedal robots walk on two legs, presenting significant balance challenges.',
    'quadrupedal': 'Quadrupedal robots walk on four legs, offering stability and mobility.',
    'hexapod': 'A hexapod robot has six legs, providing exceptional stability and terrain adaptability.',
    'crawler': 'A crawler robot uses tracks or treads for locomotion instead of wheels or legs.',
    'aerial robot': 'An aerial robot flies, including drones, helicopters, and fixed-wing aircraft robots.',
    'underwater robot': 'An underwater robot operates beneath water surfaces for exploration, inspection, or maintenance.',
    'swarm robotics': 'Swarm robotics involves coordinating multiple robots to accomplish tasks collectively.',
    'cooperative robotics': 'Cooperative robotics involves multiple robots working together to achieve common goals.',
    'teleoperation': 'Teleoperation involves remote control of robots by human operators.',
    'telepresence': 'Telepresence provides remote presence through robots, allowing distant interaction.',
    'haptic feedback': 'Haptic feedback provides tactile sensations to human operators of remote robots.',
    'calibration': 'Calibration adjusts robot sensors and actuators to minimize measurement and control errors.',
    'kinect': 'Microsoft Kinect is a sensor platform for capturing 3D depth information and gesture recognition.',
    'open cv': 'OpenCV is an open-source computer vision library widely used in robotics applications.',
    'pcl': 'Point Cloud Library (PCL) is a standalone project for 2D/3D image and point cloud processing.',
    'aruco': 'ArUco markers are synthetic fiducial markers used for camera pose estimation in robotics.',
    'apriltag': 'AprilTag markers are visual fiducials that can be used for robot localization and navigation.',
    'visual servoing': 'Visual servoing uses visual feedback to control robot motion and manipulation.',
    'structure from motion': 'Structure from Motion (SfM) reconstructs 3D structures from 2D image sequences.',
    'stereo vision': 'Stereo vision uses two cameras to determine depth information similar to human binocular vision.',
    'monocular vision': 'Monocular vision uses a single camera, often inferring depth through motion or known object sizes.',
    'optical flow': 'Optical flow analyzes motion patterns in image sequences to estimate motion direction and speed.',
    'feature extraction': 'Feature extraction identifies distinctive elements in images useful for recognition and matching.',
    'object recognition': 'Object recognition identifies and classifies objects in images or point clouds.',
    'semantic segmentation': 'Semantic segmentation labels each pixel in an image with a semantic category.',
    'instance segmentation': 'Instance segmentation distinguishes individual object instances in addition to semantic categories.',
    'tracking': 'Tracking follows objects or features through sequential frames to monitor movement.',
    'sliding mode control': 'Sliding mode control is a nonlinear control method that forces the system to slide along a surface.',
    'adaptive control': 'Adaptive control adjusts control parameters in real-time based on changing system conditions.',
    'model predictive control': 'Model predictive control optimizes control actions based on predictions of future behavior.',
    'linear quadratic regulator': 'Linear Quadratic Regulator (LQR) optimizes control for linear systems with quadratic costs.',
    'kalman filter': 'Kalman filters estimate system states from noisy measurements optimally.',
    'particle filter': 'Particle filters use Monte Carlo methods for state estimation in nonlinear systems.',
    'extended kalman filter': 'Extended Kalman Filter (EKF) handles nonlinear systems by linearizing around estimates.',
    'unscented kalman filter': 'Unscented Kalman Filter (UKF) uses deterministic sampling to estimate nonlinear transformations.',
    'monte carlo localization': 'Monte Carlo localization estimates robot pose using particle filters.',
    'occupancy grid': 'Occupancy grids represent environments as 2D probability grids indicating occupied/free space.',
    'topological map': 'Topological maps represent environments as nodes and connections rather than geometric details.',
    'metric map': 'Metric maps preserve geometric accuracy of environmental features and distances.',
    'costmap': 'Costmaps assign costs to areas based on obstacles, traversability, and other factors.',
    'voxel grid': 'Voxel grids extend occupancy grids to 3D space using volumetric pixels.',
    'probabilistic robotics': 'Probabilistic robotics deals with uncertainty in sensing and actuation using probability theory.',
    'bayesian filtering': 'Bayesian filtering updates probability distributions over states using sensor measurements.',
    'monte carlo methods': 'Monte Carlo methods use random sampling to solve numerical problems in robotics.',
    'particle swarm optimization': 'Particle Swarm Optimization (PSO) is a population-based optimization technique.',
    'genetic algorithm': 'Genetic algorithms optimize solutions through evolutionary processes mimicking natural selection.',
    'neural networks': 'Neural networks are machine learning models inspired by biological neural systems.',
    'convolutional neural network': 'Convolutional Neural Networks (CNNs) excel at processing spatial data like images.',
    'recurrent neural network': 'Recurrent Neural Networks (RNNs) handle sequential data by maintaining internal state.',
    'long short-term memory': 'Long Short-Term Memory (LSTM) networks handle long-term dependencies in sequences.',
    'generative adversarial network': 'GANs train generative and discriminative models competitively.',
    'transformer': 'Transformer architectures process sequences using attention mechanisms.',
    'attention mechanism': 'Attention mechanisms focus neural network processing on relevant input parts.',
    'reinforcement learning algorithm': 'Reinforcement learning algorithms learn via reward/punishment signals.',
    'q learning': 'Q-learning is a model-free reinforcement learning algorithm for discrete action spaces.',
    'deep q network': 'Deep Q-Networks (DQN) combine Q-learning with deep neural networks.',
    'policy gradient': 'Policy gradient methods optimize policies directly rather than value functions.',
    'actor critic': 'Actor-critic methods combine policy and value learning approaches.',
    'imitation learning': 'Imitation learning teaches robots by demonstrating desired behaviors.',
    'behavior cloning': 'Behavior cloning learns policies by mimicking expert demonstrations.',
    'inverse reinforcement learning': 'Inverse reinforcement learning infers rewards from observed expert behavior.',
    'safe exploration': 'Safe exploration methods allow robots to learn while avoiding dangerous situations.',
    'domain randomization': 'Domain randomization varies simulation parameters to improve transfer to reality.',
    'sim to real': 'Sim-to-real transfer addresses differences between simulation and real-world performance.',
    'embodiment': 'Embodiment refers to how a robot\'s physical form influences its behavior and intelligence.',
    'morphological computation': 'Morphological computation leverages physical body properties for computational advantages.',
    'affordance': 'Affordances represent possibilities for action that objects or environments offer.',
    'grounded cognition': 'Grounded cognition grounds understanding in sensorimotor experiences.',
    'situated cognition': 'Situated cognition emphasizes that thinking occurs in the context of environment.',
    'ecological psychology': 'Ecological psychology studies organism-environment interactions for robot design.',
    'active perception': 'Active perception involves selecting sensing actions to maximize information gain.',
    'multimodal perception': 'Multimodal perception combines information from multiple sensing modalities.',
    'sensorimotor loop': 'The sensorimotor loop connects sensing, decision-making, and action in robotics.',
    'affordance learning': 'Affordance learning discovers action possibilities through interaction experience.',
    'tool use': 'Tool use involves manipulating external objects to extend capabilities.',
    'cognitive architecture': 'Cognitive architectures structure robot intelligence and decision-making.',
    'working memory': 'Working memory temporarily stores and processes information for decision-making.',
    'executive function': 'Executive functions manage and coordinate cognitive processes.',
    'planning': 'Planning generates sequences of actions to achieve goals.',
    'deliberative reasoning': 'Deliberative reasoning uses logical inference for decision-making.',
    'reactive behavior': 'Reactive behaviors respond immediately to environmental stimuli.',
    'hybrid architecture': 'Hybrid architectures combine deliberative and reactive components.',
    'subsumption architecture': 'Subsumption architectures layer behaviors with higher levels inhibiting lower ones.',
    'behavior tree': 'Behavior trees organize robot behaviors in hierarchical tree structures.',
    'finite state machine': 'Finite state machines model robot behavior as discrete states and transitions.',
    'petri net': 'Petri nets model concurrent and asynchronous robot behaviors.',
    'temporal logic': 'Temporal logic specifies timing constraints for robot behaviors.',
    'linear temporal logic': 'Linear Temporal Logic (LTL) specifies sequences of conditions over time.',
    'computational geometry': 'Computational geometry algorithms solve geometric problems in robotics.',
    'voronoi diagram': 'Voronoi diagrams partition space around obstacles for path planning.',
    'visibility graph': 'Visibility graphs connect visible locations for shortest path planning.',
    'rapidly exploring random tree': 'RRT algorithms probabilistically explore configuration space for path planning.',
    'probabilistic roadmap': 'PRMs precompute connectivity in configuration space for path planning.',
    'cell decomposition': 'Cell decomposition divides configuration space into simpler regions.',
    'configuration space': 'Configuration space represents all possible robot configurations.',
    'free space': 'Free space in configuration space avoids collisions with obstacles.',
    'collision space': 'Collision space in configuration space represents configurations causing collisions.',
    'workspace': 'Workspace is the space where the robot operates.',
    'reachable space': 'Reachable space contains all positions accessible to robot end effectors.',
    'workspace analysis': 'Workspace analysis determines reachable and dexterous regions.',
    'manipulability': 'Manipulability measures how easily a robot can move in different directions.',
    'singularity': 'Singularities are configurations where robot loses degrees of freedom.',
    'redundancy': 'Redundant robots have more degrees of freedom than required for tasks.',
    'dexterity': 'Dexterity measures robot capability to perform complex manipulation tasks.',
    'maneuverability': 'Maneuverability measures how easily a robot can change its configuration.',
    'stiffness': 'Stiffness relates force applied to resulting elastic displacement.',
    'compliance': 'Compliance allows controlled flexibility in robot motion.',
    'flexibility': 'Flexibility measures robot ability to bend or flex under force.',
    'elasticity': 'Elasticity describes reversible deformation under stress.',
    'viscosity': 'Viscosity characterizes resistance to flow or motion.',
    'friction': 'Friction opposes relative motion between contacting surfaces.',
    'static friction': 'Static friction prevents motion initiation between contacting surfaces.',
    'dynamic friction': 'Dynamic friction opposes motion between sliding surfaces.',
    'rolling resistance': 'Rolling resistance opposes rolling motion of wheels or spheres.',
    'traction': 'Traction is the frictional force enabling wheel-ground adhesion.',
    'skidding': 'Skidding occurs when wheels slide laterally due to insufficient traction.',
    'slipping': 'Slipping occurs when wheels rotate without corresponding linear motion.',
    'grip': 'Grip describes adhesive contact between surfaces enabling motion control.',
    'wear': 'Wear is gradual removal of material from surfaces due to rubbing.',
    'fatigue': 'Fatigue results in material failure due to repeated loading cycles.',
    'creep': 'Creep is slow deformation under constant load over time.',
    'resonance': 'Resonance amplifies oscillations when forcing frequency matches natural frequency.',
    'bandwidth': 'Bandwidth measures the range of frequencies a system can process effectively.',
    'latency': 'Latency is delay between stimulus and response in control systems.',
    'throughput': 'Throughput measures the rate of data or command processing.',
    'real time': 'Real-time systems guarantee response within strict timing constraints.',
    'hard real time': 'Hard real-time systems must meet timing constraints without exception.',
    'soft real time': 'Soft real-time systems aim to meet timing constraints but may occasionally miss them.',
    'jitter': 'Jitter is variation in timing of periodic events or measurements.',
    'synchronization': 'Synchronization aligns timing of distributed processes or events.',
    'multiprocessing': 'Multiprocessing uses multiple processors to increase computation capacity.',
    'multithreading': 'Multithreading executes multiple threads concurrently on single processor.',
    'parallel processing': 'Parallel processing performs computations simultaneously across multiple units.',
    'distributed control': 'Distributed control spreads decision-making across multiple system components.',
    'centralized control': 'Centralized control makes decisions at single central location.',
    'hierarchical control': 'Hierarchical control organizes decision-making in multiple levels.',
    'decentralized control': 'Decentralized control distributes equal authority across system parts.',
    'network robot': 'Network robots communicate and coordinate through network connections.',
    'internet of things': 'IoT connects robots to internet infrastructure for enhanced capabilities.',
    'cloud robotics': 'Cloud robotics uses cloud computing for robot data processing and storage.',
    'edge computing': 'Edge computing processes data near data sources to reduce latency.',
    'cyber physical system': 'Cyber-physical systems integrate computation with physical processes.',
    'human in the loop': 'Human-in-the-loop systems include humans in robot decision processes.',
    'supervisory control': 'Supervisory control allows humans to oversee and intervene in robot operations.',
    'shared autonomy': 'Shared autonomy balances human and autonomous robot control.',
    'variable autonomy': 'Variable autonomy adjusts autonomy level based on circumstances.',
    'mixed initiative': 'Mixed-initiative systems share decision-making between humans and robots.',
    'trust calibration': 'Trust calibration ensures appropriate human trust in robot capabilities.',
    'explainable ai': 'XAI provides explanations for robot decision-making to humans.',
    'transparency': 'Transparency makes robot decision processes visible to humans.',
    'interpretability': 'Interpretability allows humans to understand robot behavior reasons.',
    'robustness': 'Robustness indicates system performance despite disturbances or uncertainties.',
    'reliability': 'Reliability is the probability that a robot functions correctly over time.',
    'fault tolerance': 'Fault tolerance allows continued operation despite component failures.',
    'safety': 'Safety prevents harm to humans, property, or environment during robot operation.',
    'risk assessment': 'Risk assessment identifies and evaluates potential hazards from robots.',
    'fail safe': 'Fail-safe design ensures safe system state upon component failure.',
    'graceful degradation': 'Graceful degradation maintains reduced functionality during problems.',
    'redundant systems': 'Redundant systems provide backup components for increased reliability.',
    'diagnostics': 'Diagnostics identify system health and potential failures.',
    'prognostics': 'Prognostics predict future system failures or maintenance needs.',
    'maintenance': 'Maintenance preserves robot functionality and extends operational life.',
    'validation': 'Validation confirms robot systems meet specified requirements.',
    'verification': 'Verification proves robot systems satisfy design specifications.',
    'certification': 'Certification provides official compliance confirmation for safety standards.',
    'ethics': 'Robot ethics addresses moral implications of robot design and deployment.',
    'privacy': 'Privacy protects personal information gathered by robots from unauthorized access.',
    'security': 'Security protects robots from unauthorized access or malicious control.',
    'authentication': 'Authentication verifies identity of robot users or controllers.',
    'authorization': 'Authorization determines permitted robot access levels for entities.',
    'encryption': 'Encryption protects robot communications and data confidentiality.',
    'firewall': 'Firewalls protect robot networks from unauthorized external access.',
    'penetration testing': 'Penetration testing evaluates robot security through simulated attacks.',
    'secure boot': 'Secure boot ensures robot starts with trusted software only.',
    'over air update': 'Over-the-air updates remotely deploy software updates to robots.',
    'rollback capability': 'Rollback capability restores previous software versions after failed updates.',
    'access control': 'Access control restricts robot and data access to authorized users.',
    'audit trail': 'Audit trails record robot activities for security and compliance review.',
    'data governance': 'Data governance manages robot-collected data lifecycle and usage rights.',
    'bias': 'Bias in robots leads to unfair treatment or discrimination in automated decisions.',
    'fairness': 'Fairness ensures equitable robot behavior across demographic groups.',
    'accountability': 'Accountability assigns responsibility for robot decisions and actions.',
    'responsibility': 'Responsibility defines duty for robot consequences and outcomes.',
    'liability': 'Liability assigns legal responsibility for robot-caused damages.',
    'transparency': 'Transparency reveals robot capabilities and limitations to users.',
    'accuracy': 'Accuracy measures how closely robot outputs match true values.',
    'precision': 'Precision measures robot output consistency across repeated trials.',
    'recall': 'Recall measures the fraction of true positives correctly identified.',
    'f1 score': 'F1 score balances precision and recall in performance evaluation.',
    'confusion matrix': 'Confusion matrices evaluate classification performance systematically.',
    'mean squared error': 'Mean squared error measures average squared prediction errors.',
    'root mean squared error': 'RMSE measures average prediction error magnitude in original units.',
    'mean absolute error': 'MAE measures average absolute prediction errors.',
    'mean absolute percentage error': 'MAPE measures prediction errors as percentages of actual values.',
    'r squared': 'R-squared indicates proportion of variance explained by regression models.',
    'cross validation': 'Cross-validation estimates model performance on unseen data.',
    'training': 'Training develops robot capabilities through experience or instruction.',
    'testing': 'Testing evaluates robot performance on held-out datasets.',
    'validation': 'Validation tunes model parameters during development.',
    'overfitting': 'Overfitting occurs when models capture noise instead of underlying patterns.',
    'underfitting': 'Underfitting occurs when models fail to capture important patterns.',
    'generalization': 'Generalization measures performance on previously unseen data.',
    'sample complexity': 'Sample complexity measures data needed for good generalization.',
    'convergence': 'Convergence occurs when learning algorithms reach optimal solutions.',
    'stability': 'Stability ensures small input changes produce small output changes.',
    'consistency': 'Consistency ensures identical inputs produce identical outputs.',
    'causality': 'Causality identifies cause-effect relationships in robot decision-making.',
    'correlation': 'Correlation measures statistical relationships between variables.',
    'variance': 'Variance measures output sensitivity to training data variations.',
    'bias': 'Bias measures systematic errors in learning algorithm approximations.',
    'tradeoff': 'Tradeoffs balance competing design objectives like bias and variance.',
    'regularization': 'Regularization prevents overfitting by penalizing model complexity.',
    'normalization': 'Normalization scales data to common ranges for stable training.',
    'standardization': 'Standardization transforms data to zero mean and unit variance.',
    'augmentation': 'Augmentation increases training data diversity synthetically.',
    'preprocessing': 'Preprocessing prepares raw data for robot learning and operation.',
    'feature scaling': 'Feature scaling normalizes input dimensions to similar ranges.',
    'dimensionality reduction': 'Dimensionality reduction decreases input feature numbers.',
    'principal component analysis': 'PCA finds orthogonal axes maximizing data variance.',
    'independent component analysis': 'ICA separates mixed signals into independent sources.',
    't distributed stochastic neighbor embedding': 't-SNE visualizes high-dimensional data in low dimensions.',
    'uniform manifold approximation projection': 'UMAP preserves both local and global data structure.',
    'clustering': 'Clustering groups similar data points together.',
    'k means': 'K-means partitions data into k clusters around centroids.',
    'hierarchical clustering': 'Hierarchical clustering builds tree-like cluster relationships.',
    'density based clustering': 'DBSCAN clusters points in dense regions separated by sparse areas.',
    'gaussian mixture model': 'GMM represents data as weighted sums of Gaussian distributions.',
    'expectation maximization': 'EM algorithm estimates parameters of statistical models.',
    'classification': 'Classification assigns samples to discrete categories.',
    'regression': 'Regression predicts continuous valued outputs.',
    'support vector machine': 'SVM finds optimal hyperplanes separating classes.',
    'decision tree': 'Decision trees classify by recursive attribute-based splits.',
    'random forest': 'Random forests average multiple decision trees to reduce overfitting.',
    'boosting': 'Boosting sequentially trains weak learners to reduce errors.',
    'bagging': 'Bagging trains multiple models on bootstrap samples for ensemble learning.',
    'ensemble': 'Ensemble methods combine multiple models for improved performance.',
};

function getResponse(query) {
    const lowerQuery = query.toLowerCase();

    // Enhanced search through knowledge base
    const foundEntries = [];
    for (const [keyword, response] of Object.entries(knowledgeBase)) {
        if (lowerQuery.includes(keyword)) {
            foundEntries.push(response);
        }
    }

    // If we found matches, combine them
    if (foundEntries.length > 0) {
        return foundEntries.join(' Additionally, ').substring(0, 2000) + (foundEntries.length > 1 ? '\n\nFor more detailed information, I recommend checking the specific sections in the Physical AI & Robotics textbook.' : '');
    }

    // Try to find partial matches (keywords that appear as substrings)
    for (const [keyword, response] of Object.entries(knowledgeBase)) {
        if (lowerQuery.includes(keyword.split(' ')[0])) { // Check first word of each keyword
            return response;
        }
    }

    // Try to find related concepts
    const synonyms = {
        'robot': ['robotic', 'robotics', 'robots'],
        'learn': ['learning', 'teach', 'education', 'study'],
        'ai': ['artificial intelligence', 'intelligence', 'intelligent'],
        'movement': ['motion', 'locomotion', 'move', 'moving'],
        'see': ['vision', 'image', 'camera', 'perception', 'sensing'],
        'think': ['intelligent', 'reason', 'reasoning', 'decision'],
        'control': ['controlling', 'controller', 'control system'],
        'path': ['navigate', 'navigation', 'route', 'way'],
        'avoid': ['obstacle', 'collision', 'safe', 'safety'],
        'arm': ['manipulation', 'hand', 'gripper', 'manipulator'],
        'walk': ['walking', 'leg', 'legged', 'bipedal', 'motion'],
        'talk': ['communication', 'language', 'voice', 'speech'],
        'understand': ['comprehension', 'cognition', 'cognitive'],
        'plan': ['planning', 'strategy', 'algorithm', 'method'],
        'action': ['behavior', 'task', 'perform', 'execute']
    };

    for (const [keyConcept, relatedTerms] of Object.entries(synonyms)) {
        for (const term of relatedTerms) {
            if (lowerQuery.includes(term)) {
                const response = knowledgeBase[keyConcept];
                if (response) {
                    return `Regarding ${keyConcept}: ${response}\n\nThis concept is covered in detail in the Physical AI & Robotics textbook.`;
                }
            }
        }
    }

    // Default response with more guidance
    return `I'm a chatbot assistant for the Physical AI & Robotics textbook. I can help answer questions about:\n\n• ROS2 (Robot Operating System 2)\n• Robotics Simulation (Gazebo, Unity, Isaac Sim)\n• Digital Twins\n• AI Robot Brain concepts\n• Vision-Language-Action (VLA) models\n• NVIDIA Isaac ROS\n• Navigation (Nav2)\n• URDF robot descriptions\n• Humanoid Robotics\n• Control Systems\n• Perception Systems\n• Machine Learning in Robotics\n\nYour question: "${query}"\n\nPlease try asking about specific topics like "What is ROS2?" or "Explain Digital Twin". If the AI API is unavailable, I use a comprehensive knowledge base with over 300 robotics concepts to assist you.`;
}



// Initialize local RAG system
// if (global.localRAG) {
//     global.localRAG.initializeLocalRAG()
//         .then(() => {
//             console.log("Local RAG system initialized successfully");
//         })
//         .catch(error => {
//             console.error("Failed to initialize local RAG system:", error);
//         });
// } else {
//     console.log("Local RAG system not available");
// }

// Function to translate text to Urdu
async function translateToUrdu(text, aiProvider = null) {
    console.log(`[translateToUrdu] Starting translation for text: "${text.substring(0, 50)}..."`);
    try {
        const translationPrompt = `Translate the following text to Urdu (Pakistan). Only provide the translation and nothing else:\n\n${text}`;

        if (aiProvider === 'claude' && anthropic) {
            try {
                console.log('[translateToUrdu] Attempting translation with Claude...');
                const message = await anthropic.messages.create({
                    model: "claude-3-5-sonnet-20241022",
                    max_tokens: 1024,
                    system: "You are a professional translator. Translate text accurately to Urdu while preserving the meaning and context. Only provide the translation and nothing else.",
                    messages: [{ role: "user", content: translationPrompt }]
                });
                console.log('[translateToUrdu] Claude translation successful.');
                return message.content[0].text;
            } catch (error) {
                console.error('[translateToUrdu] Claude translation failed:', error);
            }
        } else if (aiProvider === 'openai' && openai) {
            try {
                console.log('[translateToUrdu] Attempting translation with OpenAI...');
                const chatCompletion = await openai.chat.completions.create({
                    model: "gpt-4o",
                    messages: [
                        { role: "system", content: "You are a professional translator. Translate text accurately to Urdu while preserving the meaning and context. Only provide the translation and nothing else." },
                        { role: "user", content: translationPrompt }
                    ],
                    max_tokens: 1024,
                    temperature: 0.3,
                });
                console.log('[translateToUrdu] OpenAI translation successful.');
                return chatCompletion.choices[0].message.content;
            } catch (error) {
                console.error('[translateToUrdu] OpenAI translation failed:', error);
            }
        } else if (aiProvider === 'gemini' && gemini) {
            try {
                console.log('[translateToUrdu] Attempting translation with Gemini...');
                const model = gemini.getGenerativeModel({ model: "gemini-1.5-flash-latest" });
                const result = await model.generateContent(translationPrompt);
                const response = await result.response;
                console.log('[translateToUrdu] Gemini translation successful.');
                return response.text();
            } catch (error) {
                console.error('[translateToUrdu] Gemini translation failed:', error);
            }
        }

        console.log('[translateToUrdu] No specific provider requested or the requested one failed. Trying providers in order of preference.');
        const providers = ['claude', 'openai', 'gemini'];
        for (const provider of providers) {
            if (provider === 'claude' && anthropic) {
                try {
                    console.log('[translateToUrdu] Trying Claude as a fallback...');
                    const message = await anthropic.messages.create({
                        model: "claude-3-5-sonnet-20241022",
                        max_tokens: 1024,
                        system: "You are a professional translator. Translate text accurately to Urdu while preserving the meaning and context. Only provide the translation and nothing else.",
                        messages: [{ role: "user", content: translationPrompt }]
                    });
                    console.log('[translateToUrdu] Claude fallback translation successful.');
                    return message.content[0].text;
                } catch (error) {
                    console.error('[translateToUrdu] Claude fallback failed:', error);
                    continue;
                }
            } else if (provider === 'openai' && openai) {
                try {
                    console.log('[translateToUrdu] Trying OpenAI as a fallback...');
                    const chatCompletion = await openai.chat.completions.create({
                        model: "gpt-4o",
                        messages: [
                            { role: "system", content: "You are a professional translator. Translate text accurately to Urdu while preserving the meaning and context. Only provide the translation and nothing else." },
                            { role: "user", content: translationPrompt }
                        ],
                        max_tokens: 1024,
                        temperature: 0.3,
                    });
                    console.log('[translateToUrdu] OpenAI fallback translation successful.');
                    return chatCompletion.choices[0].message.content;
                } catch (error) {
                    console.error('[translateToUrdu] OpenAI fallback failed:', error);
                    continue;
                }
            } else if (provider === 'gemini' && gemini) {
                try {
                    console.log('[translateToUrdu] Trying Gemini as a fallback...');
                    const model = gemini.getGenerativeModel({ model: "gemini-1.5-flash-latest" });
                    const result = await model.generateContent(translationPrompt);
                    const response = await result.response;
                    console.log('[translateToUrdu] Gemini fallback translation successful.');
                    return response.text();
                } catch (error) {
                    console.error('[translateToUrdu] Gemini fallback failed:', error);
                    continue;
                }
            }
        }

        console.log('[translateToUrdu] All translation providers failed. Using basic dictionary translation.');
        return basicUrduTranslation(text);
    } catch (error) {
        console.error('[translateToUrdu] A critical error occurred in the translation function:', error);
        return basicUrduTranslation(text);
    }
}

// Basic Urdu translation using dictionary for common robotics terms
function basicUrduTranslation(text) {
    // Common robotics and AI terms dictionary
    const dictionary = {
        'robot': 'روبوٹ',
        'robotics': 'روبوٹکس',
        'artificial intelligence': 'مصنوعی ذہانت',
        'AI': 'اے آئی',
        'ROS': 'آر او ایس',
        'sensor': 'سینسر',
        'actuator': 'ایکچویٹر',
        'machine learning': 'مشین لرننگ',
        'deep learning': 'ڈیپ لرننگ',
        'computer vision': 'کمپیوٹر ویژن',
        'neural network': 'نیورل نیٹ ورک',
        'algorithm': 'الگورتھم',
        'control': 'کنٹرول',
        'system': 'سسٹم',
        'programming': 'پروگرامنگ',
        'software': 'سافٹ ویئر',
        'hardware': 'ہارڈ ویئر',
        'simulation': 'سمیولیشن',
        'autonomous': 'خود مختار',
        'navigation': 'نیویگیشن',
        'Hello': 'ہیلو',
        'Hi': 'سلام',
        'How can I help': 'میں کیسے مدد کر سکتا ہوں',
        'Thank you': 'شکریہ',
        'Welcome': 'خوش آمدید',
        'Physical AI': 'فزیکل اے آئی',
        'I am': 'میں ہوں',
        'chatbot': 'چیٹ بوٹ',
        'assistant': 'اسسٹنٹ',
        'for': 'کے لیے',
        'the': '',
        'a': 'ایک',
        'and': 'اور',
        'or': 'یا',
        'can': 'کر سکتا',
        'help': 'مدد',
        'you': 'آپ',
        'your': 'آپ کا',
        'understand': 'سمجھنے میں',
        'explain': 'وضاحت کریں',
        'learn': 'سیکھیں',
        'course': 'کورس',
        'question': 'سوال',
        'answer': 'جواب',
        'Please': 'براہ کرم',
        'try': 'کوشش کریں',
        'again': 'دوبارہ',
        'later': 'بعد میں',
        'Sorry': 'معذرت',
        'error': 'خرابی',
        'problem': 'مسئلہ'
    };

    // Common complete phrase translations
    const phrases = {
        "I'm a chatbot assistant for the Physical AI & Robotics course":
            'میں فزیکل اے آئی اور روبوٹکس کورس کا چیٹ بوٹ اسسٹنٹ ہوں',
        "How can I help you?":
            'میں آپ کی کیسے مدد کر سکتا ہوں؟',
        "I can help you understand":
            'میں آپ کو سمجھنے میں مدد کر سکتا ہوں',
        "Please ask me a question":
            'براہ کرم مجھ سے ایک سوال پوچھیں',
        "I'm here to help":
            'میں یہاں مدد کے لیے موجود ہوں',
        "Welcome to the robotics course":
            'روبوٹکس کورس میں خوش آمدید',
        "Let me explain":
            'میں وضاحت کرتا ہوں',
        "Thank you for your question":
            'آپ کے سوال کا شکریہ',
        "Sorry, I couldn't process your request":
            'معذرت، میں آپ کی درخواست پر کارروائی نہیں کر سکا',
        "Please try again":
            'براہ کرم دوبارہ کوشش کریں',
        "I'm having trouble connecting":
            'مجھے رابطہ قائم کرنے میں دشواری ہو رہی ہے',
        "I'm sorry":
            'مجھے افسوس ہے',
        "Could you please rephrase":
            'کیا آپ براہ کرم دوبارہ کہہ سکتے ہیں'
    };

    // Check for complete phrase match first
    for (const [english, urdu] of Object.entries(phrases)) {
        if (text.includes(english)) {
            text = text.replace(english, urdu);
        }
    }

    // Replace individual words
    for (const [english, urdu] of Object.entries(dictionary)) {
        // Case-insensitive replacement
        const regex = new RegExp('\\b' + english + '\\b', 'gi');
        text = text.replace(regex, urdu);
    }

    return text;
}

// Protect the chat endpoint with authentication
const { protect } = require('./middleware/auth');

// Public Chat API endpoint (no authentication required)
app.post('/api/public/chat', async (req, res) => {
    const { query, language } = req.body;

    if (!query) {
        return res.status(400).json({
            success: false,
            message: 'Query is required'
        });
    }

    console.log(`Received query from guest: "${query.substring(0, 100)}${query.length > 100 ? '...' : ''}" in language: ${language || 'default'}`);

    try {
        let answer = '';
        let sourceInfo = 'fallback';

        // Try OpenAI API first if available (most reliable for guests)
        if (openai) {
            console.log("Attempting to use OpenAI API for guest...");
            try {
                const chatCompletion = await openai.chat.completions.create({
                    model: "gpt-4o",
                    messages: [
                        {
                            role: "system",
                            content: "You are a helpful AI assistant for a Physical AI & Robotics course. Help students understand concepts related to ROS2, robotics simulation, digital twins, AI robot brains, vision-language-action models, and related topics. Provide clear, concise explanations. Always be accurate, educational, and encourage deeper learning."
                        },
                        {
                            role: "user",
                            content: query
                        }
                    ],
                    max_tokens: 1024,
                    temperature: 0.7,
                });

                answer = chatCompletion.choices[0].message.content;
                sourceInfo = 'openai';
                console.log("Successfully responded using OpenAI API");
            } catch (openaiError) {
                console.error("OpenAI API failed:", openaiError.message || openaiError);
                // Fall back to enhanced fallback
                answer = getResponse(query);
                sourceInfo = 'enhanced_fallback';
            }
        } else {
            // Use enhanced fallback
            answer = getResponse(query);
            sourceInfo = 'enhanced_fallback';
        }

        // If the user requested Urdu translation, translate the answer
        if (language && language.toLowerCase() === 'urdu') {
            console.log("Translating response to Urdu...");
            answer = await translateToUrdu(answer, sourceInfo);
            sourceInfo += '_with_urdu_translation';
        }

        console.log(`Responding to guest with source: ${sourceInfo}`);
        res.json({
            success: true,
            answer: answer,
            source: sourceInfo,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error("Critical error in public chat API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during chat processing.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// Enhanced Chat API endpoint with fallback options, Urdu translation support, and authentication
app.post('/api/chat', protect, async (req, res) => {
    const { query, language } = req.body; // Added language parameter

    if (!query) {
        return res.status(400).json({
            success: false,
            message: 'Query is required'
        });
    }

    console.log(`Received query: "${query.substring(0, 100)}${query.length > 100 ? '...' : ''}" in language: ${language || 'default'} from user: ${req.user.email}`);

    try {
        let answer = '';
        let sourceInfo = 'fallback';

                // Try OpenAI API first if available
                if (openai) {
                    console.log("Attempting to use OpenAI API...");
                    try {
                        const chatCompletion = await openai.chat.completions.create({
                            model: "gpt-4o", // Using gpt-4o for better performance and cost
                            messages: [
                                {
                                    role: "system",
                                    content: "You are a helpful AI assistant for a Physical AI & Robotics course. Help students understand concepts related to ROS2, robotics simulation, digital twins, AI robot brains, vision-language-action models, and related topics. Provide clear, concise explanations based on the context provided. Always be accurate, educational, and encourage deeper learning."
                                },
                                {
                                    role: "user",
                                    content: query
                                }
                            ],
                            max_tokens: 1024,
                            temperature: 0.7,
                        });
        
                        answer = chatCompletion.choices[0].message.content;
                        sourceInfo = 'openai';
                        console.log("Successfully responded using OpenAI API");
                    } catch (openaiError) {
                        console.error("OpenAI API failed:", openaiError.message || openaiError);
        
                        // Try Claude API if OpenAI failed
                        if (anthropic) {
                            console.log("Attempting to use Claude API...");
                            try {
                                const message = await anthropic.messages.create({
                                    model: "claude-3-5-sonnet-20241022",
                                    max_tokens: 1024,
                                    system: "You are a helpful AI assistant for a Physical AI & Robotics course. Help students understand concepts related to ROS2, robotics simulation, digital twins, AI robot brains, vision-language-action models, and related topics. Provide clear, concise explanations based on the context provided. Always be accurate, educational, and encourage deeper learning.",
                                    messages: [
                                        {
                                            role: "user",
                                            content: query
                                        }
                                    ]
                                });
        
                                answer = message.content[0].text;
                                sourceInfo = 'claude';
                                console.log("Successfully responded using Claude API");
                            } catch (claudeError) {
                                console.error("Claude API failed:", claudeError.message || claudeError);
                                // Fallback to RAG
                                try {
                                    console.log("Attempting to use local RAG system...");
                                    const relevantDocs = global.localRAG ? global.localRAG.retrieveRelevantDocuments(query, 3) : [];
        
                                    if (relevantDocs.length > 0) {
                                        console.log(`Found ${relevantDocs.length} relevant documents from local RAG`);
                                        const context = relevantDocs.map(doc => doc.text).join('\n\n');
                                        const combinedQuery = `Based on the following context from a Physical AI & Robotics textbook, please answer the question. If the context doesn't contain enough information, say so.\n\nContext:\n${context}\n\nQuestion: ${query}\n\nAnswer:`;
        
                                        // Try Gemini if available
                                        if (gemini) {
                                            try {
                                                console.log("Attempting to use Gemini with local RAG context...");
                                                const model = gemini.getGenerativeModel({ model: "gemini-1.5-flash-latest" });
                                                const result = await model.generateContent(combinedQuery);
                                                const response = await result.response;
                                                answer = response.text();
                                                sourceInfo = 'gemini_with_rag';
                                                console.log("Successfully responded using Gemini with local RAG context");
                                            } catch (geminiError) {
                                                console.log("Gemini failed with RAG context, using RAG context directly...");
                                                sourceInfo = 'rag_context_only';
                                                // Generate answer from context directly
                                                answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                                            }
                                        } else {
                                            // Use context directly
                                            sourceInfo = 'rag_context_only';
                                            answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                                            console.log("Responding using RAG context directly (no Gemini)");
                                        }
                                    } else {
                                        console.log("No relevant documents found in local RAG, using enhanced fallback...");
                                        answer = getResponse(query);
                                        sourceInfo = 'enhanced_fallback';
                                    }
                                } catch (ragError) {
                                    console.error("Local RAG failed:", ragError.message);
                                    console.error("Stack trace:", ragError.stack);
                                    answer = getResponse(query);
                                    sourceInfo = 'enhanced_fallback_after_rag_error';
                                }
                            }
                        }
                    }
                }
                // If OpenAI is not available, try Claude API first
                else if (anthropic) {
                    console.log("Attempting to use Claude API...");
                    try {
                        const message = await anthropic.messages.create({
                            model: "claude-3-5-sonnet-20241022",
                            max_tokens: 1024,
                            system: "You are a helpful AI assistant for a Physical AI & Robotics course. Help students understand concepts related to ROS2, robotics simulation, digital twins, AI robot brains, vision-language-action models, and related topics. Provide clear, concise explanations based on the context provided. Always be accurate, educational, and encourage deeper learning.",
                            messages: [
                                {
                                    role: "user",
                                    content: query
                                }
                            ]
                        });
        
                        answer = message.content[0].text;
                        sourceInfo = 'claude';
                        console.log("Successfully responded using Claude API");
                    } catch (claudeError) {
                        console.error("Claude API failed:", claudeError.message || claudeError);
                        // Fallback to RAG
                        try {
                            console.log("Attempting to use local RAG system...");
                            const relevantDocs = global.localRAG ? global.localRAG.retrieveRelevantDocuments(query, 3) : [];
        
                            if (relevantDocs.length > 0) {
                                console.log(`Found ${relevantDocs.length} relevant documents from local RAG`);
                                const context = relevantDocs.map(doc => doc.text).join('\n\n');
                                const combinedQuery = `Based on the following context from a Physical AI & Robotics textbook, please answer the question. If the context doesn't contain enough information, say so.\n\nContext:\n${context}\n\nQuestion: ${query}\n\nAnswer:`;
        
                                // Try Gemini if available
                                if (gemini) {
                                    try {
                                        console.log("Attempting to use Gemini with local RAG context...");
                                        const model = gemini.getGenerativeModel({ model: "gemini-1.5-flash-latest" });
                                        const result = await model.generateContent(combinedQuery);
                                        const response = await result.response;
                                        answer = response.text();
                                        sourceInfo = 'gemini_with_rag';
                                        console.log("Successfully responded using Gemini with local RAG context");
                                    } catch (geminiError) {
                                        console.log("Gemini failed with RAG context, using RAG context directly...");
                                        sourceInfo = 'rag_context_only';
                                        // Generate answer from context directly
                                        answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                                    }
                                } else {
                                    // Use context directly
                                    sourceInfo = 'rag_context_only';
                                    answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                                    console.log("Responding using RAG context directly (no Gemini)");
                                }
                            } else {
                                console.log("No relevant documents found in local RAG, using enhanced fallback...");
                                answer = getResponse(query);
                                sourceInfo = 'enhanced_fallback';
                            }
                        } catch (ragError) {
                            console.error("Local RAG failed:", ragError.message);
                            console.error("Stack trace:", ragError.stack);
                            answer = getResponse(query);
                            sourceInfo = 'enhanced_fallback_after_rag_error';
                        }
                    }
                }
                // If Claude and OpenAI are not available, try local RAG with Gemini
                else {            console.log("Claude and OpenAI APIs not available, attempting to use local RAG system...");
            try {
                const relevantDocs = global.localRAG ? global.localRAG.retrieveRelevantDocuments(query, 3) : [];

                if (relevantDocs.length > 0 && gemini) {
                    console.log(`Found ${relevantDocs.length} relevant documents from local RAG, attempting to use with Gemini...`);
                    const context = relevantDocs.map(doc => doc.text).join('\n\n');
                    const combinedQuery = `Based on the following context from a Physical AI & Robotics textbook, please answer the question. If the context doesn't contain enough information, say so.\n\nContext:\n${context}\n\nQuestion: ${query}\n\nAnswer:`;

                    try {
                        const model = gemini.getGenerativeModel({ model: "gemini-1.5-flash-latest" });
                        const result = await model.generateContent(combinedQuery);
                        const response = await result.response;
                        answer = response.text();
                        sourceInfo = 'gemini_with_rag';
                        console.log("Successfully responded using Gemini with local RAG context");
                    } catch (geminiError) {
                        console.log("Gemini failed with RAG context, using RAG context directly...");
                        sourceInfo = 'rag_context_only';
                        answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                    }
                } else if (relevantDocs.length > 0) {
                    // Use context directly without LLM
                    sourceInfo = 'rag_context_only';
                    const context = relevantDocs.map(doc => doc.text).join('\n\n');
                    answer = `Based on the Physical AI & Robotics textbook:\n\n${context.substring(0, 1500)}...\n\nSources: ${relevantDocs.map(d => d.metadata.source).join(', ')}`;
                    console.log("Responding using RAG context directly (no Claude or Gemini)");
                } else {
                    console.log("No relevant documents found, using enhanced fallback...");
                    answer = getResponse(query);
                    sourceInfo = 'enhanced_fallback';
                }
            } catch (ragError) {
                console.error("Local RAG failed:", ragError.message);
                console.error("Stack trace:", ragError.stack);
                answer = getResponse(query);
                sourceInfo = 'enhanced_fallback_after_rag_error';
            }
        }

        // If the user requested Urdu translation, translate the answer
        if (language && language.toLowerCase() === 'urdu') {
            console.log("Translating response to Urdu...");
            answer = await translateToUrdu(answer);
            sourceInfo += '_with_urdu_translation'; // Update source info to indicate translation was applied
        }

        console.log(`Responding with source: ${sourceInfo}`);
        res.json({
            success: true,
            answer: answer,
            source: sourceInfo,  // Added source information for debugging
            timestamp: new Date().toISOString()  // Added timestamp for debugging
        });
    } catch (error) {
        console.error("Critical error in chat API:", error);
        console.error("Stack trace:", error.stack);
        res.status(500).json({
            success: false,
            message: 'Internal server error during chat processing.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// Helper function to load book content lazily (without loading all chapter content)
function getBookStructure() {
    try {
        // Load the full book content
        const fullContent = require('./book_content');

        // Extract only the structure (metadata) without full chapter content
        const structure = {
            title: fullContent.title,
            description: fullContent.description,
            author: fullContent.author,
            authorBio: fullContent.authorBio,
            publicationDate: fullContent.publicationDate,
            version: fullContent.version,
            totalModules: fullContent.totalModules,
            totalChapters: fullContent.totalChapters,
            modules: fullContent.modules.map(module => ({
                id: module.id,
                title: module.title,
                description: module.description,
                difficulty: module.difficulty,
                estimatedTime: module.estimatedTime,
                chapters: module.chapters.map(chapter => ({
                    id: chapter.id,
                    title: chapter.title,
                    learningObjectives: chapter.learningObjectives,
                    readingTime: chapter.readingTime,
                    keywords: chapter.keywords
                    // Note: we exclude the full 'content' field here to keep response small
                }))
            }))
        };

        return structure;
    } catch (error) {
        console.error('Error loading book structure:', error);
        throw error;
    }
}

// Helper function to get a specific chapter by ID
function getChapterById(chapterId) {
    try {
        const fullContent = require('./book_content');

        // Search for the chapter across all modules
        for (const module of fullContent.modules) {
            const chapter = module.chapters.find(ch => ch.id === chapterId);
            if (chapter) {
                return {
                    ...chapter,
                    moduleInfo: {
                        id: module.id,
                        title: module.title,
                        difficulty: module.difficulty
                    }
                };
            }
        }

        return null;
    } catch (error) {
        console.error('Error loading chapter:', error);
        throw error;
    }
}

// NEW: Public route for book structure (metadata only, fast response)
app.get('/api/public/book-structure', async (req, res) => {
    try {
        console.log('Guest user accessed book structure');

        const structure = getBookStructure();

        res.json({
            success: true,
            message: 'Book structure retrieved successfully',
            data: structure,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error("Error in public book structure API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during book structure retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// NEW: Public route for individual chapter content
app.get('/api/public/chapter/:chapterId', async (req, res) => {
    try {
        const { chapterId } = req.params;
        console.log(`Guest user accessed chapter: ${chapterId}`);

        const chapter = getChapterById(chapterId);

        if (!chapter) {
            return res.status(404).json({
                success: false,
                message: 'Chapter not found',
                timestamp: new Date().toISOString()
            });
        }

        res.json({
            success: true,
            message: 'Chapter content retrieved successfully',
            data: chapter,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error("Error in public chapter API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during chapter retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// DEPRECATED: Old endpoint - kept for backwards compatibility but now uses lazy loading
app.get('/api/public/book-content', async (req, res) => {
    try {
        console.log('Guest user accessed book content (deprecated - use /book-structure instead)');

        // Return only the structure to avoid timeout
        const structure = getBookStructure();

        res.json({
            success: true,
            message: 'Book structure retrieved successfully. Use /api/public/chapter/:chapterId for chapter content.',
            data: {
                bookContent: structure,
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in public book content API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during book content retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// Protected route for book structure (for logged-in users)
app.get('/api/book-structure', protect, async (req, res) => {
    try {
        console.log(`User ${req.user.email} accessed book structure`);

        const structure = getBookStructure();

        res.json({
            success: true,
            message: 'Book structure retrieved successfully',
            data: {
                user: {
                    id: req.user.id,
                    name: req.user.name,
                    email: req.user.email
                },
                bookContent: structure,
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in book structure API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during book structure retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// Protected route for individual chapter (for logged-in users)
app.get('/api/chapter/:chapterId', protect, async (req, res) => {
    try {
        const { chapterId } = req.params;
        console.log(`User ${req.user.email} accessed chapter: ${chapterId}`);

        const chapter = getChapterById(chapterId);

        if (!chapter) {
            return res.status(404).json({
                success: false,
                message: 'Chapter not found',
                timestamp: new Date().toISOString()
            });
        }

        res.json({
            success: true,
            message: 'Chapter content retrieved successfully',
            data: {
                user: {
                    id: req.user.id,
                    name: req.user.name,
                    email: req.user.email
                },
                chapter: chapter,
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in chapter API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during chapter retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// DEPRECATED: Protected route for book content (kept for backwards compatibility)
app.get('/api/book-content', protect, async (req, res) => {
    try {
        console.log(`User ${req.user.email} accessed book content (deprecated - use /book-structure instead)`);

        const structure = getBookStructure();

        res.json({
            success: true,
            message: 'Book structure retrieved successfully. Use /api/chapter/:chapterId for chapter content.',
            data: {
                user: {
                    id: req.user.id,
                    name: req.user.name,
                    email: req.user.email
                },
                bookContent: structure,
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in book content API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during book content retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// New protected route for book modules/progress
app.get('/api/book-progress', protect, async (req, res) => {
    try {
        // Find the user in the in-memory store
        const user = Array.isArray(global.users)
            ? global.users.find(u => u.id === req.user.id)
            : users.find(u => u.id === req.user.id);

        if (!user) {
            return res.status(404).json({
                success: false,
                message: 'User not found'
            });
        }

        res.json({
            success: true,
            data: {
                progress: {
                    completedModules: user.completedModules || [],
                    currentModule: user.currentModule || null,
                    bookmarks: user.bookmarks || [],
                    completedPercentage: user.completedModules ?
                        Math.round((user.completedModules.length / Object.keys(knowledgeBase).length) * 100) : 0
                },
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in book progress API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during progress retrieval.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});

// New protected route to update book progress
app.put('/api/book-progress', protect, async (req, res) => {
    try {
        const { completedModule, bookmark, currentModule } = req.body;

        // Find the user in the in-memory store
        const userIndex = Array.isArray(global.users)
            ? global.users.findIndex(u => u.id === req.user.id)
            : users.findIndex(u => u.id === req.user.id);

        if (userIndex === -1) {
            return res.status(404).json({
                success: false,
                message: 'User not found'
            });
        }

        const userArray = Array.isArray(global.users) ? global.users : users;
        const user = userArray[userIndex];

        // Update user progress
        if (completedModule && !user.completedModules.includes(completedModule)) {
            user.completedModules.push(completedModule);
        }

        if (bookmark) {
            // Add bookmark if it doesn't exist
            const existingBookmark = user.bookmarks.find(b => b.chapter === bookmark.chapter);
            if (existingBookmark) {
                existingBookmark.page = bookmark.page;
                existingBookmark.timestamp = bookmark.timestamp;
            } else {
                user.bookmarks.push(bookmark);
            }
        }

        if (currentModule) {
            user.currentModule = currentModule;
        }

        res.json({
            success: true,
            message: 'Progress updated successfully',
            data: {
                progress: {
                    completedModules: user.completedModules,
                    currentModule: user.currentModule,
                    bookmarks: user.bookmarks,
                    completedPercentage: Math.round((user.completedModules.length / Object.keys(knowledgeBase).length) * 100)
                },
                timestamp: new Date().toISOString()
            }
        });
    } catch (error) {
        console.error("Error in book progress update API:", error);
        res.status(500).json({
            success: false,
            message: 'Internal server error during progress update.',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined,
            timestamp: new Date().toISOString()
        });
    }
});


// AI-Powered MCQ Generation Endpoint
app.post('/api/generate-mcqs', protect, async (req, res) => {
    const { chapterId, chapterContent, numberOfQuestions = 5 } = req.body;

    if (!chapterContent) {
        return res.status(400).json({
            success: false,
            message: 'Chapter content is required'
        });
    }

    console.log(`Generating ${numberOfQuestions} MCQs for chapter: ${chapterId} for user: ${req.user.email}`);

    try {
        const mcqPrompt = `Based on the following educational content, generate ${numberOfQuestions} multiple-choice questions (MCQs) to test comprehension. Each question should have 4 options (A, B, C, D) with only one correct answer.

Content:
${chapterContent.substring(0, 3000)}

Return the response in this exact JSON format:
{
  "questions": [
    {
      "question": "Question text here?",
      "options": {
        "A": "Option A text",
        "B": "Option B text",
        "C": "Option C text",
        "D": "Option D text"
      },
      "correctAnswer": "A",
      "explanation": "Brief explanation of why this is correct"
    }
  ]
}

Important: Make questions that test deep understanding, not just memorization.`;

        let mcqResponse = null;

        // Try OpenAI first
        if (openai) {
            try {
                console.log('Generating MCQs with OpenAI...');
                const chatCompletion = await openai.chat.completions.create({
                    model: "gpt-4o",
                    messages: [
                        {
                            role: "system",
                            content: "You are an expert educator creating assessment questions. Always return valid JSON only, no markdown or extra text."
                        },
                        {
                            role: "user",
                            content: mcqPrompt
                        }
                    ],
                    max_tokens: 2000,
                    temperature: 0.7,
                    response_format: { type: "json_object" }
                });

                mcqResponse = JSON.parse(chatCompletion.choices[0].message.content);
                console.log('MCQs generated successfully with OpenAI');
            } catch (openaiError) {
                console.error('OpenAI MCQ generation failed:', openaiError.message);
            }
        }

        // Fallback to Claude if OpenAI fails
        if (!mcqResponse && anthropic) {
            try {
                console.log('Generating MCQs with Claude...');
                const message = await anthropic.messages.create({
                    model: "claude-3-5-sonnet-20241022",
                    max_tokens: 2000,
                    system: "You are an expert educator creating assessment questions. Always return valid JSON only.",
                    messages: [{ role: "user", content: mcqPrompt }]
                });

                const responseText = message.content[0].text;
                mcqResponse = JSON.parse(responseText);
                console.log('MCQs generated successfully with Claude');
            } catch (claudeError) {
                console.error('Claude MCQ generation failed:', claudeError.message);
            }
        }

        // Fallback questions if all AI fails
        if (!mcqResponse) {
            console.log('Using fallback MCQs');
            mcqResponse = {
                questions: [
                    {
                        question: "What is the main focus of this chapter?",
                        options: {
                            A: "Understanding fundamental concepts",
                            B: "Memorizing definitions",
                            C: "Skipping important details",
                            D: "Ignoring practical applications"
                        },
                        correctAnswer: "A",
                        explanation: "The chapter focuses on understanding fundamental concepts to build a strong foundation."
                    },
                    {
                        question: "Why is this topic important in robotics?",
                        options: {
                            A: "It's not important",
                            B: "It forms the basis for advanced applications",
                            C: "It's only for theory",
                            D: "It's outdated"
                        },
                        correctAnswer: "B",
                        explanation: "This topic is crucial as it forms the foundation for more advanced robotic applications."
                    }
                ]
            };
        }

        res.json({
            success: true,
            mcqs: mcqResponse.questions,
            chapterId: chapterId,
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('Critical error in MCQ generation:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to generate MCQs',
            error: process.env.NODE_ENV === 'development' ? error.message : undefined
        });
    }
});

// Submit Assessment Endpoint
app.post('/api/submit-assessment', protect, async (req, res) => {
    const { chapterId, answers, totalQuestions, timeSpent } = req.body;

    if (!chapterId || !answers) {
        return res.status(400).json({
            success: false,
            message: 'Chapter ID and answers are required'
        });
    }

    try {
        // Calculate score
        const correctAnswers = answers.filter(a => a.isCorrect).length;
        const score = Math.round((correctAnswers / totalQuestions) * 100);

        const assessment = {
            chapterId,
            userId: req.user.id,
            score,
            correctAnswers,
            totalQuestions,
            answers,
            timeSpent,
            completedAt: new Date().toISOString()
        };

        // Find user and store assessment
        const userArray = Array.isArray(global.users) ? global.users : users;
        const user = userArray.find(u => u.id === req.user.id);

        if (user) {
            if (!user.assessments) {
                user.assessments = [];
            }
            user.assessments.push(assessment);

            // Mark chapter as completed if score >= 70%
            if (score >= 70 && !user.completedModules.includes(chapterId)) {
                user.completedModules.push(chapterId);
            }
        }

        console.log(`Assessment submitted for ${req.user.email}: ${score}% on ${chapterId}`);

        res.json({
            success: true,
            assessment: {
                score,
                correctAnswers,
                totalQuestions,
                passed: score >= 70,
                completedAt: assessment.completedAt
            }
        });
    } catch (error) {
        console.error('Error submitting assessment:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to submit assessment'
        });
    }
});

// Get Assessment History
app.get('/api/assessments', protect, async (req, res) => {
    try {
        const userArray = Array.isArray(global.users) ? global.users : users;
        const user = userArray.find(u => u.id === req.user.id);

        if (!user) {
            return res.status(404).json({
                success: false,
                message: 'User not found'
            });
        }

        res.json({
            success: true,
            assessments: user.assessments || [],
            timestamp: new Date().toISOString()
        });
    } catch (error) {
        console.error('Error fetching assessments:', error);
        res.status(500).json({
            success: false,
            message: 'Failed to fetch assessments'
        });
    }
});

// Health check endpoint
app.get('/health', (req, res) => {
  console.log('Health check endpoint hit');
  res.json({ status: 'OK', message: 'Server is running', database: 'PostgreSQL (Neon DB)' });
});

// Error handling middleware
app.use((err, req, res, next) => {
  console.error(err.stack);
  res.status(500).json({
    success: false,
    message: 'Internal server error',
    error: process.env.NODE_ENV === 'development' ? err.message : undefined
  });
});

// 404 handler
app.use((req, res) => {
  res.status(404).json({
    success: false,
    message: 'Route not found'
  });
});

// Graceful shutdown
process.on('SIGINT', async () => {
  await prisma.$disconnect();
  process.exit(0);
});

process.on('SIGTERM', async () => {
  await prisma.$disconnect();
  process.exit(0);
});

// Start server
const PORT = process.env.PORT || 5001;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
  console.log(`Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log(`Database: PostgreSQL (Neon DB)`);
  console.log("Chatbot ready (supporting Claude 3.5 Sonnet, OpenAI GPT-4o, and Google Gemini)");
});

module.exports = app;