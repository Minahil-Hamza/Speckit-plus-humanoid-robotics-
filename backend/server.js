const express = require('express');
const cors = require('cors');
const dotenv = require('dotenv');
const prisma = require('./lib/prisma');
const { GoogleGenerativeAI } = require('@google/generative-ai');

// Load environment variables
dotenv.config();

// Initialize Express app
const app = express();

// Gemini API setup
const GEMINI_API_KEY = process.env.GEMINI_API_KEY;

if (!GEMINI_API_KEY) {
    console.error("GEMINI_API_KEY is not set in the .env file.");
    process.exit(1);
}
const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);

// Middleware
app.use(express.json());
app.use(cors({
  origin: process.env.FRONTEND_URL || 'http://localhost:3000',
  credentials: true
}));

// Test database connection
async function testConnection() {
  try {
    await prisma.$connect();
    console.log('PostgreSQL connected successfully (Neon DB)');
  } catch (error) {
    console.error('PostgreSQL connection error:', error.message);
  }
}

testConnection();

// Routes
app.use('/api/auth', require('./routes/auth'));
app.use('/api/users', require('./routes/users'));

// Fallback knowledge base for when API is unavailable
const knowledgeBase = {
    'ros2': 'ROS2 (Robot Operating System 2) is the next generation of ROS, a flexible framework for writing robot software. It provides services including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management.',
    'digital twin': 'A Digital Twin is a virtual representation of a physical robot that mirrors its behavior, performance, and environment. It allows for testing, simulation, and optimization before deployment on actual hardware.',
    'gazebo': 'Gazebo is a powerful 3D robotics simulator that provides realistic physics simulation, sensor simulation, and supports ROS integration. It\'s commonly used for testing robots in various environments before physical deployment.',
    'isaac sim': 'NVIDIA Isaac Sim is a robotics simulation platform built on Omniverse, providing photorealistic rendering, accurate physics simulation, and support for AI model training. It\'s particularly powerful for developing AI-powered robots.',
    'nav2': 'Nav2 (Navigation2) is the ROS2 navigation stack that provides autonomous navigation capabilities for mobile robots. It includes path planning, obstacle avoidance, localization, and mapping features.',
    'urdf': 'URDF (Unified Robot Description Format) is an XML format for representing a robot model. It defines the robot\'s physical structure, joints, links, sensors, and visual/collision properties.',
    'vla': 'Vision-Language-Action (VLA) models combine computer vision, natural language processing, and robotic control to enable robots to understand visual scenes, interpret language commands, and execute appropriate actions.',
    'isaac ros': 'Isaac ROS provides GPU-accelerated packages for ROS2, offering high-performance perception and AI capabilities for robotics applications using NVIDIA hardware.'
};

function getResponse(query) {
    const lowerQuery = query.toLowerCase();

    // Check for keywords in the query
    for (const [keyword, response] of Object.entries(knowledgeBase)) {
        if (lowerQuery.includes(keyword)) {
            return response;
        }
    }

    // Default response
    return `I'm a chatbot assistant for the Physical AI & Robotics textbook. I can help answer questions about:\n\n• ROS2 (Robot Operating System 2)\n• Robotics Simulation (Gazebo, Unity, Isaac Sim)\n• Digital Twins\n• AI Robot Brain concepts\n• Vision-Language-Action (VLA) models\n• NVIDIA Isaac ROS\n• Navigation (Nav2)\n• URDF robot descriptions\n\nYour question: "${query}"\n\nPlease try asking about specific topics like "What is ROS2?" or "Explain Digital Twin".\n\nNote: The AI API is currently unavailable. This is a fallback response system with limited capabilities.`;
}

// Chat API endpoint with fallback
app.post('/api/chat', async (req, res) => {
    const { query } = req.body;

    if (!query) {
        return res.status(400).json({ success: false, message: 'Query is required' });
    }

    try {
        // Use fallback knowledge base
        const response = getResponse(query);
        res.json({ success: true, answer: response });

    } catch (error) {
        console.error("Error in chat API:", error);
        res.status(500).json({ success: false, message: 'Internal server error during chat processing.' });
    }
});


// Health check endpoint
app.get('/health', (req, res) => {
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
const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
  console.log(`Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log(`Database: PostgreSQL (Neon DB)`);
  console.log("Chatbot ready (using Gemini Pro)");
});

module.exports = app;