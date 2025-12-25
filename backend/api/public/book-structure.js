// Serverless function for book structure endpoint
const path = require('path');

module.exports = async (req, res) => {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization');

  // Handle OPTIONS request for CORS preflight
  if (req.method === 'OPTIONS') {
    return res.status(200).end();
  }

  try {
    // Load book metadata (optimized for serverless - small file)
    const metadata = require('../../book_metadata');

    res.status(200).json({
      success: true,
      message: 'Book structure retrieved successfully',
      data: metadata,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('Error loading book metadata:', error);
    res.status(500).json({
      success: false,
      message: 'Internal server error during book structure retrieval.',
      error: error.message,
      timestamp: new Date().toISOString()
    });
  }
};
