// Serverless function for individual chapter endpoint
// Vercel will map /api/public/chapter/:chapterId to this file

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
    // Get chapter ID from query params (Vercel dynamic route)
    const { chapterId } = req.query;

    if (!chapterId) {
      return res.status(400).json({
        success: false,
        message: 'Chapter ID is required',
        timestamp: new Date().toISOString()
      });
    }

    // Load full book content to get the specific chapter
    const bookContent = require('../../../book_content');

    // Search for the chapter across all modules
    let foundChapter = null;
    for (const module of bookContent.modules) {
      const chapter = module.chapters.find(ch => ch.id === chapterId);
      if (chapter) {
        foundChapter = {
          ...chapter,
          moduleInfo: {
            id: module.id,
            title: module.title,
            difficulty: module.difficulty
          }
        };
        break;
      }
    }

    if (!foundChapter) {
      return res.status(404).json({
        success: false,
        message: 'Chapter not found',
        timestamp: new Date().toISOString()
      });
    }

    res.status(200).json({
      success: true,
      message: 'Chapter content retrieved successfully',
      data: foundChapter,
      timestamp: new Date().toISOString()
    });
  } catch (error) {
    console.error('Error loading chapter:', error);
    res.status(500).json({
      success: false,
      message: 'Internal server error during chapter retrieval.',
      error: error.message,
      timestamp: new Date().toISOString()
    });
  }
};
