const express = require('express');
const router = express.Router();
const { protect } = require('../middleware/auth');

// Import the in-memory users from auth route
// Since we're using module-scoped variables, we'll access the users array differently
// We'll create a global variable to share the users data
if (!global.users) {
  global.users = [];
  global.nextUserId = 1;
}

// @route   PUT /api/users/progress
// @desc    Update user progress
// @access  Private
router.put('/progress', protect, async (req, res) => {
  const { moduleId, action } = req.body;

  if (!moduleId || !action) {
    return res.status(400).json({
      success: false,
      message: 'Please provide moduleId and action'
    });
  }

  try {
    const userIndex = global.users.findIndex(user => user.id === req.user.id);

    if (userIndex === -1) {
      return res.status(404).json({
        success: false,
        message: 'User not found'
      });
    }

    if (action === 'complete') {
      // Check if module already completed
      const existingModule = global.users[userIndex].completedModules.find(m => m.moduleId === moduleId);

      if (!existingModule) {
        // Add completed module
        global.users[userIndex].completedModules.push({
          moduleId,
          completedAt: new Date()
        });
      }
    } else if (action === 'start') {
      // Update current module
      global.users[userIndex].currentModule = moduleId;
    }

    res.json({
      success: true,
      message: 'Progress updated successfully',
      data: {
        progress: {
          completedModules: global.users[userIndex].completedModules,
          currentModule: global.users[userIndex].currentModule,
          bookmarks: global.users[userIndex].bookmarks
        }
      }
    });
  } catch (error) {
    console.error('Update progress error:', error);
    res.status(500).json({
      success: false,
      message: 'Error updating progress'
    });
  }
});

// @route   POST /api/users/bookmark
// @desc    Add or remove bookmark
// @access  Private
router.post('/bookmark', protect, async (req, res) => {
  const { pageUrl, action } = req.body;

  if (!pageUrl || !action) {
    return res.status(400).json({
      success: false,
      message: 'Please provide pageUrl and action (add/remove)'
    });
  }

  try {
    const userIndex = global.users.findIndex(user => user.id === req.user.id);

    if (userIndex === -1) {
      return res.status(404).json({
        success: false,
        message: 'User not found'
      });
    }

    let bookmarks = global.users[userIndex].bookmarks || [];

    if (action === 'add') {
      if (!bookmarks.includes(pageUrl)) {
        bookmarks.push(pageUrl);
      }
    } else if (action === 'remove') {
      bookmarks = bookmarks.filter(bookmark => bookmark !== pageUrl);
    }

    // Update bookmarks
    global.users[userIndex].bookmarks = bookmarks;

    res.json({
      success: true,
      message: `Bookmark ${action === 'add' ? 'added' : 'removed'} successfully`,
      data: { bookmarks }
    });
  } catch (error) {
    console.error('Bookmark error:', error);
    res.status(500).json({
      success: false,
      message: 'Error updating bookmark'
    });
  }
});

// @route   GET /api/users/stats
// @desc    Get user statistics
// @access  Private
router.get('/stats', protect, async (req, res) => {
  try {
    const userIndex = global.users.findIndex(user => user.id === req.user.id);

    if (userIndex === -1) {
      return res.status(404).json({
        success: false,
        message: 'User not found'
      });
    }

    const user = global.users[userIndex];
    const stats = {
      totalModulesCompleted: user.completedModules.length,
      currentModule: user.currentModule,
      bookmarksCount: user.bookmarks ? user.bookmarks.length : 0,
      memberSince: user.createdAt,
      lastActive: user.lastLogin
    };

    res.json({
      success: true,
      data: { stats }
    });
  } catch (error) {
    console.error('Get stats error:', error);
    res.status(500).json({
      success: false,
      message: 'Error fetching user statistics'
    });
  }
});

module.exports = router;