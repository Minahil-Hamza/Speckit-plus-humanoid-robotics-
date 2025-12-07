const express = require('express');
const router = express.Router();
const prisma = require('../lib/prisma');
const { protect } = require('../middleware/auth');

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
    if (action === 'complete') {
      // Check if module already completed
      const existingModule = await prisma.completedModule.findFirst({
        where: {
          userId: req.user.id,
          moduleId
        }
      });

      if (!existingModule) {
        // Add completed module
        await prisma.completedModule.create({
          data: {
            moduleId,
            userId: req.user.id
          }
        });
      }
    } else if (action === 'start') {
      // Update current module
      await prisma.user.update({
        where: { id: req.user.id },
        data: { currentModule: moduleId }
      });
    }

    // Get updated progress
    const user = await prisma.user.findUnique({
      where: { id: req.user.id },
      include: {
        completedModules: true
      }
    });

    res.json({
      success: true,
      message: 'Progress updated successfully',
      data: {
        progress: {
          completedModules: user.completedModules,
          currentModule: user.currentModule,
          bookmarks: user.bookmarks
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
    const user = await prisma.user.findUnique({
      where: { id: req.user.id }
    });

    let bookmarks = user.bookmarks || [];

    if (action === 'add') {
      if (!bookmarks.includes(pageUrl)) {
        bookmarks.push(pageUrl);
      }
    } else if (action === 'remove') {
      bookmarks = bookmarks.filter(bookmark => bookmark !== pageUrl);
    }

    // Update bookmarks
    await prisma.user.update({
      where: { id: req.user.id },
      data: { bookmarks }
    });

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
    const user = await prisma.user.findUnique({
      where: { id: req.user.id },
      include: {
        completedModules: true
      }
    });

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
