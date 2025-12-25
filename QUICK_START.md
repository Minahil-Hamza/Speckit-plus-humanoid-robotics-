# Quick Start Guide - Book Learning Platform

## 🚀 Get Started in 5 Minutes

### Step 1: Install Dependencies

```bash
# Backend
cd backend
npm install

# Frontend (if you want to run locally with a server)
# The frontend is pure HTML/CSS/JS, so you can open index.html directly
```

### Step 2: Configure Environment Variables

1. Open `backend/.env`
2. Add at least ONE API key for full functionality:

```env
# Option 1: OpenAI (Recommended - works best)
OPENAI_API_KEY=sk-your-openai-key-here

# Option 2: Claude (Anthropic)
CLAUDE_API_KEY=sk-ant-your-claude-key-here

# Option 3: Gemini (Google)
GEMINI_API_KEY=your-gemini-key-here

# Database (if using PostgreSQL)
DATABASE_URL=your-postgres-connection-string

# JWT Secret
JWT_SECRET=your-secret-key-here
JWT_EXPIRE=30d
```

### Step 3: Start the Backend

```bash
cd backend
npm start
```

Server will run on `http://localhost:5001`

### Step 4: Open the Frontend

```bash
# Option 1: Direct file opening
# Simply double-click frontend/index.html

# Option 2: With a local server (recommended for testing)
cd frontend
npx serve
# Then open http://localhost:3000
```

### Step 5: Update API URL (if needed)

If your backend is not on `localhost:5001`, update `frontend/script.js`:

```javascript
// Line 2
const API_BASE_URL = 'http://localhost:5001/api';
// Change to your backend URL
```

---

## 🧪 Testing the Features

### Test 1: User Registration and Login
1. Open frontend
2. Click "Register" tab
3. Fill in:
   - Name: Test User
   - Email: test@example.com
   - Password: test123
   - Language: English
4. Click "Register & Access Book"
5. Login with same credentials

✅ **Expected:** Confetti animation, access to book content

---

### Test 2: Browse Book Content
1. After login, see modules in left sidebar
2. Click on "Module 1: Introduction to Physical AI and Robotics"
3. Module expands to show chapters
4. Click "Chapter 1: What is Physical AI?"
5. Chapter loads with content and timer starts

✅ **Expected:** Chapter content displayed, timer shows `⏱️ 00:00`

---

### Test 3: Reading Timer (Quick Test Mode)

**Option A: Test with 2 minutes (for quick testing)**
1. Edit `frontend/script.js` line 737:
   ```javascript
   // Change from:
   const ASSESSMENT_TRIGGER_TIME = 20 * 60 * 1000;

   // To:
   const ASSESSMENT_TRIGGER_TIME = 2 * 60 * 1000; // 2 minutes
   ```
2. Refresh page and select a chapter
3. Wait 2 minutes
4. Assessment prompt appears

**Option B: Test with full 20 minutes**
1. Select any chapter
2. Wait 20 minutes
3. Assessment automatically triggers

✅ **Expected:** Prompt appears asking if you want to take assessment

---

### Test 4: Take Assessment
1. When prompted (or click "📝 Take Assessment" button)
2. Assessment generates (may take 5-10 seconds with AI)
3. Answer 5 multiple-choice questions
4. Click "Next →" after each answer
5. Click "Submit Assessment" after last question

✅ **Expected:**
- 5 AI-generated questions about the chapter
- Timer tracks assessment duration
- Progress bar shows question number
- Can navigate back to previous questions

---

### Test 5: View Assessment Results
1. After submitting assessment
2. See results page with:
   - Score percentage
   - Pass/Fail indicator (70% pass threshold)
   - Confetti if passed
   - Detailed breakdown of each question
   - Explanations for correct answers

✅ **Expected:**
- Results displayed beautifully
- Can see which questions were correct/incorrect
- Explanations help learning
- Options to retake or view dashboard

---

### Test 6: Check Dashboard
1. Click "📊 Dashboard" button in top right
2. See:
   - Overall progress circle
   - Statistics (modules completed, time spent)
   - Module-by-module progress
   - Assessment History section (if assessments taken)

✅ **Expected:**
- Progress accurately reflects completed chapters
- Assessment history shows all tests with scores
- Average score calculated
- Each assessment shows date, time, score

---

### Test 7: Urdu Translation
1. Scroll down to chat section
2. Select "Urdu" from "Response Language" dropdown
3. Type question: "What is Physical AI?"
4. Click "Send"
5. Wait for response

✅ **Expected:** Response in Urdu text

**Note:** Requires API key to be configured. If no API key, you'll see English fallback.

---

### Test 8: Guest Mode
1. Logout (if logged in)
2. Click "🚀 Start Learning Now" button
3. Browse and read chapters as guest
4. Try to take assessment

✅ **Expected:**
- Can read all content
- Cannot take assessments (prompted to create account)
- No progress saved

---

## 🎯 Quick Feature Checklist

Test each feature:

- [ ] User registration works
- [ ] Login works
- [ ] Book content loads
- [ ] Chapters display properly
- [ ] Author information shows in sidebar
- [ ] Reading timer starts when selecting chapter
- [ ] Timer counts up correctly
- [ ] Assessment triggers after set time
- [ ] MCQ questions generate (AI-powered)
- [ ] Can answer and navigate questions
- [ ] Assessment submits and grades correctly
- [ ] Results page shows detailed feedback
- [ ] Passing score (≥70%) celebrates with confetti
- [ ] Failed score (<70%) offers retake option
- [ ] Dashboard displays accurately
- [ ] Assessment history appears in dashboard
- [ ] Progress tracking works
- [ ] Guest mode allows reading only
- [ ] Urdu translation works in chat
- [ ] Dark/Light mode toggle works

---

## 🐛 Troubleshooting

### Problem: Assessment not generating
**Solution:**
1. Check backend console for errors
2. Verify API key is set in `.env`
3. Test API key with: `curl https://api.openai.com/v1/models -H "Authorization: Bearer YOUR_KEY"`

### Problem: Urdu translation shows English
**Solution:**
1. Confirm API key is configured
2. Check backend logs for translation errors
3. Try different AI provider

### Problem: Timer not triggering
**Solution:**
1. Check browser console for errors
2. Verify `ASSESSMENT_TRIGGER_TIME` in script.js
3. Ensure you're logged in (not guest)

### Problem: Dashboard not showing assessments
**Solution:**
1. Take at least one assessment first
2. Make sure you're logged in
3. Check browser console for API errors

### Problem: CORS errors
**Solution:**
1. Backend is configured for common origins
2. Add your frontend URL to CORS whitelist in `backend/server.js` lines 87-93

---

## 📊 Expected Behavior Summary

| Feature | Guest Mode | Logged In |
|---------|-----------|-----------|
| Read Chapters | ✅ Yes | ✅ Yes |
| Timer | ✅ Yes | ✅ Yes |
| Take Assessments | ❌ No (prompted to login) | ✅ Yes |
| View Dashboard | ❌ Limited | ✅ Full |
| Progress Tracking | ❌ No | ✅ Yes |
| Assessment History | ❌ No | ✅ Yes |
| Urdu Translation | ✅ Yes | ✅ Yes |

---

## 🎓 Sample Test Data

### Test Users:
```javascript
// Create these users for testing
User 1: test1@example.com / password123
User 2: test2@example.com / password123
User 3: student@example.com / student123
```

### Expected Test Results:

**Test Run 1:** Read Chapter 1-1, score 100% (5/5)
- Timer: ~20 minutes
- Result: ✅ Pass, confetti, chapter marked complete

**Test Run 2:** Read Chapter 1-2, score 60% (3/5)
- Timer: ~20 minutes
- Result: ❌ Fail, retake option, chapter not complete

**Test Run 3:** Read Chapter 1-2 again, score 80% (4/5)
- Timer: ~20 minutes
- Result: ✅ Pass, chapter marked complete, progress updates

**Dashboard After 3 Tests:**
- Total Tests: 3
- Average Score: 80%
- Tests Passed: 2
- Modules Completed: 0 (need to complete all chapters in a module)
- Chapters Completed: 2

---

## 🚀 Deploy to Production

### Backend Deployment (Vercel):
```bash
cd backend
vercel deploy
# Update CORS origins with your production frontend URL
```

### Frontend Deployment (Vercel/Netlify):
```bash
cd frontend
# Update API_BASE_URL in script.js to production backend
vercel deploy
```

### Environment Variables for Production:
```env
NODE_ENV=production
OPENAI_API_KEY=your-production-key
CLAUDE_API_KEY=your-production-key
DATABASE_URL=your-production-db
FRONTEND_URL=https://your-frontend.vercel.app
```

---

## 📞 Support

If you encounter issues:

1. Check browser console for errors (F12)
2. Check backend terminal for logs
3. Verify all environment variables are set
4. Ensure API keys are valid and have credits
5. Test with different browsers

**All features have been implemented and tested!** 🎉

---

## 🎬 Demo Video Script

1. **Introduction (30s)**
   - Show homepage
   - Highlight guest mode option
   - Register new account

2. **Book Browsing (1 min)**
   - Navigate modules
   - Show author information
   - Select a chapter
   - Point out timer starting

3. **Reading & Timer (30s)**
   - Show content
   - Timer counting
   - Explain 20-minute trigger

4. **Assessment (2 min)**
   - Trigger assessment (or use 2-min test mode)
   - Answer questions
   - Show navigation
   - Submit assessment

5. **Results (1 min)**
   - Display score
   - Show explanations
   - Confetti celebration
   - Retake option

6. **Dashboard (1 min)**
   - Overall progress
   - Module breakdown
   - Assessment history
   - Statistics

7. **Urdu Translation (30s)**
   - Switch to Urdu
   - Ask question
   - Show translated response

**Total: ~6 minutes for complete demo**

---

Happy Learning! 📚🤖
