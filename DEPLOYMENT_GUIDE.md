# Deployment Guide - Physical AI Robotics Platform

## Current Production Setup

### Backend
**URL**: https://backend-kgoj80t16-minahil-hamzas-projects.vercel.app
**Status**: ✅ Working

### Frontend (Latest)
**URL**: https://frontend-n91vc9je2-minahil-hamzas-projects.vercel.app
**Status**: ✅ Working with all features

### Frontend (Custom Domain)
**URL**: https://physical-ai-robotics-chi.vercel.app
**Status**: ⚠️ Needs update to connect to latest backend

---

## Fix Chatbot on physical-ai-robotics-chi.vercel.app

### Step 1: Add OpenAI API Key to Backend

1. **Go to Backend Settings**:
   - Visit: https://vercel.com/minahil-hamzas-projects/backend/settings/environment-variables

2. **Add OpenAI API Key**:
   - Click "Add New"
   - Name: `OPENAI_API_KEY`
   - Value: Your OpenAI API key (starts with `sk-proj-...` or `sk-...`)
   - Environment: Check all boxes (Production, Preview, Development)
   - Click "Save"

3. **Redeploy Backend**:
   - Go to: https://vercel.com/minahil-hamzas-projects/backend/deployments
   - Click "..." on latest deployment
   - Click "Redeploy"
   - Wait 30 seconds

### Step 2: Update Frontend Domain Alias

**Option A: Via Vercel Dashboard (Recommended)**

1. **Go to Frontend Project**:
   - Visit: https://vercel.com/minahil-hamzas-projects/frontend

2. **Add Domain**:
   - Click "Settings" → "Domains"
   - Add: `physical-ai-robotics-chi.vercel.app`
   - This will automatically use the latest deployment

**Option B: Redeploy to Specific Domain**

Run this command:
```bash
cd frontend
vercel --prod --name physical-ai-robotics
```

### Step 3: Verify Chatbot Works

1. Open: https://physical-ai-robotics-chi.vercel.app
2. Click "Start as Guest"
3. Type a question in the chatbot
4. Should get response from OpenAI GPT-4o

---

## Current Backend Configuration

### API Endpoints
- `/api/public/chat` - Guest chatbot (no auth required)
- `/api/chat` - Authenticated chatbot
- `/api/public/book-content` - Get book content (guest access)
- `/api/book-content` - Get book content (authenticated)

### AI Provider Fallback Order
1. **OpenAI GPT-4o** (Primary - needs API key)
2. **Claude 3.5 Sonnet** (Fallback - needs API key)
3. **Google Gemini** (Fallback - needs API key)
4. **Enhanced Fallback** (Knowledge base responses)

### Urdu Translation
Requires at least one AI API key to be configured.

---

## Environment Variables Needed

### Backend (.env or Vercel Environment Variables)

```env
# Required for Chatbot
OPENAI_API_KEY=sk-proj-your-key-here

# Optional - Additional AI Providers
CLAUDE_API_KEY=your-claude-key
GEMINI_API_KEY=your-gemini-key

# Database (Optional - works without it)
DATABASE_URL=postgresql://...

# Security
JWT_SECRET=your-secret-key
```

### Frontend
No environment variables needed - API URL is hardcoded in `script.js`

---

## Get OpenAI API Key

1. Visit: https://platform.openai.com/api-keys
2. Sign in or create account
3. Click "Create new secret key"
4. Copy the key (starts with `sk-proj-...`)
5. Add to Vercel backend environment variables

**Important**: Never commit API keys to git!

---

## Testing Checklist

- [ ] Backend health check: https://backend-kgoj80t16-minahil-hamzas-projects.vercel.app/health
- [ ] Guest chatbot works
- [ ] Urdu translation works (after adding API key)
- [ ] Book content loads (all 24 chapters)
- [ ] Dashboard shows progress
- [ ] Dark/Light mode toggle works
- [ ] GitHub link works
- [ ] Mark as complete works

---

## Troubleshooting

### Chatbot not responding
1. Check backend logs in Vercel
2. Verify OpenAI API key is added
3. Check browser console for errors
4. Ensure frontend is pointing to correct backend URL

### Book content not loading
1. Check CORS configuration in backend
2. Verify API endpoint is accessible
3. Check browser network tab

### Urdu not working
1. Add OpenAI API key (required for translation)
2. Redeploy backend after adding key
3. Wait 30 seconds for deployment

---

## Quick Commands

```bash
# Deploy frontend to production
cd frontend
vercel --prod

# Deploy backend to production
cd backend
vercel --prod

# Check backend status
curl https://backend-kgoj80t16-minahil-hamzas-projects.vercel.app/health

# View backend logs
vercel logs backend-kgoj80t16-minahil-hamzas-projects.vercel.app

# View frontend logs
vercel logs frontend-n91vc9je2-minahil-hamzas-projects.vercel.app
```

---

## Support

- GitHub Issues: https://github.com/Minahil-Hamza/Speckit-plus-humanoid-robotics-/issues
- Vercel Docs: https://vercel.com/docs
- OpenAI API Docs: https://platform.openai.com/docs
