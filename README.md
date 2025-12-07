# Physical AI & Humanoid Robotics - Educational Platform

A comprehensive educational platform for learning Physical AI and Humanoid Robotics, featuring interactive documentation built with Docusaurus and user authentication.

## Project Overview

This platform provides an in-depth learning experience for students interested in:
- **ROS 2 Fundamentals** - The robotic nervous system
- **Digital Twin Development** - Gazebo and Unity simulation
- **AI-Powered Robotics** - NVIDIA Isaac platform
- **Vision-Language-Action Models** - Cutting-edge embodied AI

## Features

### Educational Content
- 4 comprehensive modules covering ROS 2, Simulation, AI Perception, and VLA models
- Detailed lessons with code examples and explanations
- Hands-on projects and exercises
- Capstone project: Build an autonomous humanoid robot

### Authentication System
- User registration and login
- Secure JWT-based authentication
- Progress tracking (completed modules, bookmarks)
- User profiles and statistics
- Persistent sessions

### Modern Tech Stack
- **Frontend:** Docusaurus (React-based documentation framework)
- **Backend:** Node.js + Express + MongoDB
- **Authentication:** JWT tokens with bcrypt password hashing
- **Styling:** Custom CSS modules with responsive design

## Project Structure

```
book-hackathone/
├── physical-ai-robotics/          # Docusaurus frontend
│   ├── docs/                      # Documentation content
│   │   ├── intro.md
│   │   ├── modules/               # Course modules
│   │   │   ├── module-1-ros2/
│   │   │   ├── module-2-digital-twin/
│   │   │   ├── module-3-ai-robot-brain/
│   │   │   └── module-4-vla/
│   │   └── ...
│   ├── src/
│   │   ├── components/
│   │   │   └── Auth/              # Authentication components
│   │   │       ├── AuthContext.tsx
│   │   │       ├── LoginModal.tsx
│   │   │       ├── RegisterModal.tsx
│   │   │       └── AuthNavbarItem.tsx
│   │   ├── theme/
│   │   │   └── Root.tsx           # AuthProvider wrapper
│   │   └── pages/
│   ├── docusaurus.config.ts
│   ├── sidebars.ts
│   └── package.json
│
├── backend/                       # Express API backend
│   ├── models/
│   │   └── User.js                # User model
│   ├── routes/
│   │   ├── auth.js                # Authentication routes
│   │   └── users.js               # User management routes
│   ├── middleware/
│   │   └── auth.js                # JWT verification
│   ├── server.js                  # Main server file
│   ├── package.json
│   └── .env.example
│
└── README.md                      # This file
```

## Getting Started

### Prerequisites

Before you begin, ensure you have the following installed:
- **Node.js** (v18 or higher) - [Download](https://nodejs.org/)
- **MongoDB** (v4.4 or higher) - [Download](https://www.mongodb.com/try/download/community)
- **npm** or **yarn** package manager
- **Git** for version control

### Installation Steps

#### 1. Clone the Repository

```bash
git clone <repository-url>
cd book-hackathone
```

#### 2. Set Up the Backend

```bash
cd backend

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Edit .env and configure your settings
# At minimum, set JWT_SECRET to a secure random string
```

**Important:** Open `.env` and set your configuration:
```env
NODE_ENV=development
PORT=5000
MONGODB_URI=mongodb://localhost:27017/physical-ai-robotics
JWT_SECRET=your-super-secret-key-change-this
JWT_EXPIRE=30d
FRONTEND_URL=http://localhost:3000
```

**Start MongoDB:**
```bash
# Windows (if installed as service)
net start MongoDB

# macOS
brew services start mongodb-community

# Linux
sudo systemctl start mongod
```

**Start the backend server:**
```bash
npm run dev
```

The backend API should now be running on `http://localhost:5000`

#### 3. Set Up the Frontend

Open a new terminal window:

```bash
cd physical-ai-robotics

# Install dependencies
npm install

# Create environment file (optional)
echo "REACT_APP_API_URL=http://localhost:5000/api" > .env.local

# Start the development server
npm start
```

The Docusaurus site should open automatically at `http://localhost:3000`

### Integration with Docusaurus

To integrate the authentication components with Docusaurus navbar:

**Edit `docusaurus.config.ts`:**

```typescript
import type {Config} from '@docusaurus/types';

const config: Config = {
  // ... other config
  themeConfig: {
    navbar: {
      // ... other navbar items
      items: [
        // ... existing items
        {
          type: 'custom-authNavbarItem',  // Custom navbar item
          position: 'right',
        },
      ],
    },
  },
};
```

**Create a swizzled navbar component:**

```bash
npm run swizzle @docusaurus/theme-classic NavbarItem -- --wrap
```

Then modify `src/theme/NavbarItem/index.tsx`:

```typescript
import React from 'react';
import DefaultNavbarItem from '@theme-original/NavbarItem';
import AuthNavbarItem from '../../components/Auth/AuthNavbarItem';

export default function NavbarItem(props) {
  const { type } = props;

  if (type === 'custom-authNavbarItem') {
    return <AuthNavbarItem />;
  }

  return <DefaultNavbarItem {...props} />;
}
```

## Usage

### For Students

1. **Sign Up** - Click "Sign up" in the navbar to create an account
2. **Explore Modules** - Browse through the 4 modules on robotics
3. **Track Progress** - Your completed modules and bookmarks are automatically saved
4. **Complete Capstone** - Build an autonomous humanoid robot in Module 4

### For Developers

#### API Endpoints

All API endpoints are documented in `backend/README.md`. Key endpoints:

- `POST /api/auth/register` - Register new user
- `POST /api/auth/login` - Login user
- `GET /api/auth/me` - Get current user
- `POST /api/auth/logout` - Logout user
- `PUT /api/users/progress` - Update module progress
- `POST /api/users/bookmark` - Add/remove bookmarks

#### Adding New Content

1. Create a new markdown file in `physical-ai-robotics/docs/modules/`
2. Update `sidebars.ts` to include the new content
3. Commit and push your changes

#### Customizing Authentication

Modify the components in `src/components/Auth/` to customize:
- Login/register forms
- User menu
- Password requirements
- Profile fields

## Building for Production

### Backend

```bash
cd backend
NODE_ENV=production npm start
```

Consider using a process manager like PM2:
```bash
npm install -g pm2
pm2 start server.js --name robotics-api
```

### Frontend

```bash
cd physical-ai-robotics
npm run build
npm run serve
```

For deployment, the `build/` directory contains static files that can be hosted on:
- Vercel
- Netlify
- GitHub Pages
- AWS S3 + CloudFront

## Environment Variables

### Backend (.env)

| Variable | Description | Required |
|----------|-------------|----------|
| NODE_ENV | Environment mode | Yes |
| PORT | Server port | Yes |
| MONGODB_URI | MongoDB connection string | Yes |
| JWT_SECRET | Secret key for JWT signing | Yes |
| JWT_EXPIRE | Token expiration time | Yes |
| FRONTEND_URL | Frontend URL for CORS | Yes |

### Frontend (.env.local)

| Variable | Description | Required |
|----------|-------------|----------|
| REACT_APP_API_URL | Backend API URL | Yes |

## Database Schema

### User Collection

```javascript
{
  _id: ObjectId,
  name: String,
  email: String (unique, indexed),
  password: String (hashed),
  role: String (student/instructor/admin),
  progress: {
    completedModules: [{
      moduleId: String,
      completedAt: Date
    }],
    currentModule: String,
    bookmarks: [String]
  },
  createdAt: Date,
  lastLogin: Date
}
```

## Security Best Practices

- Passwords are hashed using bcrypt with 10 salt rounds
- JWT tokens expire after 30 days (configurable)
- All API endpoints validate input using express-validator
- Sensitive routes protected with authentication middleware
- CORS configured to only accept requests from frontend
- Environment variables used for all secrets

## Troubleshooting

### Backend won't start
- Verify MongoDB is running: `mongo --eval "db.stats()"`
- Check PORT is not in use: `netstat -ano | findstr :5000`
- Ensure `.env` file exists and has correct values

### Frontend can't connect to backend
- Verify `REACT_APP_API_URL` matches backend URL
- Check CORS settings in `backend/server.js`
- Ensure backend is running before starting frontend

### Authentication doesn't work
- Clear browser localStorage and cookies
- Verify JWT_SECRET is set and consistent
- Check network tab for API errors

## Contributing

This is an educational project. Contributions to improve content or fix issues are welcome!

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-module`)
3. Commit your changes (`git commit -am 'Add new module on X'`)
4. Push to the branch (`git push origin feature/new-module`)
5. Create a Pull Request

## Future Enhancements

- [ ] Email verification
- [ ] Password reset via email
- [ ] OAuth integration (Google, GitHub)
- [ ] Discussion forums
- [ ] Code playgrounds
- [ ] Video tutorials
- [ ] Quizzes and assessments
- [ ] Certificates of completion
- [ ] Admin dashboard

## License

This project is open-source and available under the MIT License.

## Support

For questions or issues:
- Open an issue on GitHub
- Check the documentation in `docs/`
- Review API documentation in `backend/README.md`

## Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- Powered by [ROS 2](https://docs.ros.org/en/humble/)
- Inspired by cutting-edge robotics research
- Content designed for hands-on learning

---

**Start your robotics journey today!** Sign up and explore the fascinating world of Physical AI and Humanoid Robotics.
