# Physical AI & Robotics - Authentication Backend

This is the Node.js/Express backend API for user authentication and progress tracking in the Physical AI & Robotics educational platform.

## Features

- User registration and authentication with JWT
- Secure password hashing with bcrypt
- User progress tracking (module completion, bookmarks)
- Protected routes with middleware
- Input validation
- MongoDB database integration
- RESTful API design

## Tech Stack

- **Node.js** - Runtime environment
- **Express.js** - Web framework
- **MongoDB** - Database
- **Mongoose** - ODM
- **JWT** - Authentication
- **bcryptjs** - Password hashing
- **express-validator** - Input validation

## Prerequisites

- Node.js (v14 or higher)
- MongoDB (v4.4 or higher)
- npm or yarn package manager

## Installation

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Set up environment variables:**
   ```bash
   cp .env.example .env
   ```
   Then edit `.env` and configure your settings.

3. **Start MongoDB:**
   ```bash
   # On Windows (if installed as service)
   net start MongoDB

   # On macOS/Linux
   sudo systemctl start mongod
   ```

4. **Run the server:**
   ```bash
   # Development mode with auto-restart
   npm run dev

   # Production mode
   npm start
   ```

The server will start on `http://localhost:5000` (or your configured PORT).

## API Endpoints

### Authentication Routes (`/api/auth`)

#### Register User
```http
POST /api/auth/register
Content-Type: application/json

{
  "name": "John Doe",
  "email": "john@example.com",
  "password": "password123"
}
```

#### Login User
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "john@example.com",
  "password": "password123"
}
```

#### Get Current User
```http
GET /api/auth/me
Authorization: Bearer <your-jwt-token>
```

#### Logout
```http
POST /api/auth/logout
Authorization: Bearer <your-jwt-token>
```

#### Update Profile
```http
PUT /api/auth/update-profile
Authorization: Bearer <your-jwt-token>
Content-Type: application/json

{
  "name": "Jane Doe",
  "email": "jane@example.com"
}
```

#### Change Password
```http
PUT /api/auth/change-password
Authorization: Bearer <your-jwt-token>
Content-Type: application/json

{
  "currentPassword": "oldpassword",
  "newPassword": "newpassword123"
}
```

### User Routes (`/api/users`)

#### Update Progress
```http
PUT /api/users/progress
Authorization: Bearer <your-jwt-token>
Content-Type: application/json

{
  "moduleId": "module-1-ros2",
  "action": "complete"  // or "start"
}
```

#### Manage Bookmarks
```http
POST /api/users/bookmark
Authorization: Bearer <your-jwt-token>
Content-Type: application/json

{
  "pageUrl": "/docs/module-1/core-concepts",
  "action": "add"  // or "remove"
}
```

#### Get User Statistics
```http
GET /api/users/stats
Authorization: Bearer <your-jwt-token>
```

## Response Format

### Success Response
```json
{
  "success": true,
  "message": "Operation successful",
  "data": {
    "user": { ... },
    "token": "jwt-token"
  }
}
```

### Error Response
```json
{
  "success": false,
  "message": "Error description",
  "errors": [ ... ]  // Validation errors if any
}
```

## Database Schema

### User Model
```javascript
{
  name: String,
  email: String (unique),
  password: String (hashed),
  role: String (student/instructor/admin),
  avatar: String,
  progress: {
    completedModules: [{
      moduleId: String,
      completedAt: Date
    }],
    currentModule: String,
    bookmarks: [String]
  },
  isEmailVerified: Boolean,
  createdAt: Date,
  lastLogin: Date
}
```

## Security Features

1. **Password Hashing:** Passwords are hashed using bcrypt with salt rounds
2. **JWT Authentication:** Secure token-based authentication
3. **Input Validation:** All inputs are validated before processing
4. **CORS Protection:** Configured to only accept requests from frontend
5. **NoSQL Injection Prevention:** Mongoose sanitizes inputs
6. **Error Handling:** Sensitive error details hidden in production

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| NODE_ENV | Environment (development/production) | development |
| PORT | Server port | 5000 |
| MONGODB_URI | MongoDB connection string | mongodb://localhost:27017/physical-ai-robotics |
| JWT_SECRET | Secret key for JWT signing | (required) |
| JWT_EXPIRE | JWT expiration time | 30d |
| FRONTEND_URL | Frontend URL for CORS | http://localhost:3000 |

## Development

### Project Structure
```
backend/
├── models/
│   └── User.js           # User schema and model
├── routes/
│   ├── auth.js           # Authentication routes
│   └── users.js          # User management routes
├── middleware/
│   └── auth.js           # JWT verification middleware
├── server.js             # Main application file
├── package.json          # Dependencies
├── .env.example          # Environment variables template
└── README.md             # This file
```

### Running Tests
```bash
npm test
```

### Code Style
- Use async/await for asynchronous operations
- Follow RESTful API conventions
- Use descriptive variable and function names
- Add comments for complex logic

## Troubleshooting

### MongoDB Connection Error
- Ensure MongoDB is running: `sudo systemctl status mongod`
- Check connection string in `.env`
- Verify MongoDB port (default: 27017)

### JWT Token Invalid
- Check if JWT_SECRET matches between sessions
- Verify token hasn't expired
- Ensure Authorization header format: `Bearer <token>`

### CORS Errors
- Verify FRONTEND_URL in `.env` matches your frontend URL
- Check that credentials are enabled in frontend requests

## Future Enhancements

- [ ] Email verification
- [ ] Password reset functionality
- [ ] OAuth integration (Google, GitHub)
- [ ] Rate limiting
- [ ] Refresh tokens
- [ ] User roles and permissions
- [ ] Admin dashboard
- [ ] Analytics and logging

## License

MIT

## Support

For issues and questions, please open an issue on GitHub or contact the development team.
