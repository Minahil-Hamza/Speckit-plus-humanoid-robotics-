// API Configuration
const API_BASE_URL = 'http://localhost:5001/api';

// Global variables
let currentUser = null;
let authToken = null;
let currentChapter = null;
let isGuestMode = false;

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    initializeThreeJS();
    checkAuthStatus();
});

// Initialize Three.js for 3D robot background
function initializeThreeJS() {
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });

    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setClearColor(0x000000, 0);

    document.getElementById('robot-scene').appendChild(renderer.domElement);

    // Create a simple robot
    const robotGroup = new THREE.Group();

    // Robot body
    const bodyGeometry = new THREE.BoxGeometry(1, 1.5, 0.8);
    const bodyMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ffff,
        wireframe: true
    });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    robotGroup.add(body);

    // Robot head
    const headGeometry = new THREE.BoxGeometry(0.6, 0.6, 0.6);
    const headMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ff88,
        wireframe: true
    });
    const head = new THREE.Mesh(headGeometry, headMaterial);
    head.position.y = 1.2;
    robotGroup.add(head);

    // Robot arms
    const armGeometry = new THREE.BoxGeometry(0.3, 0.8, 0.3);
    const armMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ffff,
        wireframe: true
    });

    const leftArm = new THREE.Mesh(armGeometry, armMaterial);
    leftArm.position.x = -0.7;
    leftArm.position.y = 0.3;
    robotGroup.add(leftArm);

    const rightArm = new THREE.Mesh(armGeometry, armMaterial);
    rightArm.position.x = 0.7;
    rightArm.position.y = 0.3;
    robotGroup.add(rightArm);

    // Robot legs
    const legGeometry = new THREE.BoxGeometry(0.4, 0.8, 0.4);
    const legMaterial = new THREE.MeshBasicMaterial({
        color: 0x00ff88,
        wireframe: true
    });

    const leftLeg = new THREE.Mesh(legGeometry, legMaterial);
    leftLeg.position.x = -0.3;
    leftLeg.position.y = -1.1;
    robotGroup.add(leftLeg);

    const rightLeg = new THREE.Mesh(legGeometry, legMaterial);
    rightLeg.position.x = 0.3;
    rightLeg.position.y = -1.1;
    robotGroup.add(rightLeg);

    scene.add(robotGroup);

    camera.position.z = 5;

    // Animation
    function animate() {
        requestAnimationFrame(animate);

        robotGroup.rotation.x += 0.01;
        robotGroup.rotation.y += 0.01;

        renderer.render(scene, camera);
    }

    animate();

    // Handle window resize
    window.addEventListener('resize', function() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });
}

// Switch between login and register tabs
function switchTab(tab) {
    const loginForm = document.getElementById('loginForm');
    const registerForm = document.getElementById('registerForm');
    const loginTab = document.getElementById('loginTab');
    const registerTab = document.getElementById('registerTab');

    if (tab === 'login') {
        loginForm.style.display = 'block';
        registerForm.style.display = 'none';
        loginTab.classList.add('active');
        registerTab.classList.remove('active');
    } else {
        registerForm.style.display = 'block';
        loginForm.style.display = 'none';
        registerTab.classList.add('active');
        loginTab.classList.remove('active');
    }
}

// Register function
async function register() {
    const name = document.getElementById('registerName').value;
    const email = document.getElementById('registerEmail').value;
    const password = document.getElementById('registerPassword').value;
    const confirmPassword = document.getElementById('registerConfirmPassword').value;
    const language = document.getElementById('registerLanguage').value;

    if (password !== confirmPassword) {
        alert('Passwords do not match!');
        return;
    }

    try {
        const response = await fetch(`${API_BASE_URL}/auth/register`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                name,
                email,
                password
            })
        });

        const data = await response.json();

        if (data.success) {
            alert('Registration successful! Please log in.');
            switchTab('login');
            // Store language preference
            localStorage.setItem('preferredLanguage', language);
        } else {
            alert('Registration failed: ' + (data.message || 'Unknown error'));
        }
    } catch (error) {
        console.error('Registration error:', error);
        alert('Registration failed: ' + error.message);
    }
}

// Login function
async function login() {
    const email = document.getElementById('loginEmail').value;
    const password = document.getElementById('loginPassword').value;
    const language = document.getElementById('languageSelect').value;

    try {
        const response = await fetch(`${API_BASE_URL}/auth/login`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                email,
                password
            })
        });

        const data = await response.json();

        if (data.success) {
            currentUser = data.data.user;
            authToken = data.data.token;

            // Store auth data
            localStorage.setItem('authToken', authToken);
            localStorage.setItem('currentUser', JSON.stringify(currentUser));
            localStorage.setItem('preferredLanguage', language);

            // Show main content and hide auth container
            document.getElementById('authContainer').style.display = 'none';
            document.getElementById('mainContent').style.display = 'block';

            // Update UI with user info
            document.getElementById('userName').textContent = currentUser.name;

            // Load book content and chapters
            loadBookContent();
            loadChapters();
            loadProgress();

            // Celebrate login
            confetti({
                particleCount: 100,
                spread: 70,
                origin: { y: 0.6 }
            });
        } else {
            alert('Login failed: ' + (data.message || 'Unknown error'));
        }
    } catch (error) {
        console.error('Login error:', error);
        alert('Login failed: ' + error.message);
    }
}

// Check if user is already authenticated
function checkAuthStatus() {
    const token = localStorage.getItem('authToken');
    const user = localStorage.getItem('currentUser');

    if (token && user) {
        authToken = token;
        currentUser = JSON.parse(user);

        // Show main content
        document.getElementById('authContainer').style.display = 'none';
        document.getElementById('mainContent').style.display = 'block';

        // Update UI with user info
        document.getElementById('userName').textContent = currentUser.name;

        // Load book content and chapters
        loadBookContent();
        loadChapters();
        loadProgress();
    }
    // If no token, stay on auth page (default behavior)
}

// Guest Mode - Start learning without registration
function startGuestMode() {
    isGuestMode = true;
    currentUser = { name: 'Guest User' };

    // Show main content and hide auth container
    document.getElementById('authContainer').style.display = 'none';
    document.getElementById('mainContent').style.display = 'block';

    // Update UI with guest info
    document.getElementById('userName').textContent = 'Guest User';

    // Load book content and chapters
    loadBookContent();
    loadChapters();

    // Celebrate guest access
    confetti({
        particleCount: 100,
        spread: 70,
        origin: { y: 0.6 }
    });
}

// Logout function
function logout() {
    const message = isGuestMode ? 'Exit guest mode?' : 'Are you sure you want to logout?';
    if (confirm(message)) {
        localStorage.removeItem('authToken');
        localStorage.removeItem('currentUser');
        localStorage.removeItem('preferredLanguage');

        currentUser = null;
        authToken = null;
        isGuestMode = false;

        // Show auth container and hide main content
        document.getElementById('authContainer').style.display = 'block';
        document.getElementById('mainContent').style.display = 'none';

        // Reset forms
        document.getElementById('loginForm').style.display = 'block';
        document.getElementById('registerForm').style.display = 'none';
        document.getElementById('loginTab').classList.add('active');
        document.getElementById('registerTab').classList.remove('active');

        // Clear form fields
        document.getElementById('loginEmail').value = '';
        document.getElementById('loginPassword').value = '';
        document.getElementById('registerName').value = '';
        document.getElementById('registerEmail').value = '';
        document.getElementById('registerPassword').value = '';
        document.getElementById('registerConfirmPassword').value = '';
    }
}

// Load book content
async function loadBookContent() {
    try {
        const response = await fetch(`${API_BASE_URL}/book-content`, {
            headers: {
                'Authorization': `Bearer ${authToken}`
            }
        });

        const data = await response.json();

        if (data.success) {
            // Update book content display (we'll update this when a chapter is selected)
            console.log('Book content loaded');
        } else {
            console.error('Failed to load book content:', data.message);
        }
    } catch (error) {
        console.error('Error loading book content:', error);
    }
}

// Load chapters list (now with modules)
async function loadChapters() {
    try {
        // Use public or protected endpoint based on mode
        const endpoint = isGuestMode ? `${API_BASE_URL}/public/book-content` : `${API_BASE_URL}/book-content`;
        const headers = isGuestMode ? {} : { 'Authorization': `Bearer ${authToken}` };

        const response = await fetch(endpoint, { headers });

        const data = await response.json();

        if (data.success) {
            const chaptersList = document.getElementById('chaptersList');
            chaptersList.innerHTML = '';

            // Display book title
            const bookTitle = document.createElement('h3');
            bookTitle.textContent = data.data.bookContent.title;
            bookTitle.style.cssText = 'color: #00ffff; margin-bottom: 20px; text-align: center;';
            chaptersList.appendChild(bookTitle);

            // Display modules and chapters
            data.data.bookContent.modules.forEach((module, moduleIndex) => {
                // Module header
                const moduleHeader = document.createElement('div');
                moduleHeader.className = 'module-header';
                moduleHeader.style.cssText = 'background: rgba(0, 255, 255, 0.1); padding: 10px; margin: 10px 0; border-left: 3px solid #00ffff; cursor: pointer;';
                moduleHeader.textContent = module.title;

                // Module description
                const moduleDesc = document.createElement('div');
                moduleDesc.style.cssText = 'font-size: 12px; color: #888; margin-left: 10px;';
                moduleDesc.textContent = module.description;
                moduleHeader.appendChild(moduleDesc);

                // Chapters container (initially hidden)
                const chaptersContainer = document.createElement('div');
                chaptersContainer.className = 'chapters-container';
                chaptersContainer.style.cssText = 'display: none; margin-left: 20px;';

                // Add chapters
                module.chapters.forEach((chapter, chapterIndex) => {
                    const chapterItem = document.createElement('div');
                    chapterItem.className = 'chapter-item';
                    chapterItem.style.cssText = 'padding: 8px; margin: 5px 0; cursor: pointer; border-left: 2px solid transparent; transition: all 0.3s;';
                    chapterItem.innerHTML = `
                        <div style="font-weight: bold; color: #00ff88;">${chapter.title}</div>
                        <div style="font-size: 11px; color: #666;">Reading time: ${chapter.readingTime || '15 min'}</div>
                    `;
                    chapterItem.onmouseenter = function() {
                        this.style.borderLeft = '2px solid #00ffff';
                        this.style.backgroundColor = 'rgba(0, 255, 255, 0.05)';
                    };
                    chapterItem.onmouseleave = function() {
                        this.style.borderLeft = '2px solid transparent';
                        this.style.backgroundColor = 'transparent';
                    };
                    chapterItem.onclick = () => selectChapter(chapter);
                    chaptersContainer.appendChild(chapterItem);
                });

                // Toggle chapters visibility on module click
                moduleHeader.onclick = () => {
                    if (chaptersContainer.style.display === 'none') {
                        chaptersContainer.style.display = 'block';
                        moduleHeader.style.background = 'rgba(0, 255, 255, 0.2)';
                    } else {
                        chaptersContainer.style.display = 'none';
                        moduleHeader.style.background = 'rgba(0, 255, 255, 0.1)';
                    }
                };

                chaptersList.appendChild(moduleHeader);
                chaptersList.appendChild(chaptersContainer);
            });
        } else {
            console.error('Failed to load chapters:', data.message);
        }
    } catch (error) {
        console.error('Error loading chapters:', error);
    }
}

// Simple markdown parser for chapter content
function parseMarkdown(text) {
    // Headers
    text = text.replace(/^### (.+)$/gm, '<h3>$1</h3>');
    text = text.replace(/^## (.+)$/gm, '<h2>$1</h2>');
    text = text.replace(/^# (.+)$/gm, '<h1>$1</h1>');

    // Bold
    text = text.replace(/\*\*(.+?)\*\*/g, '<strong>$1</strong>');

    // Code blocks
    text = text.replace(/```(\w+)?\n([\s\S]+?)```/g, '<pre><code>$2</code></pre>');

    // Inline code
    text = text.replace(/`([^`]+)`/g, '<code>$1</code>');

    // Lists
    text = text.replace(/^- (.+)$/gm, '<li>$1</li>');
    text = text.replace(/(<li>.*<\/li>\n?)+/g, '<ul>$&</ul>');

    // Paragraphs
    text = text.replace(/^(?!<[hul]|```|<pre)(.+)$/gm, '<p>$1</p>');

    return text;
}

// Select a chapter to display
async function selectChapter(chapter) {
    currentChapter = chapter;

    const bookContent = document.getElementById('bookContent');

    // Parse markdown content
    const htmlContent = parseMarkdown(chapter.content);

    bookContent.innerHTML = `
        <div style="margin-bottom: 20px;">
            <h2 style="color: #00ffff; margin-bottom: 10px;">${chapter.title}</h2>
            <div style="color: #888; font-size: 14px; margin-bottom: 20px;">
                <span>📚 Reading time: ${chapter.readingTime || '15 min'}</span>
                ${chapter.keywords ? `<span style="margin-left: 20px;">🏷️ ${chapter.keywords.join(', ')}</span>` : ''}
            </div>
        </div>
        <div class="chapter-content" style="line-height: 1.8; color: #ddd;">
            ${htmlContent}
        </div>
    `;

    // Style code blocks
    const codeBlocks = bookContent.querySelectorAll('pre code');
    codeBlocks.forEach(block => {
        block.style.cssText = 'background: rgba(0, 255, 255, 0.1); padding: 15px; border-radius: 5px; display: block; overflow-x: auto; color: #00ff88;';
        block.parentElement.style.cssText = 'background: rgba(0, 0, 0, 0.3); margin: 10px 0; border-left: 3px solid #00ffff;';
    });

    // Style inline code
    const inlineCodes = bookContent.querySelectorAll('code');
    inlineCodes.forEach(code => {
        if (!code.parentElement.tagName === 'PRE') {
            code.style.cssText = 'background: rgba(0, 255, 255, 0.2); padding: 2px 6px; border-radius: 3px; color: #00ffff; font-family: monospace;';
        }
    });

    // Mark chapter as viewed/started
    await updateProgress({ currentModule: chapter.id });

    // Scroll to top of content
    bookContent.scrollIntoView({ behavior: 'smooth' });
}

// Load user progress
async function loadProgress() {
    // Skip progress loading for guest users
    if (isGuestMode) {
        console.log('Guest mode - progress tracking disabled');
        return;
    }

    try {
        const response = await fetch(`${API_BASE_URL}/book-progress`, {
            headers: {
                'Authorization': `Bearer ${authToken}`
            }
        });

        const data = await response.json();

        if (data.success) {
            // Update chapter items to show completed status
            const chapterItems = document.querySelectorAll('.chapter-item');
            const completedModules = data.data.progress.completedModules;

            chapterItems.forEach(item => {
                const chapterTitle = item.textContent.replace(/^\d+\.\s*/, '');
                if (completedModules.includes(chapterTitle)) {
                    item.classList.add('completed');
                }
            });
        }
    } catch (error) {
        console.error('Error loading progress:', error);
    }
}

// Update user progress
async function updateProgress(progressData) {
    // Skip progress updates for guest users
    if (isGuestMode) {
        return;
    }

    try {
        const response = await fetch(`${API_BASE_URL}/book-progress`, {
            method: 'PUT',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${authToken}`
            },
            body: JSON.stringify(progressData)
        });

        const data = await response.json();

        if (data.success) {
            console.log('Progress updated successfully');
            loadProgress(); // Reload progress to update UI
        } else {
            console.error('Failed to update progress:', data.message);
        }
    } catch (error) {
        console.error('Error updating progress:', error);
    }
}

// Send message to AI chatbot
async function sendMessage() {
    const userInput = document.getElementById('userInput');
    const message = userInput.value.trim();

    if (!message) return;

    const chatMessages = document.getElementById('chatMessages');
    const language = document.getElementById('chatLanguage').value;

    // Add user message to chat
    const userMessageDiv = document.createElement('div');
    userMessageDiv.className = 'message user';
    userMessageDiv.innerHTML = `
        <div class="content">
            <p>${message}</p>
        </div>
        <div class="avatar">👤</div>
    `;
    chatMessages.appendChild(userMessageDiv);

    // Clear input
    userInput.value = '';

    // Show loading indicator
    const loadingDiv = document.createElement('div');
    loadingDiv.className = 'message bot';
    loadingDiv.innerHTML = `
        <div class="avatar">🤖</div>
        <div class="content">
            <p><span class="loading"></span> AI is thinking...</p>
        </div>
    `;
    chatMessages.appendChild(loadingDiv);

    // Scroll to bottom
    chatMessages.scrollTop = chatMessages.scrollHeight;

    try {
        // Use public or protected endpoint based on mode
        const endpoint = isGuestMode ? `${API_BASE_URL}/public/chat` : `${API_BASE_URL}/chat`;
        const headers = isGuestMode
            ? { 'Content-Type': 'application/json' }
            : { 'Content-Type': 'application/json', 'Authorization': `Bearer ${authToken}` };

        // Send message to backend API
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: headers,
            body: JSON.stringify({
                query: message,
                language: language
            })
        });

        const data = await response.json();

        // Remove loading indicator
        chatMessages.removeChild(loadingDiv);

        if (data.success) {
            // Add AI response to chat
            const aiMessageDiv = document.createElement('div');
            aiMessageDiv.className = 'message bot';
            aiMessageDiv.innerHTML = `
                <div class="avatar">🤖</div>
                <div class="content">
                    <p>${data.answer}</p>
                </div>
            `;
            chatMessages.appendChild(aiMessageDiv);
        } else {
            // Show error message
            const errorDiv = document.createElement('div');
            errorDiv.className = 'message bot';
            errorDiv.innerHTML = `
                <div class="avatar">🤖</div>
                <div class="content">
                    <p>Sorry, I encountered an error: ${data.message || 'Unknown error'}</p>
                </div>
            `;
            chatMessages.appendChild(errorDiv);
        }
    } catch (error) {
        // Remove loading indicator
        chatMessages.removeChild(loadingDiv);

        // Show error message
        const errorDiv = document.createElement('div');
        errorDiv.className = 'message bot';
        errorDiv.innerHTML = `
            <div class="avatar">🤖</div>
            <div class="content">
                <p>Sorry, I encountered an error: ${error.message}</p>
            </div>
        `;
        chatMessages.appendChild(errorDiv);
    }

    // Scroll to bottom
    chatMessages.scrollTop = chatMessages.scrollHeight;
}

// Handle Enter key press in chat input
function handleKeyPress(event) {
    if (event.key === 'Enter') {
        sendMessage();
    }
}

// Add some sample Three.js animations for interactive elements
function addInteractiveEffects() {
    // Add hover effects to buttons
    const buttons = document.querySelectorAll('button');
    buttons.forEach(button => {
        button.addEventListener('mouseenter', () => {
            button.style.transform = 'scale(1.05)';
            button.style.boxShadow = '0 0 20px rgba(0, 255, 255, 0.6)';
        });

        button.addEventListener('mouseleave', () => {
            button.style.transform = 'scale(1)';
            button.style.boxShadow = 'none';
        });
    });
}

// Initialize interactive effects
window.addEventListener('load', addInteractiveEffects);