// API Configuration
const API_BASE_URL = 'https://backend-minahil-hamzas-projects.vercel.app/api';

console.log('Script loaded! Version 2.1 - Author Bio Displayed!');
console.log('API URL:', API_BASE_URL);

// Global variables
let currentUser = null;
let authToken = null;
let currentChapter = null;
let isGuestMode = false;

// Initialize the application
document.addEventListener('DOMContentLoaded', function() {
    console.log('DOM Content Loaded - initializing application...');
    console.log('Checking if buttons exist...');
    console.log('Guest button:', document.querySelector('.guest-btn'));
    console.log('Login button:', document.querySelector('.auth-btn'));

    initializeThreeJS();
    checkAuthStatus();

    // Make functions globally accessible
    window.startGuestMode = startGuestMode;
    window.login = login;
    window.register = register;
    window.switchTab = switchTab;
    window.logout = logout;
    window.toggleTheme = toggleTheme;
    window.openDashboard = openDashboard;
    window.closeDashboard = closeDashboard;
    window.sendMessage = sendMessage;
    window.handleKeyPress = handleKeyPress;
    window.selectChapter = selectChapter;
    window.navigateToPreviousChapter = navigateToPreviousChapter;
    window.navigateToNextChapter = navigateToNextChapter;
    window.exitReadingMode = exitReadingMode;
    window.markChapterComplete = markChapterComplete;
    window.startQuiz = startQuiz;
    window.nextQuestion = nextQuestion;
    window.retakeQuiz = retakeQuiz;
    window.closeQuiz = closeQuiz;
    window.toggleChatbot = toggleChatbot;
    window.sendChatbotMessage = sendChatbotMessage;
    window.handleChatbotKeyPress = handleChatbotKeyPress;

    console.log('All functions attached to window object');
});

// Initialize Three.js for 3D robot background
function initializeThreeJS() {
    try {
        console.log('Initializing Three.js...');
        if (typeof THREE === 'undefined') {
            console.warn('THREE.js not loaded, skipping 3D background');
            return;
        }

        const robotSceneEl = document.getElementById('robot-scene');
        if (!robotSceneEl) {
            console.warn('robot-scene element not found');
            return;
        }

        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });

        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setClearColor(0x000000, 0);

        robotSceneEl.appendChild(renderer.domElement);

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
        console.log('Three.js initialized successfully');
    } catch (error) {
        console.error('Error initializing Three.js:', error);
        console.log('Continuing without 3D background');
    }
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
    try {
        console.log('✅ Register button clicked!');
        const name = document.getElementById('registerName').value;
        const email = document.getElementById('registerEmail').value;
        const password = document.getElementById('registerPassword').value;
        const confirmPassword = document.getElementById('registerConfirmPassword').value;
        const language = document.getElementById('registerLanguage').value;

        console.log('Form values:', { name, email, password: '***', language });

        if (!name || !email || !password) {
            alert('Please fill in all fields');
            return;
        }

        if (password !== confirmPassword) {
            alert('Passwords do not match!');
            return;
        }

        const response = await fetch(`${API_BASE_URL}/auth/register`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                name,
                email,
                password
            }),
            signal: AbortSignal.timeout(8000)
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

        // Show user-friendly message about backend unavailability
        const useGuest = confirm(
            '⚠️ Registration server is currently unavailable.\n\n' +
            'Would you like to continue as a Guest instead?\n\n' +
            '✓ Guest mode gives you full access to all 10 modules and 80 chapters\n' +
            '✓ AI chatbot with Urdu support\n' +
            '✓ Progress tracking (saved locally)\n\n' +
            'Click OK to start learning now, or Cancel to try registration later.'
        );

        if (useGuest) {
            startGuestMode();
        }
    }
}

// Login function
// Check if user is already authenticated
function checkAuthStatus() {
    try {
        console.log('Checking auth status...');
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

            // Show chatbot button
            const chatbotBtn = document.getElementById('chatbotFloatingBtn');
            if (chatbotBtn) chatbotBtn.style.display = 'flex';

            console.log('User auto-logged in');
        } else {
            console.log('No saved auth, showing login page');
        }
    } catch (error) {
        console.error('Error checking auth status:', error);
    }
}

// Guest Mode - Start learning without registration
function startGuestMode() {
    try {
        console.log('✅ Guest mode button clicked!');
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

        // Show chatbot button
        const chatbotBtn = document.getElementById('chatbotFloatingBtn');
        if (chatbotBtn) chatbotBtn.style.display = 'flex';

        // Celebrate guest access
        confetti({
            particleCount: 100,
            spread: 70,
            origin: { y: 0.6 }
        });
        console.log('✅ Guest mode activated successfully!');
    } catch (error) {
        console.error('❌ Error in startGuestMode:', error);
        alert('Error starting guest mode: ' + error.message);
    }
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

// Load book structure (metadata only, fast) with local fallback
async function loadBookContent() {
    try {
        // Try to load from backend API first
        const endpoint = isGuestMode ? `${API_BASE_URL}/public/book-structure` : `${API_BASE_URL}/book-structure`;
        const headers = isGuestMode ? {} : { 'Authorization': `Bearer ${authToken}` };

        const response = await fetch(endpoint, { headers, signal: AbortSignal.timeout(5000) });

        if (response.ok) {
            const data = await response.json();
            if (data.success) {
                console.log('Book structure loaded from backend');
                return;
            }
        }
        throw new Error('Backend not available');
    } catch (error) {
        console.log('Backend unavailable, using local metadata:', error.message);
        // Fallback to local JSON file
        try {
            const response = await fetch('./book_metadata.json');
            const metadata = await response.json();
            console.log('Book structure loaded from local file');
            // Store in global variable for other functions to use
            window.localBookMetadata = metadata;
        } catch (fallbackError) {
            console.error('Failed to load local metadata:', fallbackError);
        }
    }
}

// Load chapters list (now with modules) with local fallback
async function loadChapters() {
    try {
        let data = null;

        // Try to load from backend API first
        try {
            const endpoint = isGuestMode ? `${API_BASE_URL}/public/book-structure` : `${API_BASE_URL}/book-structure`;
            const headers = isGuestMode ? {} : { 'Authorization': `Bearer ${authToken}` };
            const response = await fetch(endpoint, { headers, signal: AbortSignal.timeout(5000) });

            if (response.ok) {
                data = await response.json();
            }
        } catch (backendError) {
            console.log('Backend unavailable, using local metadata');
        }

        // Fallback to local metadata if backend failed
        if (!data || !data.success) {
            if (window.localBookMetadata) {
                data = { success: true, data: { bookContent: window.localBookMetadata } };
            } else {
                // Load local JSON file
                const response = await fetch('./book_metadata.json');
                const metadata = await response.json();
                data = { success: true, data: { bookContent: metadata } };
                window.localBookMetadata = metadata;
            }
        }

        // Normalize data structure (backend may return data.data or data.data.bookContent)
        if (data.data && !data.data.bookContent) {
            data.data = { bookContent: data.data };
        }

        if (data.success) {
            const chaptersList = document.getElementById('chaptersList');
            chaptersList.innerHTML = '';

            // Display book title
            const bookTitle = document.createElement('h3');
            bookTitle.textContent = data.data.bookContent.title;
            bookTitle.style.cssText = 'color: #00ffff; margin-bottom: 20px; text-align: center; font-size: 1.3em;';
            chaptersList.appendChild(bookTitle);

            // Add book overview
            const overview = document.createElement('div');
            overview.className = 'book-overview';
            overview.innerHTML = `
                <div style="color: #00ff88; font-weight: bold; margin-bottom: 10px;">📚 Book Overview</div>
                <div class="overview-stats">
                    <div class="overview-stat">
                        <div class="overview-stat-value">${data.data.bookContent.totalModules}</div>
                        <div class="overview-stat-label">Modules</div>
                    </div>
                    <div class="overview-stat">
                        <div class="overview-stat-value">${data.data.bookContent.totalChapters}</div>
                        <div class="overview-stat-label">Chapters</div>
                    </div>
                    <div class="overview-stat">
                        <div class="overview-stat-value">${bookProgress.completedChapters.length}</div>
                        <div class="overview-stat-label">Completed</div>
                    </div>
                </div>
            `;
            chaptersList.appendChild(overview);

            // Add search bar
            const searchContainer = document.createElement('div');
            searchContainer.className = 'book-search';
            searchContainer.innerHTML = `
                <input type="text" id="chapterSearch" class="search-input" placeholder="Search chapters...">
                <span class="search-icon">🔍</span>
            `;
            chaptersList.appendChild(searchContainer);

            // Add collapse/expand all button
            const collapseBtn = document.createElement('button');
            collapseBtn.className = 'collapse-all-btn';
            collapseBtn.id = 'collapseAllBtn';
            collapseBtn.textContent = '▼ Collapse All Modules';
            collapseBtn.onclick = toggleAllModules;
            chaptersList.appendChild(collapseBtn);

            // Store modules data for search
            window.allModulesData = data.data.bookContent.modules;

            // Display author information
            if (data.data.bookContent.author) {
                const bookInfo = document.getElementById('bookInfo');
                const authorInfo = document.getElementById('authorInfo');
                if (bookInfo && authorInfo) {
                    authorInfo.innerHTML = `
                        <div style="margin-bottom: 10px; padding-bottom: 10px; border-bottom: 1px solid rgba(0,255,255,0.2);">
                            <strong style="color: #00ffff; font-size: 13px;">✍️ Author:</strong>
                            <span style="color: #fff; font-size: 13px;">${data.data.bookContent.author}</span>
                        </div>
                        ${data.data.bookContent.authorBio ? `
                            <div style="font-size: 12px; line-height: 1.6; color: #ccc; margin-bottom: 10px; text-align: justify;">
                                ${data.data.bookContent.authorBio}
                            </div>
                        ` : ''}
                        ${data.data.bookContent.publicationDate ? `
                            <div style="margin-top: 8px; font-size: 11px; color: #888;">
                                <strong style="color: #00ff88;">📅 Published:</strong> ${data.data.bookContent.publicationDate} |
                                <strong style="color: #00ff88;">📚 Version:</strong> ${data.data.bookContent.version}
                            </div>
                        ` : ''}
                        <div style="margin-top: 10px; padding-top: 10px; border-top: 1px solid rgba(0,255,255,0.2); font-size: 11px; color: #888;">
                            <strong style="color: #00ffff;">📖 Total:</strong> ${data.data.bookContent.totalModules} Modules | ${data.data.bookContent.totalChapters} Chapters
                        </div>
                    `;
                    bookInfo.style.display = 'block';
                }
            }

            // Display modules and chapters
            data.data.bookContent.modules.forEach((module, moduleIndex) => {
                // Calculate module progress
                const completedInModule = module.chapters.filter(ch =>
                    bookProgress.completedChapters.includes(ch.id)
                ).length;
                const progressPercent = Math.round((completedInModule / module.chapters.length) * 100);

                // Module header
                const moduleHeader = document.createElement('div');
                moduleHeader.className = 'module-header expanded';
                moduleHeader.style.cssText = 'background: rgba(0, 255, 255, 0.2); padding: 12px; margin: 10px 0; border-left: 3px solid #00ffff; cursor: pointer; border-radius: 8px;';
                moduleHeader.innerHTML = `
                    <div style="display: flex; align-items: center; justify-content: space-between;">
                        <div style="flex: 1;">
                            <span class="module-icon" style="margin-right: 10px;">▼</span>
                            <strong>${module.title}</strong>
                        </div>
                        <span style="color: #00ff88; font-size: 12px;">${completedInModule}/${module.chapters.length} ✓</span>
                    </div>
                    <div style="font-size: 12px; color: #888; margin-left: 28px; margin-top: 5px;">${module.description}</div>
                    <div class="module-progress-bar">
                        <div class="module-progress-fill" style="width: ${progressPercent}%"></div>
                    </div>
                `;

                // Chapters container (visible by default)
                const chaptersContainer = document.createElement('div');
                chaptersContainer.className = 'chapters-container';
                chaptersContainer.style.cssText = 'display: block; margin-left: 20px;';

                // Add chapters
                module.chapters.forEach((chapter, chapterIndex) => {
                    // Add moduleId to chapter object
                    chapter.moduleId = module.id;

                    const chapterItem = document.createElement('div');
                    chapterItem.className = 'chapter-item';
                    chapterItem.style.cssText = 'padding: 8px; margin: 5px 0; cursor: pointer; border-left: 2px solid transparent; transition: all 0.3s;';

                    // Check if chapter is completed
                    const isCompleted = bookProgress.completedChapters.includes(chapter.id);
                    const checkmark = isCompleted ? '<span style="color: #00ff88; margin-right: 5px;">✓</span>' : '';

                    chapterItem.innerHTML = `
                        <div style="font-weight: bold; color: #00ff88;">${checkmark}${chapter.title}</div>
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

    console.log(`[sendMessage] Language selected: ${language}`);
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

// Theme Toggle Functionality
function toggleTheme() {
    const body = document.body;
    const sunIcon = document.getElementById('sunIcon');
    const moonIcon = document.getElementById('moonIcon');

    body.classList.toggle('light-mode');

    // Toggle icons
    if (body.classList.contains('light-mode')) {
        sunIcon.style.display = 'none';
        moonIcon.style.display = 'block';
        localStorage.setItem('theme', 'light');
    } else {
        sunIcon.style.display = 'block';
        moonIcon.style.display = 'none';
        localStorage.setItem('theme', 'dark');
    }
}

// Load saved theme on page load
function loadSavedTheme() {
    const savedTheme = localStorage.getItem('theme');
    const body = document.body;
    const sunIcon = document.getElementById('sunIcon');
    const moonIcon = document.getElementById('moonIcon');

    if (savedTheme === 'light') {
        body.classList.add('light-mode');
        if (sunIcon && moonIcon) {
            sunIcon.style.display = 'none';
            moonIcon.style.display = 'block';
        }
    }
}

// Call on page load
window.addEventListener('load', loadSavedTheme);

// Progress tracking
let bookProgress = {
    completedChapters: [],
    moduleProgress: {},
    currentChapter: null,
    startTime: Date.now(),
    totalTimeSpent: 0,
    readingStartTime: null,
    readingTimeElapsed: 0,
    assessmentsTaken: []
};

// Reading timer variables
let readingTimer = null;
let assessmentTriggered = false;
const ASSESSMENT_TRIGGER_TIME = 20 * 60 * 1000; // 20 minutes in milliseconds

// Load progress from localStorage
function loadProgress() {
    const saved = localStorage.getItem('bookProgress');
    if (saved) {
        bookProgress = JSON.parse(saved);
    }
}

// Save progress to localStorage
function saveProgress() {
    localStorage.setItem('bookProgress', JSON.stringify(bookProgress));
}

// Mark chapter as complete
function markChapterComplete(chapterId, moduleId) {
    if (!bookProgress.completedChapters.includes(chapterId)) {
        bookProgress.completedChapters.push(chapterId);

        // Update module progress
        if (!bookProgress.moduleProgress[moduleId]) {
            bookProgress.moduleProgress[moduleId] = { completed: 0, total: 0 };
        }
        bookProgress.moduleProgress[moduleId].completed++;

        saveProgress();
        updateDashboard();
    }
}

// Select chapter and track as current
async function selectChapter(chapter) {
    // Stop previous timer if running
    if (readingTimer) {
        clearInterval(readingTimer);
    }
    assessmentTriggered = false;

    // Enter full-screen reading mode
    document.body.classList.add('reading-mode');

    // Start reading timer
    bookProgress.readingStartTime = Date.now();
    bookProgress.readingTimeElapsed = 0;

    // Display loading state
    const bookContent = document.getElementById('bookContent');
    bookContent.innerHTML = `
        <div style="text-align: center; padding: 100px 20px; color: #00ffff;">
            <div class="loading" style="margin: 20px auto;"></div>
            <h2>Loading Chapter...</h2>
            <p style="color: #888;">Please wait while we fetch the content</p>
        </div>
    `;

    try {
        // Fetch chapter content on-demand if not already loaded
        let chapterWithContent = chapter;
        if (!chapter.content) {
            console.log(`Fetching content for chapter: ${chapter.id}`);

            // Try backend API first
            try {
                const endpoint = isGuestMode ? `${API_BASE_URL}/public/chapter/${chapter.id}` : `${API_BASE_URL}/chapter/${chapter.id}`;
                const headers = isGuestMode ? {} : { 'Authorization': `Bearer ${authToken}` };

                const response = await fetch(endpoint, { headers, signal: AbortSignal.timeout(5000) });

                if (response.ok) {
                    const data = await response.json();
                    if (data.success) {
                        chapterWithContent = data.data.chapter || data.data;
                        console.log(`Chapter content loaded from backend: ${chapter.title}`);
                    }
                }
            } catch (backendError) {
                console.log('Backend unavailable for chapter, using local content');
            }

            // Fallback to local chapters file if backend failed
            if (!chapterWithContent.content) {
                if (!window.localChapters) {
                    const response = await fetch('./book_chapters.json');
                    window.localChapters = await response.json();
                }
                chapterWithContent = window.localChapters[chapter.id] || chapter;
                console.log(`Chapter content loaded from local file: ${chapter.title}`);
            }
        }

        // Display chapter content with timer
        const parsedContent = parseMarkdown(chapterWithContent.content);

        // Store chapter for navigation
        window.currentChapterData = chapterWithContent;

        bookContent.innerHTML = `
            <button class="back-to-library" onclick="exitReadingMode()">
                ← Back to Library
            </button>

            <div class="reading-progress">
                <div class="reading-progress-bar" id="readingProgressBar"></div>
            </div>

            <div class="chapter-header">
                <h1>${chapterWithContent.title}</h1>
                <div class="chapter-meta">
                    <span>📖 ${chapterWithContent.readingTime}</span>
                    <span id="readingTimerDisplay" style="margin-left: 20px; color: #00ffff;">⏱️ 00:00</span>
                    <button onclick="markChapterComplete('${chapterWithContent.id}', '${chapterWithContent.moduleId || 'unknown'}')" class="mark-complete-btn">
                        ${bookProgress.completedChapters.includes(chapterWithContent.id) ? '✅ Completed' : '✓ Mark as Complete'}
                    </button>
                </div>
            </div>
            <div class="chapter-content">${parsedContent}</div>

            <div class="chapter-navigation">
                <button class="nav-btn" id="prevChapterBtn" onclick="navigateToPreviousChapter()">
                    ← Previous Chapter
                </button>
                <button class="nav-btn" id="nextChapterBtn" onclick="navigateToNextChapter()">
                    Next Chapter →
                </button>
            </div>
        `;

        // Setup navigation buttons
        setupChapterNavigation(chapterWithContent);

        // Start timer
        startReadingTimer(chapterWithContent);

        // Track current chapter
        currentChapter = chapterWithContent;
        bookProgress.currentChapter = chapterWithContent.id;
        saveProgress();

        // Add quiz button
        addCompleteChapterButton();

        // Scroll to top
        window.scrollTo(0, 0);

        // Setup reading progress indicator
        setupReadingProgress();

        // Store chapter with content for future use
        chapter.content = chapterWithContent.content;

    } catch (error) {
        console.error('Error loading chapter:', error);
        bookContent.innerHTML = `
            <div style="text-align: center; padding: 100px 20px; color: #ff6b6b;">
                <h2>❌ Error Loading Chapter</h2>
                <p style="color: #888;">Failed to load chapter content. Please try again.</p>
                <button onclick="exitReadingMode()" class="back-to-library" style="margin-top: 20px;">
                    ← Back to Library
                </button>
            </div>
        `;
    }
}

// Start reading timer
function startReadingTimer(chapter) {
    readingTimer = setInterval(() => {
        const elapsed = Date.now() - bookProgress.readingStartTime;
        bookProgress.readingTimeElapsed = elapsed;

        // Update display
        const minutes = Math.floor(elapsed / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        const timerDisplay = document.getElementById('readingTimerDisplay');
        if (timerDisplay) {
            timerDisplay.textContent = `⏱️ ${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;

            // Change color as approaching 20 minutes
            if (elapsed >= ASSESSMENT_TRIGGER_TIME * 0.9) {
                timerDisplay.style.color = '#ff6b6b';
            } else if (elapsed >= ASSESSMENT_TRIGGER_TIME * 0.75) {
                timerDisplay.style.color = '#ffd93d';
            }
        }

        // Trigger assessment after 20 minutes
        if (elapsed >= ASSESSMENT_TRIGGER_TIME && !assessmentTriggered) {
            assessmentTriggered = true;
            clearInterval(readingTimer);
            triggerAssessment(chapter);
        }
    }, 1000);
}

// Trigger assessment
async function triggerAssessment(chapter) {
    if (isGuestMode) {
        alert('Assessment feature requires login. Please create an account to track your progress and take assessments!');
        return;
    }

    // Show notification
    const proceed = confirm(`Great job reading for 20 minutes! 📚\n\nTime to test your understanding with a quick assessment on:\n"${chapter.title}"\n\nReady to take the quiz?`);

    if (proceed) {
        await loadAssessment(chapter);
    } else {
        // Give option to take it later
        alert('No problem! You can take the assessment anytime from the chapter menu.');
    }
}

// Dashboard Functions
function openDashboard() {
    loadProgress();
    updateDashboard();
    document.getElementById('dashboardModal').style.display = 'flex';
}

function closeDashboard() {
    document.getElementById('dashboardModal').style.display = 'none';
}

// Close modal when clicking outside
window.addEventListener('click', (e) => {
    const modal = document.getElementById('dashboardModal');
    if (e.target === modal) {
        closeDashboard();
    }
});

// Update dashboard with current progress
async function updateDashboard() {
    try {
        let data = null;

        // Try to load from backend API first
        try {
            const endpoint = isGuestMode ? `${API_BASE_URL}/public/book-structure` : `${API_BASE_URL}/book-structure`;
            const headers = isGuestMode ? {} : { 'Authorization': `Bearer ${authToken}` };
            const response = await fetch(endpoint, { headers, signal: AbortSignal.timeout(5000) });
            if (response.ok) {
                data = await response.json();
            }
        } catch (backendError) {
            console.log('Backend unavailable for dashboard, using local metadata');
        }

        // Fallback to local metadata if backend failed
        if (!data || !data.success) {
            if (window.localBookMetadata) {
                data = { success: true, data: { bookContent: window.localBookMetadata } };
            } else {
                const response = await fetch('./book_metadata.json');
                const metadata = await response.json();
                data = { success: true, data: { bookContent: metadata } };
                window.localBookMetadata = metadata;
            }
        }

        // Normalize data structure
        if (data.data && !data.data.bookContent) {
            data.data = { bookContent: data.data };
        }

        if (!data.success) return;

        // Fetch assessment history if not guest
        let assessments = [];
        if (!isGuestMode) {
            try {
                const assessmentResponse = await fetch(`${API_BASE_URL}/assessments`, {
                    headers: { 'Authorization': `Bearer ${authToken}` }
                });
                const assessmentData = await assessmentResponse.json();
                if (assessmentData.success) {
                    assessments = assessmentData.assessments || [];
                }
            } catch (error) {
                console.error('Error fetching assessments:', error);
            }
        }

        const modules = data.data.bookContent.modules;
        const totalChapters = modules.reduce((sum, m) => sum + m.chapters.length, 0);
        const completedCount = bookProgress.completedChapters.length;
        const percentage = Math.round((completedCount / totalChapters) * 100);

        // Update progress circle
        const circumference = 339.292;
        const offset = circumference - (percentage / 100) * circumference;
        document.getElementById('progressCircle').style.strokeDashoffset = offset;
        document.getElementById('progressPercentage').textContent = `${percentage}%`;
        document.getElementById('chaptersCompleted').textContent = `${completedCount} of ${totalChapters} chapters completed`;

        // Update stats
        const completedModules = modules.filter(m => {
            const moduleChapters = m.chapters.map(c => c.id);
            return moduleChapters.every(id => bookProgress.completedChapters.includes(id));
        }).length;

        document.getElementById('completedModules').textContent = completedModules;
        document.getElementById('totalModules').textContent = modules.length;

        // Find current reading
        const currentChapter = bookProgress.currentChapter;
        if (currentChapter) {
            let currentTitle = 'Unknown';
            modules.forEach(m => {
                const chapter = m.chapters.find(c => c.id === currentChapter);
                if (chapter) currentTitle = chapter.title.substring(0, 20) + '...';
            });
            document.getElementById('currentReading').textContent = currentTitle;
        }

        // Calculate time spent (simplified)
        const hours = Math.floor((Date.now() - bookProgress.startTime) / (1000 * 60 * 60));
        document.getElementById('timeSpent').textContent = hours + 'h';

        // Update module progress list
        const moduleList = document.getElementById('moduleProgressList');
        moduleList.innerHTML = '';

        modules.forEach((module, index) => {
            const moduleChapters = module.chapters;
            const completedInModule = moduleChapters.filter(c =>
                bookProgress.completedChapters.includes(c.id)
            ).length;
            const modulePercentage = Math.round((completedInModule / moduleChapters.length) * 100);

            let status = 'not-started';
            let statusText = 'Not Started';
            if (modulePercentage === 100) {
                status = 'completed';
                statusText = 'Completed';
            } else if (modulePercentage > 0) {
                status = 'in-progress';
                statusText = 'In Progress';
            }

            const moduleItem = document.createElement('div');
            moduleItem.className = 'module-progress-item';
            moduleItem.innerHTML = `
                <div class="module-title">
                    <h4>${module.title}</h4>
                    <div class="module-status">
                        <span class="status-badge status-${status}">${statusText}</span>
                    </div>
                </div>
                <div class="progress-bar">
                    <div class="progress-fill" style="width: ${modulePercentage}%"></div>
                </div>
                <div class="chapter-count">${completedInModule} / ${moduleChapters.length} chapters completed</div>
            `;
            moduleList.appendChild(moduleItem);
        });

        // Add assessment section if user has taken assessments
        if (!isGuestMode && assessments.length > 0) {
            const assessmentSection = document.createElement('div');
            assessmentSection.className = 'assessment-history';
            assessmentSection.style.cssText = 'margin-top: 30px;';

            // Calculate average score
            const avgScore = Math.round(assessments.reduce((sum, a) => sum + a.score, 0) / assessments.length);
            const passedCount = assessments.filter(a => a.score >= 70).length;

            assessmentSection.innerHTML = `
                <h3 style="color: #00ffff; margin-bottom: 20px;">📊 Assessment History</h3>
                <div class="assessment-stats" style="display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 15px; margin-bottom: 20px;">
                    <div class="stat-box" style="background: rgba(0,255,255,0.1); padding: 15px; border-radius: 8px; text-align: center;">
                        <div style="font-size: 28px; color: #00ffff; font-weight: bold;">${assessments.length}</div>
                        <div style="color: #888; font-size: 14px;">Tests Taken</div>
                    </div>
                    <div class="stat-box" style="background: rgba(0,255,136,0.1); padding: 15px; border-radius: 8px; text-align: center;">
                        <div style="font-size: 28px; color: #00ff88; font-weight: bold;">${avgScore}%</div>
                        <div style="color: #888; font-size: 14px;">Average Score</div>
                    </div>
                    <div class="stat-box" style="background: rgba(0,255,255,0.1); padding: 15px; border-radius: 8px; text-align: center;">
                        <div style="font-size: 28px; color: #00ffff; font-weight: bold;">${passedCount}</div>
                        <div style="color: #888; font-size: 14px;">Tests Passed</div>
                    </div>
                </div>

                <div class="assessment-list" style="max-height: 400px; overflow-y: auto;">
                    ${assessments.slice().reverse().map(assessment => {
                        const date = new Date(assessment.completedAt);
                        const passed = assessment.score >= 70;
                        return `
                            <div class="assessment-item" style="background: rgba(255,255,255,0.05); padding: 15px; border-radius: 8px; margin-bottom: 10px; border-left: 4px solid ${passed ? '#00ff88' : '#ff6b6b'};">
                                <div style="display: flex; justify-content: space-between; align-items: center;">
                                    <div>
                                        <div style="font-weight: bold; color: ${passed ? '#00ff88' : '#ff6b6b'};">
                                            ${passed ? '✅' : '❌'} Score: ${assessment.score}%
                                        </div>
                                        <div style="color: #888; font-size: 12px; margin-top: 5px;">
                                            ${assessment.chapterId} • ${date.toLocaleDateString()} ${date.toLocaleTimeString()}
                                        </div>
                                    </div>
                                    <div style="text-align: right;">
                                        <div style="font-size: 14px; color: #00ffff;">
                                            ${assessment.correctAnswers}/${assessment.totalQuestions}
                                        </div>
                                        <div style="color: #888; font-size: 12px;">
                                            ${Math.floor(assessment.timeSpent / 60)}m ${assessment.timeSpent % 60}s
                                        </div>
                                    </div>
                                </div>
                            </div>
                        `;
                    }).join('')}
                </div>
            `;

            // Insert after module progress
            const modalBody = document.querySelector('.modal-body');
            if (modalBody) {
                const modulesProgress = modalBody.querySelector('.modules-progress');
                if (modulesProgress) {
                    modulesProgress.parentNode.insertBefore(assessmentSection, modulesProgress.nextSibling);
                }
            }
        }

    } catch (error) {
        console.error('Error updating dashboard:', error);
    }
}

// Load and render assessment
async function loadAssessment(chapter) {
    try {
        // Show loading
        const bookContent = document.getElementById('bookContent');
        bookContent.innerHTML = `
            <div style="text-align: center; padding: 50px;">
                <h2>🧠 Generating Assessment...</h2>
                <p>Creating personalized questions for you...</p>
                <div class="loading"></div>
            </div>
        `;

        // Fetch MCQs from API
        const response = await fetch(`${API_BASE_URL}/generate-mcqs`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${authToken}`
            },
            body: JSON.stringify({
                chapterId: chapter.id,
                chapterContent: chapter.content,
                numberOfQuestions: 5
            })
        });

        const data = await response.json();

        if (data.success) {
            renderAssessmentUI(chapter, data.mcqs);
        } else {
            throw new Error(data.message || 'Failed to load assessment');
        }
    } catch (error) {
        console.error('Error loading assessment:', error);
        alert('Failed to load assessment. Please try again later.');
        selectChapter(chapter); // Go back to chapter
    }
}

// Render assessment UI
function renderAssessmentUI(chapter, questions) {
    const bookContent = document.getElementById('bookContent');
    const assessmentStartTime = Date.now();

    let currentQuestionIndex = 0;
    let userAnswers = [];

    function renderQuestion() {
        const question = questions[currentQuestionIndex];
        const questionNumber = currentQuestionIndex + 1;

        bookContent.innerHTML = `
            <div class="assessment-container" style="max-width: 800px; margin: 0 auto;">
                <div class="assessment-header" style="background: rgba(0,255,255,0.1); padding: 20px; border-radius: 10px; margin-bottom: 30px;">
                    <h2>📝 Assessment: ${chapter.title}</h2>
                    <div style="display: flex; justify-content: space-between; margin-top: 10px; color: #888;">
                        <span>Question ${questionNumber} of ${questions.length}</span>
                        <span id="assessmentTimer">Time: 00:00</span>
                    </div>
                    <div class="progress-bar" style="margin-top: 15px; background: #333; height: 8px; border-radius: 4px; overflow: hidden;">
                        <div class="progress-fill" style="width: ${(questionNumber / questions.length) * 100}%; height: 100%; background: linear-gradient(90deg, #00ffff, #00ff88); transition: width 0.3s;"></div>
                    </div>
                </div>

                <div class="question-card" style="background: rgba(255,255,255,0.05); padding: 30px; border-radius: 10px; margin-bottom: 20px;">
                    <h3 style="color: #00ffff; margin-bottom: 25px; font-size: 20px;">${question.question}</h3>
                    <div class="options" style="display: flex; flex-direction: column; gap: 15px;">
                        ${Object.entries(question.options).map(([key, value]) => `
                            <button class="option-btn" data-answer="${key}" style="
                                padding: 15px 20px;
                                background: rgba(0,255,255,0.1);
                                border: 2px solid transparent;
                                border-radius: 8px;
                                color: #fff;
                                text-align: left;
                                cursor: pointer;
                                transition: all 0.3s;
                                font-size: 16px;
                            " onclick="selectAnswer('${key}')">
                                <strong>${key}.</strong> ${value}
                            </button>
                        `).join('')}
                    </div>
                </div>

                <div style="display: flex; justify-content: space-between; margin-top: 20px;">
                    <button onclick="previousQuestion()" ${currentQuestionIndex === 0 ? 'disabled' : ''} style="
                        padding: 12px 24px;
                        background: rgba(255,255,255,0.1);
                        border: 2px solid #00ffff;
                        border-radius: 5px;
                        color: #fff;
                        cursor: pointer;
                    ">← Previous</button>
                    <button id="nextBtn" onclick="nextQuestion()" disabled style="
                        padding: 12px 24px;
                        background: linear-gradient(135deg, #00ffff, #00ff88);
                        border: none;
                        border-radius: 5px;
                        color: #000;
                        font-weight: bold;
                        cursor: pointer;
                        opacity: 0.5;
                    ">${currentQuestionIndex === questions.length - 1 ? 'Submit Assessment' : 'Next →'}</button>
                </div>
            </div>
        `;

        // Restore previous answer if exists
        if (userAnswers[currentQuestionIndex]) {
            selectAnswer(userAnswers[currentQuestionIndex], false);
        }

        // Update timer
        const timerInterval = setInterval(() => {
            const elapsed = Date.now() - assessmentStartTime;
            const minutes = Math.floor(elapsed / 60000);
            const seconds = Math.floor((elapsed % 60000) / 1000);
            const timerEl = document.getElementById('assessmentTimer');
            if (timerEl) {
                timerEl.textContent = `Time: ${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`;
            }
        }, 1000);

        // Store timer for cleanup
        bookContent.dataset.timerInterval = timerInterval;
    }

    window.selectAnswer = function(answer, updateArray = true) {
        // Highlight selected option
        document.querySelectorAll('.option-btn').forEach(btn => {
            btn.style.border = '2px solid transparent';
            btn.style.background = 'rgba(0,255,255,0.1)';
        });

        const selectedBtn = document.querySelector(`.option-btn[data-answer="${answer}"]`);
        if (selectedBtn) {
            selectedBtn.style.border = '2px solid #00ffff';
            selectedBtn.style.background = 'rgba(0,255,255,0.2)';
        }

        // Save answer
        if (updateArray) {
            userAnswers[currentQuestionIndex] = answer;
        }

        // Enable next button
        const nextBtn = document.getElementById('nextBtn');
        nextBtn.disabled = false;
        nextBtn.style.opacity = '1';
    };

    window.previousQuestion = function() {
        if (currentQuestionIndex > 0) {
            currentQuestionIndex--;
            renderQuestion();
        }
    };

    window.nextQuestion = function() {
        if (!userAnswers[currentQuestionIndex]) {
            alert('Please select an answer before continuing.');
            return;
        }

        if (currentQuestionIndex < questions.length - 1) {
            currentQuestionIndex++;
            renderQuestion();
        } else {
            // Submit assessment
            submitAssessment(chapter, questions, userAnswers, assessmentStartTime);
        }
    };

    renderQuestion();
}

// Submit assessment
async function submitAssessment(chapter, questions, userAnswers, startTime) {
    try {
        // Calculate results
        const results = questions.map((q, index) => ({
            question: q.question,
            userAnswer: userAnswers[index],
            correctAnswer: q.correctAnswer,
            isCorrect: userAnswers[index] === q.correctAnswer,
            explanation: q.explanation
        }));

        const timeSpent = Math.round((Date.now() - startTime) / 1000); // in seconds

        // Send to API
        const response = await fetch(`${API_BASE_URL}/submit-assessment`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${authToken}`
            },
            body: JSON.stringify({
                chapterId: chapter.id,
                answers: results,
                totalQuestions: questions.length,
                timeSpent: timeSpent
            })
        });

        const data = await response.json();

        if (data.success) {
            showAssessmentResults(chapter, results, data.assessment, questions);
            // Update local progress
            if (data.assessment.passed && !bookProgress.completedChapters.includes(chapter.id)) {
                bookProgress.completedChapters.push(chapter.id);
                saveProgress();
            }
        } else {
            throw new Error(data.message || 'Failed to submit assessment');
        }
    } catch (error) {
        console.error('Error submitting assessment:', error);
        alert('Failed to submit assessment. Please try again.');
    }
}

// Show assessment results
function showAssessmentResults(chapter, results, assessment, questions) {
    const bookContent = document.getElementById('bookContent');

    const passed = assessment.passed;
    const score = assessment.score;

    // Celebration for passing
    if (passed) {
        confetti({
            particleCount: 100,
            spread: 70,
            origin: { y: 0.6 }
        });
    }

    bookContent.innerHTML = `
        <div class="assessment-results" style="max-width: 800px; margin: 0 auto;">
            <div class="results-header" style="text-align: center; padding: 40px; background: ${passed ? 'linear-gradient(135deg, rgba(0,255,136,0.2), rgba(0,255,255,0.2))' : 'rgba(255,107,107,0.2)'}; border-radius: 10px; margin-bottom: 30px;">
                <div style="font-size: 64px; margin-bottom: 20px;">${passed ? '🎉' : '📚'}</div>
                <h2 style="color: ${passed ? '#00ff88' : '#ff6b6b'}; margin-bottom: 10px;">
                    ${passed ? 'Congratulations!' : 'Keep Learning!'}
                </h2>
                <h1 style="font-size: 48px; margin: 20px 0;">Score: ${score}%</h1>
                <p style="font-size: 18px; color: #888;">
                    You got ${assessment.correctAnswers} out of ${assessment.totalQuestions} questions correct
                </p>
                ${passed
                    ? '<p style="color: #00ff88; margin-top: 10px;">✅ Chapter Completed!</p>'
                    : '<p style="color: #ffd93d; margin-top: 10px;">💪 You need 70% to pass. Try again!</p>'}
            </div>

            <div class="results-breakdown">
                <h3 style="color: #00ffff; margin-bottom: 20px;">📊 Detailed Results</h3>
                ${results.map((result, index) => `
                    <div class="result-card" style="background: rgba(255,255,255,0.05); padding: 20px; border-radius: 10px; margin-bottom: 15px; border-left: 4px solid ${result.isCorrect ? '#00ff88' : '#ff6b6b'};">
                        <div style="display: flex; align-items: center; gap: 10px; margin-bottom: 10px;">
                            <span style="font-size: 24px;">${result.isCorrect ? '✅' : '❌'}</span>
                            <strong>Question ${index + 1}:</strong>
                        </div>
                        <p style="color: #ddd; margin-bottom: 15px;">${result.question}</p>
                        <div style="margin-left: 20px;">
                            <p style="color: ${result.isCorrect ? '#00ff88' : '#ff6b6b'};">
                                <strong>Your answer:</strong> ${result.userAnswer}. ${questions[index].options[result.userAnswer]}
                            </p>
                            ${!result.isCorrect ? `
                                <p style="color: #00ff88;">
                                    <strong>Correct answer:</strong> ${result.correctAnswer}. ${questions[index].options[result.correctAnswer]}
                                </p>
                            ` : ''}
                            <p style="color: #00ffff; margin-top: 10px; padding: 10px; background: rgba(0,255,255,0.1); border-radius: 5px;">
                                💡 ${result.explanation}
                            </p>
                        </div>
                    </div>
                `).join('')}
            </div>

            <div style="display: flex; gap: 15px; justify-content: center; margin-top: 30px;">
                <button onclick="selectChapter(currentChapter)" style="
                    padding: 15px 30px;
                    background: rgba(0,255,255,0.2);
                    border: 2px solid #00ffff;
                    border-radius: 5px;
                    color: #fff;
                    cursor: pointer;
                    font-size: 16px;
                ">📖 Back to Chapter</button>
                ${!passed ? `
                    <button onclick="loadAssessment(currentChapter)" style="
                        padding: 15px 30px;
                        background: linear-gradient(135deg, #00ffff, #00ff88);
                        border: none;
                        border-radius: 5px;
                        color: #000;
                        font-weight: bold;
                        cursor: pointer;
                        font-size: 16px;
                    ">🔄 Retake Assessment</button>
                ` : ''}
                <button onclick="openDashboard()" style="
                    padding: 15px 30px;
                    background: linear-gradient(135deg, #00ffff, #00ff88);
                    border: none;
                    border-radius: 5px;
                    color: #000;
                    font-weight: bold;
                    cursor: pointer;
                    font-size: 16px;
                ">📊 View Dashboard</button>
            </div>
        </div>
    `;

    // Scroll to top
    bookContent.scrollTop = 0;
}

// Add manual assessment trigger button to chapters
function addAssessmentButton() {
    if (!isGuestMode && currentChapter) {
        const chapterMeta = document.querySelector('.chapter-meta');
        if (chapterMeta && !document.getElementById('takeAssessmentBtn')) {
            const assessmentBtn = document.createElement('button');
            assessmentBtn.id = 'takeAssessmentBtn';
            assessmentBtn.className = 'assessment-btn';
            assessmentBtn.textContent = '📝 Take Assessment';
            assessmentBtn.style.cssText = 'margin-left: 10px; padding: 8px 16px; background: linear-gradient(135deg, #00ffff, #00ff88); border: none; border-radius: 5px; color: #000; font-weight: bold; cursor: pointer;';
            assessmentBtn.onclick = () => loadAssessment(currentChapter);
            chapterMeta.appendChild(assessmentBtn);
        }
    }
}

// Initialize progress tracking
loadProgress();

// ========================== QUIZ SYSTEM ==========================

let currentQuiz = null;
let currentQuestionIndex = 0;
let quizAnswers = [];
let quizStartTime = null;

// Start quiz for a chapter
async function startQuiz(chapter) {
    console.log('Starting quiz for chapter:', chapter.title);

    // Show quiz modal with loading
    const quizModal = document.getElementById('quizModal');
    const quizBody = document.getElementById('quizBody');

    quizBody.innerHTML = `
        <div class="quiz-loading">
            <div class="quiz-loading-spinner"></div>
            <div class="quiz-loading-text">Generating quiz questions...</div>
        </div>
    `;

    quizModal.style.display = 'block';
    quizStartTime = Date.now();

    try {
        // Generate MCQs from backend
        const endpoint = isGuestMode ? `${API_BASE_URL}/public/chat` : `${API_BASE_URL}/generate-mcqs`;
        const headers = isGuestMode
            ? { 'Content-Type': 'application/json' }
            : { 'Content-Type': 'application/json', 'Authorization': `Bearer ${authToken}` };

        let response, data;

        if (isGuestMode) {
            // For guest mode, use chat to generate questions
            response = await fetch(endpoint, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    query: `Generate 5 multiple choice questions about: ${chapter.title}. Return only valid JSON with this format: {"questions":[{"question":"text","options":{"A":"opt1","B":"opt2","C":"opt3","D":"opt4"},"correctAnswer":"A","explanation":"why"}]}`,
                    language: 'english'
                })
            });
            data = await response.json();

            // Try to parse MCQs from chat response
            try {
                const jsonMatch = data.answer.match(/\{[\s\S]*\}/);
                if (jsonMatch) {
                    const parsed = JSON.parse(jsonMatch[0]);
                    currentQuiz = parsed.questions || generateFallbackQuiz(chapter);
                } else {
                    currentQuiz = generateFallbackQuiz(chapter);
                }
            } catch (e) {
                currentQuiz = generateFallbackQuiz(chapter);
            }
        } else {
            // For logged-in users, use dedicated MCQ endpoint
            response = await fetch(endpoint, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    chapterId: chapter.id,
                    chapterContent: chapter.content,
                    numberOfQuestions: 5
                })
            });
            data = await response.json();
            currentQuiz = data.success ? data.mcqs : generateFallbackQuiz(chapter);
        }

        currentQuestionIndex = 0;
        quizAnswers = [];

        displayQuizQuestion();

    } catch (error) {
        console.error('Error loading quiz:', error);
        currentQuiz = generateFallbackQuiz(chapter);
        currentQuestionIndex = 0;
        quizAnswers = [];
        displayQuizQuestion();
    }
}

// Generate fallback quiz questions
function generateFallbackQuiz(chapter) {
    return [
        {
            question: `What is the main topic covered in "${chapter.title}"?`,
            options: {
                A: "Understanding the fundamental concepts",
                B: "Advanced implementation details only",
                C: "Historical background only",
                D: "Future predictions"
            },
            correctAnswer: "A",
            explanation: "This chapter focuses on understanding the fundamental concepts of the topic."
        },
        {
            question: "Why is this topic important in robotics and AI?",
            options: {
                A: "It's not important",
                B: "It forms the foundation for advanced applications",
                C: "It's only for academic purposes",
                D: "It's outdated"
            },
            correctAnswer: "B",
            explanation: "This topic is crucial as it forms the foundation for more advanced applications."
        },
        {
            question: "What is a key takeaway from this chapter?",
            options: {
                A: "Memorize all definitions",
                B: "Understand core concepts and their applications",
                C: "Skip to advanced topics",
                D: "Focus only on theory"
            },
            correctAnswer: "B",
            explanation: "The key is to understand core concepts and how they apply in practice."
        },
        {
            question: "How should you approach learning this topic?",
            options: {
                A: "Rush through quickly",
                B: "Skip the examples",
                C: "Take time to understand and practice",
                D: "Only read the summary"
            },
            correctAnswer: "C",
            explanation: "The best approach is to take time to understand concepts and practice applying them."
        },
        {
            question: "What's the next step after completing this chapter?",
            options: {
                A: "Stop learning",
                B: "Move to the next chapter and build on this knowledge",
                C: "Repeat this chapter indefinitely",
                D: "Skip several chapters"
            },
            correctAnswer: "B",
            explanation: "The best approach is to build on this knowledge by progressing to the next chapter."
        }
    ];
}

// Display current quiz question
function displayQuizQuestion() {
    if (!currentQuiz || currentQuestionIndex >= currentQuiz.length) {
        showQuizResults();
        return;
    }

    const question = currentQuiz[currentQuestionIndex];
    const quizBody = document.getElementById('quizBody');

    const optionLetters = Object.keys(question.options);

    quizBody.innerHTML = `
        <div class="quiz-header">
            <h2>📝 Chapter Quiz</h2>
            <div class="quiz-info">
                <div class="quiz-info-item">
                    <span>📊</span>
                    <span>Question ${currentQuestionIndex + 1} of ${currentQuiz.length}</span>
                </div>
            </div>
        </div>

        <div class="quiz-question">
            <div class="question-number">Question ${currentQuestionIndex + 1}</div>
            <div class="question-text">${question.question}</div>

            <div class="quiz-options">
                ${optionLetters.map(letter => `
                    <div class="quiz-option" data-answer="${letter}" onclick="selectQuizOption('${letter}')">
                        <div class="option-letter">${letter}</div>
                        <div class="option-text">${question.options[letter]}</div>
                    </div>
                `).join('')}
            </div>
        </div>

        <div class="quiz-controls">
            <div class="quiz-progress">
                <strong>${quizAnswers.length}</strong> of ${currentQuiz.length} answered
            </div>
            <div>
                ${currentQuestionIndex > 0 ? '<button class="quiz-btn secondary" onclick="previousQuestion()">← Previous</button>' : ''}
                <button id="nextBtn" class="quiz-btn" onclick="nextQuestion()" disabled>
                    ${currentQuestionIndex < currentQuiz.length - 1 ? 'Next →' : 'Finish Quiz'}
                </button>
            </div>
        </div>
    `;
}

// Select quiz option
function selectQuizOption(letter) {
    // Remove previous selections
    document.querySelectorAll('.quiz-option').forEach(opt => {
        opt.classList.remove('selected');
    });

    // Mark selected option
    const selected = document.querySelector(`[data-answer="${letter}"]`);
    if (selected) {
        selected.classList.add('selected');
    }

    // Store answer
    quizAnswers[currentQuestionIndex] = letter;

    // Enable next button
    document.getElementById('nextBtn').disabled = false;
}

// Next question
function nextQuestion() {
    if (currentQuestionIndex < currentQuiz.length - 1) {
        currentQuestionIndex++;
        displayQuizQuestion();
    } else {
        showQuizResults();
    }
}

// Previous question
function previousQuestion() {
    if (currentQuestionIndex > 0) {
        currentQuestionIndex--;
        displayQuizQuestion();
    }
}

// Show quiz results
async function showQuizResults() {
    const quizBody = document.getElementById('quizBody');
    const quizEndTime = Date.now();
    const timeSpent = Math.round((quizEndTime - quizStartTime) / 1000); // seconds

    // Calculate score
    let correctCount = 0;
    currentQuiz.forEach((question, index) => {
        if (quizAnswers[index] === question.correctAnswer) {
            correctCount++;
        }
    });

    const scorePercentage = Math.round((correctCount / currentQuiz.length) * 100);
    const passed = scorePercentage >= 70;

    // Confetti for passing
    if (passed) {
        confetti({
            particleCount: 150,
            spread: 100,
            origin: { y: 0.6 }
        });
    }

    quizBody.innerHTML = `
        <div class="quiz-results">
            <h2 style="color: #00ffff; font-size: 2em; margin-bottom: 20px;">Quiz Complete!</h2>

            <div class="results-score">${scorePercentage}%</div>

            <div class="results-message ${passed ? 'pass' : 'fail'}">
                ${passed
                    ? '🎉 Excellent! You passed the quiz!'
                    : '📚 Keep studying! Try again to improve your score.'}
            </div>

            <div class="results-stats">
                <div class="result-stat">
                    <div class="result-stat-value">${correctCount}</div>
                    <div class="result-stat-label">Correct</div>
                </div>
                <div class="result-stat">
                    <div class="result-stat-value">${currentQuiz.length - correctCount}</div>
                    <div class="result-stat-label">Incorrect</div>
                </div>
                <div class="result-stat">
                    <div class="result-stat-value">${currentQuiz.length}</div>
                    <div class="result-stat-label">Total</div>
                </div>
                <div class="result-stat">
                    <div class="result-stat-value">${Math.floor(timeSpent / 60)}:${(timeSpent % 60).toString().padStart(2, '0')}</div>
                    <div class="result-stat-label">Time</div>
                </div>
            </div>

            <div style="margin-top: 30px; text-align: left;">
                <h3 style="color: #00ff88; margin-bottom: 15px;">Review Answers:</h3>
                ${currentQuiz.map((q, i) => {
                    const userAnswer = quizAnswers[i];
                    const isCorrect = userAnswer === q.correctAnswer;
                    return `
                        <div style="background: rgba(0,255,255,0.05); border-left: 4px solid ${isCorrect ? '#00ff88' : '#ff6b6b'}; padding: 15px; margin: 10px 0; border-radius: 8px;">
                            <div style="color: #00ffff; font-weight: bold; margin-bottom: 8px;">Q${i + 1}: ${q.question}</div>
                            <div style="color: ${isCorrect ? '#00ff88' : '#ff6b6b'}; margin-bottom: 5px;">
                                Your answer: ${userAnswer ? userAnswer + '. ' + q.options[userAnswer] : 'Not answered'}
                                ${isCorrect ? '✓' : '✗'}
                            </div>
                            ${!isCorrect ? `<div style="color: #00ff88; margin-bottom: 5px;">Correct answer: ${q.correctAnswer}. ${q.options[q.correctAnswer]}</div>` : ''}
                            <div style="color: #888; font-size: 0.9em; margin-top: 5px;">💡 ${q.explanation}</div>
                        </div>
                    `;
                }).join('')}
            </div>

            <div class="results-actions">
                <button class="quiz-btn" onclick="retakeQuiz()">🔄 Retake Quiz</button>
                <button class="quiz-btn secondary" onclick="closeQuiz()">Close</button>
            </div>
        </div>
    `;

    // Save assessment if logged in
    if (!isGuestMode && currentChapter) {
        try {
            await fetch(`${API_BASE_URL}/submit-assessment`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${authToken}`
                },
                body: JSON.stringify({
                    chapterId: currentChapter.id,
                    answers: quizAnswers.map((answer, index) => ({
                        question: currentQuiz[index].question,
                        userAnswer: answer,
                        correctAnswer: currentQuiz[index].correctAnswer,
                        isCorrect: answer === currentQuiz[index].correctAnswer
                    })),
                    totalQuestions: currentQuiz.length,
                    timeSpent: timeSpent
                })
            });

            // Reload progress to reflect completion
            if (passed) {
                loadProgress();
            }
        } catch (error) {
            console.error('Error saving assessment:', error);
        }
    }
}

// Retake quiz
function retakeQuiz() {
    if (currentChapter) {
        startQuiz(currentChapter);
    }
}

// Close quiz
function closeQuiz() {
    document.getElementById('quizModal').style.display = 'none';
    currentQuiz = null;
    currentQuestionIndex = 0;
    quizAnswers = [];
}

// Add "Complete & Take Quiz" button to chapter display
function addCompleteChapterButton() {
    const bookContent = document.getElementById('bookContent');

    // Remove existing button if any
    const existingBtn = document.getElementById('completeChapterBtn');
    if (existingBtn) {
        existingBtn.remove();
    }

    if (currentChapter) {
        const completeBtn = document.createElement('button');
        completeBtn.id = 'completeChapterBtn';
        completeBtn.className = 'complete-chapter-btn';
        completeBtn.innerHTML = '✓ Complete Chapter & Take Quiz';
        completeBtn.onclick = () => startQuiz(currentChapter);

        bookContent.appendChild(completeBtn);
    }
}

// ========================== ENHANCED NAVIGATION ==========================

// Toggle all modules collapse/expand
function toggleAllModules() {
    const collapseBtn = document.getElementById('collapseAllBtn');
    const chaptersContainers = document.querySelectorAll('.chapters-container');
    const moduleHeaders = document.querySelectorAll('.module-header');

    const allExpanded = Array.from(chaptersContainers).some(container => container.style.display === 'block');

    chaptersContainers.forEach((container, index) => {
        if (allExpanded) {
            container.style.display = 'none';
            moduleHeaders[index].style.background = 'rgba(0, 255, 255, 0.1)';
            moduleHeaders[index].classList.remove('expanded');
        } else {
            container.style.display = 'block';
            moduleHeaders[index].style.background = 'rgba(0, 255, 255, 0.2)';
            moduleHeaders[index].classList.add('expanded');
        }
    });

    collapseBtn.textContent = allExpanded ? '▶ Expand All Modules' : '▼ Collapse All Modules';
}

// Search chapters
document.addEventListener('DOMContentLoaded', () => {
    // Setup search with debounce
    let searchTimeout;
    document.addEventListener('input', (e) => {
        if (e.target && e.target.id === 'chapterSearch') {
            clearTimeout(searchTimeout);
            searchTimeout = setTimeout(() => {
                const searchTerm = e.target.value.toLowerCase();
                filterChapters(searchTerm);
            }, 300);
        }
    });
});

function filterChapters(searchTerm) {
    const moduleHeaders = document.querySelectorAll('.module-header');
    const chaptersContainers = document.querySelectorAll('.chapters-container');

    if (!searchTerm) {
        // Show all modules and chapters
        moduleHeaders.forEach(header => header.style.display = 'block');
        chaptersContainers.forEach(container => {
            container.style.display = 'block';
            const chapters = container.querySelectorAll('.chapter-item');
            chapters.forEach(chapter => chapter.style.display = 'block');
        });
        return;
    }

    moduleHeaders.forEach((header, index) => {
        const chaptersContainer = chaptersContainers[index];
        const chapters = chaptersContainer.querySelectorAll('.chapter-item');
        let hasVisibleChapter = false;

        // Check if module title matches
        const moduleMatches = header.textContent.toLowerCase().includes(searchTerm);

        chapters.forEach(chapter => {
            const chapterText = chapter.textContent.toLowerCase();
            if (chapterText.includes(searchTerm) || moduleMatches) {
                chapter.style.display = 'block';
                hasVisibleChapter = true;
            } else {
                chapter.style.display = 'none';
            }
        });

        // Show/hide module based on matches
        if (hasVisibleChapter) {
            header.style.display = 'block';
            chaptersContainer.style.display = 'block';
            header.style.background = 'rgba(0, 255, 255, 0.2)';
        } else {
            header.style.display = 'none';
        }
    });
}

// Scroll to top button (add to HTML later)
const scrollToTopBtn = document.createElement('div');
scrollToTopBtn.className = 'scroll-to-top';
scrollToTopBtn.innerHTML = '↑';
scrollToTopBtn.onclick = () => {
    const chaptersList = document.getElementById('chaptersList');
    if (chaptersList) {
        chaptersList.scrollTop = 0;
    }
};
document.body.appendChild(scrollToTopBtn);

// Show/hide scroll button based on scroll position
const chaptersList = document.getElementById('chaptersList');
if (chaptersList) {
    chaptersList.addEventListener('scroll', () => {
        if (chaptersList.scrollTop > 300) {
            scrollToTopBtn.classList.add('visible');
        } else {
            scrollToTopBtn.classList.remove('visible');
        }
    });
}

// ========================== FULL-SCREEN READING MODE ==========================

// Exit reading mode and return to library
function exitReadingMode() {
    document.body.classList.remove('reading-mode');
    window.scrollTo(0, 0);

    // Clear reading timer
    if (readingTimer) {
        clearInterval(readingTimer);
    }

    // Show welcome message
    const bookContent = document.getElementById('bookContent');
    bookContent.innerHTML = `
        <div style="text-align: center; padding: 100px 20px;">
            <h1 style="color: #00ffff; font-size: 3em; margin-bottom: 20px;">📚 Welcome to Your Library</h1>
            <p style="color: #888; font-size: 1.2em;">Select a chapter from the sidebar to start reading</p>
            <div style="margin-top: 40px;">
                <p style="color: #00ff88; font-size: 1.1em;">
                    ✓ 80 Comprehensive Chapters<br>
                    ✓ 10 Professional Modules<br>
                    ✓ Interactive Quizzes<br>
                    ✓ Progress Tracking
                </p>
            </div>
        </div>
    `;
}

// Get all chapters from all modules
function getAllChapters() {
    if (!window.allModulesData) return [];

    const chapters = [];
    window.allModulesData.forEach(module => {
        module.chapters.forEach(chapter => {
            chapters.push({
                ...chapter,
                moduleId: module.id,
                moduleTitle: module.title
            });
        });
    });
    return chapters;
}

// Setup chapter navigation buttons
function setupChapterNavigation(currentChapter) {
    const allChapters = getAllChapters();
    const currentIndex = allChapters.findIndex(ch => ch.id === currentChapter.id);

    const prevBtn = document.getElementById('prevChapterBtn');
    const nextBtn = document.getElementById('nextChapterBtn');

    // Store navigation data
    window.previousChapter = currentIndex > 0 ? allChapters[currentIndex - 1] : null;
    window.nextChapter = currentIndex < allChapters.length - 1 ? allChapters[currentIndex + 1] : null;

    // Update button states
    if (prevBtn) {
        if (window.previousChapter) {
            prevBtn.disabled = false;
            prevBtn.innerHTML = `← Previous Chapter<br><span style="font-size: 12px; opacity: 0.7;">${window.previousChapter.title}</span>`;
        } else {
            prevBtn.disabled = true;
            prevBtn.innerHTML = '← Previous Chapter';
        }
    }

    if (nextBtn) {
        if (window.nextChapter) {
            nextBtn.disabled = false;
            nextBtn.innerHTML = `Next Chapter →<br><span style="font-size: 12px; opacity: 0.7;">${window.nextChapter.title}</span>`;
        } else {
            nextBtn.disabled = true;
            nextBtn.innerHTML = 'Next Chapter →';
        }
    }
}

// Navigate to previous chapter
function navigateToPreviousChapter() {
    if (window.previousChapter) {
        selectChapter(window.previousChapter);
    }
}

// Navigate to next chapter
function navigateToNextChapter() {
    if (window.nextChapter) {
        selectChapter(window.nextChapter);
    }
}

// Setup reading progress indicator
function setupReadingProgress() {
    window.addEventListener('scroll', updateReadingProgress);
    updateReadingProgress(); // Initial call
}

function updateReadingProgress() {
    const progressBar = document.getElementById('readingProgressBar');
    if (!progressBar) return;

    const windowHeight = window.innerHeight;
    const documentHeight = document.documentElement.scrollHeight;
    const scrollTop = window.pageYOffset || document.documentElement.scrollTop;

    const scrollPercent = (scrollTop / (documentHeight - windowHeight)) * 100;
    progressBar.style.width = Math.min(scrollPercent, 100) + '%';
}

// Fix login - make sure languageSelect exists or use default
async function login() {
    console.log('Login button clicked!');
    const email = document.getElementById('loginEmail').value;
    const password = document.getElementById('loginPassword').value;
    const languageSelect = document.getElementById('languageSelect');
    const language = languageSelect ? languageSelect.value : 'english';

    if (!email || !password) {
        alert('Please enter both email and password');
        return;
    }

    try {
        // Try to login with backend (with timeout)
        const response = await fetch(`${API_BASE_URL}/auth/login`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                email,
                password
            }),
            signal: AbortSignal.timeout(8000)
        });

        const data = await response.json();

        if (data.success) {
            currentUser = data.data.user;
            authToken = data.data.token;
            isGuestMode = false;

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

            // Show chatbot button
            const chatbotBtn = document.getElementById('chatbotFloatingBtn');
            if (chatbotBtn) chatbotBtn.style.display = 'flex';

            // Celebrate login
            confetti({
                particleCount: 100,
                spread: 70,
                origin: { y: 0.6 }
            });
        } else {
            alert('Login failed: ' + (data.message || 'Invalid email or password'));
        }
    } catch (error) {
        console.error('Login error:', error);

        // Show user-friendly message about backend unavailability
        const useGuest = confirm(
            '⚠️ Authentication server is currently unavailable.\n\n' +
            'Would you like to continue as a Guest instead?\n\n' +
            '✓ Guest mode gives you full access to all 10 modules and 80 chapters\n' +
            '✓ AI chatbot with Urdu support\n' +
            '✓ Progress tracking (saved locally)\n\n' +
            'Click OK to start learning now, or Cancel to try login later.'
        );

        if (useGuest) {
            startGuestMode();
        }
    }
}
// ========================================
// FLOATING CHATBOT FUNCTIONS
// ========================================

// Toggle chatbot popup
function toggleChatbot() {
    const chatbotModal = document.getElementById('chatbotModal');
    const isVisible = chatbotModal.style.display !== 'none';
    chatbotModal.style.display = isVisible ? 'none' : 'block';

    // Focus on input when opening
    if (!isVisible) {
        setTimeout(() => {
            document.getElementById('chatbotInput').focus();
        }, 100);
    }
}

// Send chatbot message
async function sendChatbotMessage() {
    const input = document.getElementById('chatbotInput');
    const message = input.value.trim();

    if (!message) return;

    const language = document.getElementById('chatbotLanguage').value;
    const messagesContainer = document.getElementById('chatbotMessages');

    // Add user message to chat
    const userMessageDiv = document.createElement('div');
    userMessageDiv.className = 'message user';
    userMessageDiv.innerHTML = `
        <div class="user-avatar">👤</div>
        <div class="message-content">
            <p>${message}</p>
        </div>
    `;
    messagesContainer.appendChild(userMessageDiv);

    // Clear input
    input.value = '';

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    // Add typing indicator
    const typingDiv = document.createElement('div');
    typingDiv.className = 'message bot typing-indicator';
    typingDiv.id = 'typingIndicator';
    typingDiv.innerHTML = `
        <div class="bot-avatar">🤖</div>
        <div class="message-content">
            <p>Typing...</p>
        </div>
    `;
    messagesContainer.appendChild(typingDiv);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    try {
        let botResponse = null;

        // Try backend API first with timeout
        try {
            const endpoint = isGuestMode ? `${API_BASE_URL}/public/chat` : `${API_BASE_URL}/chat`;
            const headers = {
                'Content-Type': 'application/json'
            };

            if (!isGuestMode && authToken) {
                headers['Authorization'] = `Bearer ${authToken}`;
            }

            const response = await fetch(endpoint, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    query: message,
                    language: language,
                    context: currentChapter ? currentChapter.title : 'General'
                }),
                signal: AbortSignal.timeout(8000)
            });

            if (response.ok) {
                const data = await response.json();
                botResponse = data.answer;
            }
        } catch (backendError) {
            console.log('Backend unavailable for chat, using local chatbot');
        }

        // Fallback to local chatbot if backend failed
        if (!botResponse) {
            botResponse = getLocalChatbotResponse(message, language);
        }

        // Remove typing indicator
        const typing = document.getElementById('typingIndicator');
        if (typing) typing.remove();

        // Add bot response
        const botMessageDiv = document.createElement('div');
        botMessageDiv.className = 'message bot';
        botMessageDiv.innerHTML = `
            <div class="bot-avatar">🤖</div>
            <div class="message-content">
                <p>${botResponse}</p>
            </div>
        `;
        messagesContainer.appendChild(botMessageDiv);

        // Scroll to bottom
        messagesContainer.scrollTop = messagesContainer.scrollHeight;

    } catch (error) {
        console.error('Chat error:', error);

        // Remove typing indicator
        const typing = document.getElementById('typingIndicator');
        if (typing) typing.remove();

        // Add error message
        const errorMessageDiv = document.createElement('div');
        errorMessageDiv.className = 'message bot';
        const errorMsg = language === 'urdu'
            ? 'معذرت، میں ابھی آپ کی مدد نہیں کر سکتا۔ براہ کرم دوبارہ کوشش کریں۔'
            : 'I\'m sorry, I\'m having trouble right now. Please try again.';
        errorMessageDiv.innerHTML = `
            <div class="bot-avatar">🤖</div>
            <div class="message-content">
                <p>${errorMsg}</p>
            </div>
        `;
        messagesContainer.appendChild(errorMessageDiv);
        messagesContainer.scrollTop = messagesContainer.scrollHeight;
    }
}

// Local chatbot response generator (works offline with Urdu support)
function getLocalChatbotResponse(message, language) {
    const lowerMessage = message.toLowerCase();

    // Responses in both languages
    const responses = {
        // Greetings
        greeting: {
            english: "Hello! I'm your AI Robotics Assistant. I can help you with robotics, ROS2, computer vision, machine learning, and more. How can I assist you today?",
            urdu: "السلام علیکم! میں آپ کا AI روبوٹکس اسسٹنٹ ہوں۔ میں آپ کی روبوٹکس، ROS2، کمپیوٹر ویژن، مشین لرننگ اور بہت کچھ میں مدد کر سکتا ہوں۔ میں آج آپ کی کیسے مدد کروں؟"
        },
        // About the book
        aboutBook: {
            english: "This book covers Physical AI & Robotics comprehensively across 10 modules and 80 chapters. It includes topics like ROS2, Computer Vision, Machine Learning, Autonomous Navigation, Humanoid Robotics, and real-world projects. Perfect for learners from beginner to expert level!",
            urdu: "یہ کتاب 10 ماڈیولز اور 80 ابواب میں فزیکل AI اور روبوٹکس کا جامع احاطہ کرتی ہے۔ اس میں ROS2، کمپیوٹر ویژن، مشین لرننگ، خودکار نیویگیشن، ہیومنائڈ روبوٹکس، اور حقیقی منصوبے شامل ہیں۔ ابتدائی سے ماہر سطح تک سیکھنے والوں کے لیے بہترین!"
        },
        // ROS2
        ros2: {
            english: "ROS2 (Robot Operating System 2) is a flexible framework for robot software development. It provides tools and libraries for building robot applications including: message passing, package management, hardware abstraction, and distributed computing. Our Module 3 covers ROS2 in depth!",
            urdu: "ROS2 روبوٹ سافٹ ویئر کی ترقی کے لیے ایک لچکدار فریم ورک ہے۔ یہ روبوٹ ایپلیکیشنز بنانے کے لیے ٹولز اور لائبریریاں فراہم کرتا ہے بشمول: میسج پاسنگ، پیکیج مینجمنٹ، ہارڈویئر ایبسٹریکشن، اور ڈسٹریبیوٹڈ کمپیوٹنگ۔ ہمارا ماڈیول 3 ROS2 کا گہرائی سے احاطہ کرتا ہے!"
        },
        // Computer Vision
        vision: {
            english: "Computer Vision enables robots to see and understand their environment. Key topics include: image processing, object detection (YOLO, R-CNN), semantic segmentation, 3D vision, depth estimation, SLAM, and real-time processing. Check out Module 4 for detailed coverage!",
            urdu: "کمپیوٹر ویژن روبوٹس کو اپنے ماحول کو دیکھنے اور سمجھنے کے قابل بناتا ہے۔ اہم موضوعات میں شامل ہیں: تصویری پروسیسنگ، آبجیکٹ ڈیٹیکشن (YOLO، R-CNN)، سیمانٹک سیگمنٹیشن، 3D ویژن، گہرائی کا تخمینہ، SLAM، اور ریئل ٹائم پروسیسنگ۔ تفصیلی احاطہ کے لیے ماڈیول 4 دیکھیں!"
        },
        // Machine Learning
        ml: {
            english: "Machine Learning for Robotics covers supervised learning, reinforcement learning (DQN, PPO, SAC), imitation learning, transfer learning, and sim-to-real transfer. Module 6 provides in-depth coverage of ML techniques for robot intelligence.",
            urdu: "روبوٹکس کے لیے مشین لرننگ میں سپروائزڈ لرننگ، ری انفورسمنٹ لرننگ (DQN، PPO، SAC)، امیٹیشن لرننگ، ٹرانسفر لرننگ، اور سم ٹو ریئل ٹرانسفر شامل ہیں۔ ماڈیول 6 روبوٹ انٹیلیجنس کے لیے ML تکنیکوں کا گہرائی سے احاطہ فراہم کرتا ہے۔"
        },
        // Modules
        modules: {
            english: "The book has 10 modules: 1) Introduction to Robotics & AI, 2) Robot Hardware & Components, 3) ROS2 Deep Dive, 4) Computer Vision, 5) Motion Planning & Control, 6) Machine Learning, 7) Autonomous Navigation, 8) Humanoid Robotics, 9) Advanced Topics, 10) Real-World Projects.",
            urdu: "کتاب میں 10 ماڈیولز ہیں: 1) روبوٹکس اور AI کا تعارف، 2) روبوٹ ہارڈویئر اور اجزاء، 3) ROS2 گہرائی میں، 4) کمپیوٹر ویژن، 5) موشن پلاننگ اور کنٹرول، 6) مشین لرننگ، 7) خودکار نیویگیشن، 8) ہیومنائڈ روبوٹکس، 9) جدید موضوعات، 10) حقیقی منصوبے۔"
        },
        // Chapters
        chapters: {
            english: "This comprehensive book contains 80 detailed chapters covering everything from robotics basics to advanced topics like humanoid robotics and real-world projects. Each chapter includes practical examples, code snippets, and hands-on exercises.",
            urdu: "اس جامع کتاب میں 80 تفصیلی ابواب ہیں جو روبوٹکس کی بنیادی باتوں سے لے کر ہیومنائڈ روبوٹکس اور حقیقی منصوبوں جیسے جدید موضوعات تک ہر چیز کا احاطہ کرتے ہیں۔ ہر باب میں عملی مثالیں، کوڈ سنیپٹس، اور ہینڈز آن مشقیں شامل ہیں۔"
        },
        // Navigation
        navigation: {
            english: "Autonomous Navigation (Module 7) covers localization, map building, AMCL, Navigation2 stack, path planning, obstacle avoidance, multi-robot coordination, and GPS navigation. Essential skills for building autonomous mobile robots!",
            urdu: "خودکار نیویگیشن (ماڈیول 7) میں لوکلائزیشن، نقشہ سازی، AMCL، نیویگیشن2 اسٹیک، پاتھ پلاننگ، رکاوٹوں سے بچنا، ملٹی روبوٹ کوآرڈینیشن، اور GPS نیویگیشن شامل ہیں۔ خودکار موبائل روبوٹس بنانے کے لیے ضروری مہارتیں!"
        },
        // Humanoid
        humanoid: {
            english: "Humanoid Robotics (Module 8) teaches bipedal walking, ZMP balance control, gait generation, whole-body control, perception, human-robot interaction, and manipulation. Learn how robots like Atlas, Optimus, and Figure 01 work!",
            urdu: "ہیومنائڈ روبوٹکس (ماڈیول 8) دو پیروں پر چلنا، ZMP بیلنس کنٹرول، چال کی تخلیق، پورے جسم کا کنٹرول، ادراک، انسان-روبوٹ تعامل، اور ہیرا پھیری سکھاتا ہے۔ جانیں کہ Atlas، Optimus، اور Figure 01 جیسے روبوٹس کیسے کام کرتے ہیں!"
        },
        // Default response
        default: {
            english: "I can help you with questions about robotics, ROS2, computer vision, machine learning, autonomous navigation, humanoid robotics, and more. Try asking about specific topics like 'What is ROS2?' or 'Tell me about computer vision'. What would you like to know?",
            urdu: "میں روبوٹکس، ROS2، کمپیوٹر ویژن، مشین لرننگ، خودکار نیویگیشن، ہیومنائڈ روبوٹکس، اور مزید کے بارے میں سوالات میں آپ کی مدد کر سکتا ہوں۔ مخصوص موضوعات کے بارے میں پوچھنے کی کوشش کریں جیسے 'ROS2 کیا ہے؟' یا 'مجھے کمپیوٹر ویژن کے بارے میں بتائیں'۔ آپ کیا جاننا چاہیں گے؟"
        }
    };

    // Determine which response to return based on message content
    let responseKey = 'default';

    if (lowerMessage.match(/hi|hello|salam|السلام علیکم|ہیلو/)) {
        responseKey = 'greeting';
    } else if (lowerMessage.match(/book|کتاب|modules|ماڈیولز|overview/)) {
        if (lowerMessage.match(/module|ماڈیول/)) {
            responseKey = 'modules';
        } else if (lowerMessage.match(/chapter|باب/)) {
            responseKey = 'chapters';
        } else {
            responseKey = 'aboutBook';
        }
    } else if (lowerMessage.match(/ros2|ros 2|robot operating system/)) {
        responseKey = 'ros2';
    } else if (lowerMessage.match(/vision|computer vision|opencv|yolo|کمپیوٹر ویژن|بصارت/)) {
        responseKey = 'vision';
    } else if (lowerMessage.match(/machine learning|ml|deep learning|reinforcement|مشین لرننگ/)) {
        responseKey = 'ml';
    } else if (lowerMessage.match(/navigation|nav2|autonomous|خودکار|نیویگیشن/)) {
        responseKey = 'navigation';
    } else if (lowerMessage.match(/humanoid|bipedal|atlas|optimus|figure|ہیومنائڈ/)) {
        responseKey = 'humanoid';
    }

    return responses[responseKey][language];
}

// Handle enter key in chatbot input
function handleChatbotKeyPress(event) {
    if (event.key === 'Enter') {
        sendChatbotMessage();
    }
}
