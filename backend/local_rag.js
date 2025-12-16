const fs = require('fs').promises;
const path = require('path');

let documentStore = []; // Stores {text, metadata, tokens}

// Simple tokenizer
function tokenize(text) {
    return text.toLowerCase()
        .replace(/[^\w\s]/g, ' ')
        .split(/\s+/)
        .filter(token => token.length > 2);
}

// Calculate TF-IDF scores
function calculateTFIDF(documents) {
    const N = documents.length;
    const idf = {};

    // Calculate document frequency for each term
    documents.forEach(doc => {
        const uniqueTokens = [...new Set(doc.tokens)];
        uniqueTokens.forEach(token => {
            idf[token] = (idf[token] || 0) + 1;
        });
    });

    // Convert to IDF
    Object.keys(idf).forEach(token => {
        idf[token] = Math.log(N / idf[token]);
    });

    return idf;
}

// Calculate similarity between query and document
function calculateSimilarity(queryTokens, docTokens, idf) {
    const queryTF = {};
    queryTokens.forEach(token => {
        queryTF[token] = (queryTF[token] || 0) + 1;
    });

    const docTF = {};
    docTokens.forEach(token => {
        docTF[token] = (docTF[token] || 0) + 1;
    });

    let score = 0;
    const queryTerms = Object.keys(queryTF);

    queryTerms.forEach(term => {
        if (docTF[term]) {
            const tf = docTF[term] / docTokens.length;
            const termIDF = idf[term] || 0;
            score += queryTF[term] * tf * termIDF;
        }
    });

    return score;
}

// Chunk text into smaller pieces
function chunkText(text, chunkSize = 800, overlap = 200) {
    const chunks = [];
    let i = 0;

    while (i < text.length) {
        let end = i + chunkSize;
        if (end > text.length) {
            end = text.length;
        } else {
            // Try to end at sentence boundary
            const lastPeriod = text.lastIndexOf('.', end);
            const lastNewline = text.lastIndexOf('\n', end);
            const breakPoint = Math.max(lastPeriod, lastNewline);
            if (breakPoint > i && end - breakPoint < overlap) {
                end = breakPoint + 1;
            }
        }

        const chunk = text.substring(i, end).trim();
        if (chunk.length > 50) { // Only add substantial chunks
            chunks.push(chunk);
        }
        i += chunkSize - overlap;
    }

    return chunks;
}

// Load and process a single document
async function loadDocument(filePath) {
    try {
        const content = await fs.readFile(filePath, 'utf8');

        // Remove markdown syntax for better text processing
        const plainText = content
            .replace(/```[\s\S]*?```/g, '') // Remove code blocks
            .replace(/`[^`]+`/g, '') // Remove inline code
            .replace(/#+\s/g, '') // Remove headers
            .replace(/\[([^\]]+)\]\([^)]+\)/g, '$1') // Remove links, keep text
            .replace(/[*_~]/g, ''); // Remove emphasis markers

        const chunks = chunkText(plainText);
        const fileName = path.basename(filePath);

        chunks.forEach(chunk => {
            const tokens = tokenize(chunk);
            documentStore.push({
                text: chunk,
                tokens: tokens,
                metadata: {
                    source: fileName,
                    path: filePath
                }
            });
        });

        console.log(`Loaded ${chunks.length} chunks from ${fileName}`);
    } catch (error) {
        console.error(`Failed to load ${filePath}:`, error.message);
    }
}

// Initialize the RAG system by loading all documents
async function initializeLocalRAG() {
    console.log('Initializing local RAG system...');
    documentStore = [];

    const docFiles = [
        "physical-ai-robotics/docs/intro.md",
        "physical-ai-robotics/docs/getting-started.md",
        "physical-ai-robotics/docs/why-physical-ai-matters.md",
        "physical-ai-robotics/docs/learning-outcomes.md",
        "physical-ai-robotics/docs/quarter-overview.md",
        "physical-ai-robotics/docs/weekly-breakdown.md",
        "physical-ai-robotics/docs/assessments.md",
        "physical-ai-robotics/docs/constitution.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/1-core-concepts.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/2-bridging-agents-to-controllers.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/3-understanding-urdf.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/index.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/1-gazebo-simulation.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/2-unity-rendering.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/3-simulating-sensors.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/index.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/1-isaac-sim.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/2-isaac-ros.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/3-nav2.md",
        "physical-ai-robotics/docs/modules/module-4-vla/index.md",
        "physical-ai-robotics/docs/modules/module-4-vla/1-introduction-to-vla.md",
        "physical-ai-robotics/docs/modules/module-4-vla/2-voice-to-action-whisper.md",
        "physical-ai-robotics/docs/modules/module-4-vla/3-cognitive-planning-llms.md",
        "physical-ai-robotics/docs/modules/module-4-vla/4-capstone-autonomous-humanoid.md",
        "physical-ai-robotics/docs/hardware-requirements/index.md",
        "physical-ai-robotics/docs/hardware-requirements/digital-twin-workstation.md",
        "physical-ai-robotics/docs/hardware-requirements/edge-kit.md",
        "physical-ai-robotics/docs/hardware-requirements/robot-lab.md"
    ];

    // Load all documents
    await Promise.all(docFiles.map(file =>
        loadDocument(path.join(__dirname, '..', file))
    ));

    console.log(`Local RAG initialized with ${documentStore.length} document chunks`);
}

// Retrieve relevant documents for a query
function retrieveRelevantDocuments(query, topK = 3) {
    if (documentStore.length === 0) {
        console.warn('Document store is empty. Call initializeLocalRAG first.');
        return [];
    }

    const queryTokens = tokenize(query);
    const idf = calculateTFIDF(documentStore);

    // Calculate similarity for each document
    const scores = documentStore.map(doc => ({
        doc,
        score: calculateSimilarity(queryTokens, doc.tokens, idf)
    }));

    // Sort by score and return top K
    scores.sort((a, b) => b.score - a.score);

    return scores.slice(0, topK).map(s => s.doc);
}

module.exports = {
    initializeLocalRAG,
    retrieveRelevantDocuments
};
