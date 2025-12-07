const fs = require('fs').promises;
const path = require('path');
const { GoogleGenerativeAI } = require('@google/generative-ai');
const markdownit = require('markdown-it')();

// Load environment variables from backend directory
require('dotenv').config();

const MODEL_NAME = "gemini-pro"; // Or "gemini-ultra" if available and desired
const API_KEY = process.env.GEMINI_API_KEY;

if (!API_KEY) {
    console.error("GEMINI_API_KEY is not set in the .env file.");
    process.exit(1);
}

const genAI = new GoogleGenerativeAI(API_KEY);
const embeddingModel = genAI.getGenerativeModel({ model: "embedding-001" }); // Specific embedding model

let documentStore = []; // In-memory store for { text, embedding, metadata }

async function getEmbedding(text) {
    try {
        const result = await embeddingModel.embedContent(text);
        return result.embedding.values;
    } catch (error) {
        console.error("Error getting embedding:", error);
        throw error;
    }
}

function chunkText(text, chunkSize = 1000, overlap = 200) {
    const chunks = [];
    let i = 0;
    while (i < text.length) {
        let end = i + chunkSize;
        if (end > text.length) {
            end = text.length;
        } else {
            // Try to end at a natural break (e.g., end of sentence/paragraph)
            let lastPeriod = text.lastIndexOf('.', end);
            let lastNewline = text.lastIndexOf('\n', end);
            let breakPoint = Math.max(lastPeriod, lastNewline);
            if (breakPoint > i && end - breakPoint < overlap) {
                end = breakPoint + 1;
            }
        }
        
        chunks.push(text.substring(i, end));
        i += chunkSize - overlap;
    }
    return chunks;
}


async function loadAndProcessDocument(filePath) {
    console.log(`Processing ${filePath}...`);
    try {
        const markdownContent = await fs.readFile(filePath, 'utf8');
        // Convert markdown to plain text
        const plainText = markdownit.render(markdownContent);

        const chunks = chunkText(plainText);

        for (const chunk of chunks) {
            const embedding = await getEmbedding(chunk);
            documentStore.push({
                text: chunk,
                embedding: embedding,
                metadata: { source: filePath }
            });
        }
        console.log(`Finished processing ${filePath}. Chunks: ${chunks.length}`);
    } catch (error) {
        console.error(`Failed to process ${filePath}:`, error);
    }
}

function cosineSimilarity(vec1, vec2) {
    const dotProduct = vec1.reduce((sum, val, i) => sum + val * vec2[i], 0);
    const magnitude1 = Math.sqrt(vec1.reduce((sum, val) => sum + val * val, 0));
    const magnitude2 = Math.sqrt(vec2.reduce((sum, val) => sum + val * val, 0));
    if (magnitude1 === 0 || magnitude2 === 0) return 0;
    return dotProduct / (magnitude1 * magnitude2);
}

async function retrieveRelevantDocuments(query, topK = 3) {
    const queryEmbedding = await getEmbedding(query);
    const similarities = documentStore.map(doc => ({
        doc,
        similarity: cosineSimilarity(queryEmbedding, doc.embedding)
    }));

    similarities.sort((a, b) => b.similarity - a.similarity);

    return similarities.slice(0, topK).map(s => s.doc);
}

// Function to initialize the RAG pipeline by loading all documents
async function initializeRagPipeline() {
    console.log("Initializing RAG pipeline...");
    documentStore = []; // Clear previous store if any

    const docFiles = [
        // List all your markdown/mdx files here
        // Example:
        // path.join(__dirname, '..', 'physical-ai-robotics', 'docs', 'intro.md'),
        // path.join(__dirname, '..', 'physical-ai-robotics', 'blog', '2025-12-05-meet-dr-minahil-hamza.md'),
        // ... (populate this list dynamically or manually based on the glob results)
    ];

    // Placeholder for all markdown files. This will be populated after testing.
    const allMarkdownFiles = [
        "physical-ai-robotics/docs/intro.md",
        "physical-ai-robotics/docs/getting-started.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/index.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/index.md",
        "physical-ai-robotics/docs/modules/module-4-vla/index.md",
        "physical-ai-robotics/docs/modules/module-4-vla/4-capstone-autonomous-humanoid.md",
        "physical-ai-robotics/docs/modules/module-4-vla/3-cognitive-planning-llms.md",
        "physical-ai-robotics/docs/modules/module-4-vla/2-voice-to-action-whisper.md",
        "physical-ai-robotics/docs/modules/module-4-vla/1-introduction-to-vla.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/3-nav2.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/2-isaac-ros.md",
        "physical-ai-robotics/docs/modules/module-3-ai-robot-brain/1-isaac-sim.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/3-simulating-sensors.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/2-unity-rendering.md",
        "physical-ai-robotics/docs/modules/module-2-digital-twin/1-gazebo-simulation.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/3-understanding-urdf.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/2-bridging-agents-to-controllers.md",
        "physical-ai-robotics/docs/modules/module-1-ros2/1-core-concepts.md",
        "physical-ai-robotics/docs/assessments.md",
        "physical-ai-robotics/docs/constitution.md",
        "physical-ai-robotics/docs/hardware-requirements/digital-twin-workstation.md",
        "physical-ai-robotics/docs/hardware-requirements/edge-kit.md",
        "physical-ai-robotics/docs/hardware-requirements/index.md",
        "physical-ai-robotics/docs/hardware-requirements/robot-lab.md",
        "physical-ai-robotics/docs/learning-outcomes.md",
        "physical-ai-robotics/docs/quarter-overview.md",
        "physical-ai-robotics/docs/tutorial-basics/congratulations.md",
        "physical-ai-robotics/docs/tutorial-basics/create-a-blog-post.md",
        "physical-ai-robotics/docs/tutorial-basics/create-a-document.md",
        "physical-ai-robotics/docs/tutorial-basics/create-a-page.md",
        "physical-ai-robotics/docs/tutorial-basics/deploy-your-site.md",
        "physical-ai-robotics/docs/tutorial-extras/manage-docs-versions.md",
        "physical-ai-robotics/docs/tutorial-extras/translate-your-site.md",
        "physical-ai-robotics/docs/weekly-breakdown.md",
        "physical-ai-robotics/docs/why-physical-ai-matters.md",
        "physical-ai-robotics/docs/tutorial-basics/markdown-features.mdx",
        "physical-ai-robotics/blog/2025-12-05-meet-dr-minahil-hamza.md",
        "physical-ai-robotics/blog/2019-05-28-first-blog-post.md",
        "physical-ai-robotics/blog/2019-05-29-long-blog-post.md",
        "physical-ai-robotics/blog/2021-08-01-mdx-blog-post.mdx"
    ];

    await Promise.all(allMarkdownFiles.map(file => loadAndProcessDocument(path.join(__dirname, '..', file))));

    console.log(`RAG pipeline initialized with ${documentStore.length} document chunks.`);
}

module.exports = {
    initializeRagPipeline,
    retrieveRelevantDocuments,
    getEmbedding, // Export for testing
    chunkText // Export for testing
};
