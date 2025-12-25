// Script to extract book structure/metadata without chapter content
const fs = require('fs');
const bookContent = require('./book_content');

// Extract structure without full chapter content
const metadata = {
    title: bookContent.title,
    description: bookContent.description,
    author: bookContent.author,
    authorBio: bookContent.authorBio,
    publicationDate: bookContent.publicationDate,
    version: bookContent.version,
    totalModules: bookContent.totalModules,
    totalChapters: bookContent.totalChapters,
    modules: bookContent.modules.map(module => ({
        id: module.id,
        title: module.title,
        description: module.description,
        difficulty: module.difficulty,
        estimatedTime: module.estimatedTime,
        chapters: module.chapters.map(chapter => ({
            id: chapter.id,
            title: chapter.title,
            learningObjectives: chapter.learningObjectives,
            readingTime: chapter.readingTime,
            keywords: chapter.keywords
            // NOTE: content field is excluded to keep file size small
        }))
    }))
};

// Write metadata to separate file
const outputPath = __dirname + '/book_metadata.js';
const outputContent = `// Book Structure/Metadata (without full chapter content)
// This file is optimized for fast loading on serverless platforms
const bookMetadata = ${JSON.stringify(metadata, null, 2)};

module.exports = bookMetadata;
`;

fs.writeFileSync(outputPath, outputContent);
console.log('✅ Book metadata generated successfully!');
console.log(`📚 Total Modules: ${metadata.totalModules}`);
console.log(`📖 Total Chapters: ${metadata.totalChapters}`);
console.log(`💾 Metadata File: ${outputPath}`);

// Calculate file sizes
const fullContentSize = fs.statSync(__dirname + '/book_content.js').size;
const metadataSize = fs.statSync(outputPath).size;
console.log(`\n📊 Size Comparison:`);
console.log(`   Full content: ${(fullContentSize / 1024 / 1024).toFixed(2)} MB`);
console.log(`   Metadata only: ${(metadataSize / 1024).toFixed(2)} KB`);
console.log(`   Reduction: ${((1 - metadataSize / fullContentSize) * 100).toFixed(1)}%`);
