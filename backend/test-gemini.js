const { GoogleGenerativeAI } = require('@google/generative-ai');
require('dotenv').config();

const GEMINI_API_KEY = process.env.GEMINI_API_KEY;
console.log('Testing Gemini API Key...');
console.log('Key present:', !!GEMINI_API_KEY);

const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);

async function testModels() {
    const models = [
        'gemini-pro',
        'gemini-1.0-pro',
        'gemini-1.5-pro',
        'gemini-1.5-flash',
        'models/gemini-pro',
        'models/gemini-1.5-flash'
    ];

    for (const modelName of models) {
        try {
            console.log(`\nTesting model: ${modelName}`);
            const model = genAI.getGenerativeModel({ model: modelName });
            const result = await model.generateContent('Say hi');
            const response = await result.response;
            const text = response.text();
            console.log(`✓ SUCCESS with ${modelName}:`, text.substring(0, 50));
            break; // If successful, stop testing
        } catch (error) {
            console.log(`✗ FAILED with ${modelName}:`, error.message);
        }
    }
}

testModels().catch(console.error);
