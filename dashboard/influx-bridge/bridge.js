/**
 * bridge.js - Node.js script to act as a data bridge between your AI system and InfluxDB.
 * It listens for incoming JSON sensor data and writes it to InfluxDB in line protocol format.
 * It also simulates data every 5 seconds for testing.
 */

const express = require('express');
const bodyParser = require('body-parser');
const axios = require('axios');

const app = express();
const port = 4000;

// InfluxDB settings
const INFLUX_URL = 'http://localhost:8086/api/v2/write';
const ORG = 'csse4011org';
const BUCKET = 'csse4011bucket';
const TOKEN = 'MWskLjvhS6Qh6Oa7cfnP-atlV6VfE6jys_w2GnVzuovA3f7ejA-ZO2RNtLI6GN4lLkzkG8A-lv4adE11i57nNA==';

// Middleware to parse JSON
app.use(bodyParser.json());

app.post('/data', async (req, res) => {
    const { gesture, proximity } = req.body;

    if (gesture === undefined || proximity === undefined) {
        return res.status(400).send('Missing required fields: gesture or proximity');
    }

    const lines = [
        `gesture_detection value=${gesture}`,
        `proximity_alert value=${proximity}`
    ];

    try {
        const response = await axios.post(
            `${INFLUX_URL}?org=${ORG}&bucket=${BUCKET}&precision=ns`,
            lines.join('\n'),
            {
                headers: {
                    'Authorization': `Token ${TOKEN}`,
                    'Content-Type': 'text/plain',
                }
            }
        );

        if (response.status === 204) {
            res.status(200).send('Data written to InfluxDB.');
        } else {
            res.status(500).send('Unexpected response from InfluxDB.');
        }
    } catch (err) {
        console.error('Error writing to InfluxDB:', err.message);
        res.status(500).send('Failed to write to InfluxDB.');
    }
});

app.listen(port, () => {
    console.log(`Bridge running at http://localhost:${port}/data`);
});

// === DUMMY DATA SENDER ===
setInterval(async () => {
    const gestureCode = Math.floor(Math.random() * 4); // 0, 1, 2, 3
    const dummyData = {
        gesture: gestureCode,
        proximity: (Math.random() * 3).toFixed(2)
    };

    try {
        await axios.post(`http://localhost:${port}/data`, dummyData);
        console.log(`Dummy data sent: ${JSON.stringify({
            gesture: gestureCode,
            proximity: dummyData.proximity
        })}`);
    } catch (err) {
        console.error('Failed to send dummy data:', err.message);
    }
}, 2000);
