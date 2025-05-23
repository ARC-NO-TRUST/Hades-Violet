const express = require('express');
const bodyParser = require('body-parser');
const axios = require('axios');
const mqtt = require('mqtt');

const app = express();
const port = 4000;

// InfluxDB settings
const INFLUX_URL = 'http://localhost:8086/api/v2/write';
const ORG = 'csse4011org';
const BUCKET = 'csse4011bucket';
const TOKEN = 'MWskLjvhS6Qh6Oa7cfnP-atlV6VfE6jys_w2GnVzuovA3f7ejA-ZO2RNtLI6GN4lLkzkG8A-lv4adE11i57nNA==';

// MQTT setup
const MQTT_TOPIC = 'arcnotrust/data';
const mqttClient = mqtt.connect('mqtt://localhost');

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

// MQTT handling
mqttClient.on('connect', () => {
    console.log('Connected to MQTT broker');
    mqttClient.subscribe(MQTT_TOPIC, (err) => {
        if (err) {
            console.error('MQTT subscription error:', err.message);
        } else {
            console.log(`Subscribed to topic: ${MQTT_TOPIC}`);
        }
    });
});

mqttClient.on('message', async (topic, message) => {
    try {
        const data = JSON.parse(message.toString());
        const { gesture, distance } = data;

        if (gesture === undefined || distance === undefined) {
            console.warn('MQTT message missing fields:', message.toString());
            return;
        }

        const lines = [
            `gesture_detection value=${gesture}`,
            `proximity_alert value=${distance}`
        ];

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
            console.log(`✔ MQTT data written: gesture=${gesture}, distance=${distance}`);
        } else {
            console.error('❌ InfluxDB unexpected response');
        }
    } catch (err) {
        console.error('❌ Failed to process MQTT message:', err.message);
    }
});

app.listen(port, () => {
    console.log(`REST fallback running at http://localhost:${port}/data`);
});
