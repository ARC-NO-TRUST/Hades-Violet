const express = require('express');
const bodyParser = require('body-parser');
const axios = require('axios');
const mqtt = require('mqtt');

const app = express();
const port = 4000;

// InfluxDB settings
const INFLUX_URL = 'http://172.20.10.11:8086/api/v2/write';
const ORG = 'csse4011org';
const BUCKET = 'csse4011bucket';
const TOKEN = 'MWskLjvhS6Qh6Oa7cfnP-atlV6VfE6jys_w2GnVzuovA3f7ejA-ZO2RNtLI6GN4lLkzkG8A-lv4adE11i57nNA==';

// MQTT setup
const MQTT_BROKER = 'mqtt://172.20.10.11';
const MQTT_TOPIC = 'arcnotrust/data';
const mqttClient = mqtt.connect(MQTT_BROKER);

// Express middleware
app.use(bodyParser.json());

async function writeToInfluxDB(gesture, distance) {
    const lines = [
        `gesture_detection value=${gesture}`,
        `proximity_alert value=${distance}`
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
            console.log(`Wrote to InfluxDB: gesture=${gesture}, distance=${distance}`);
            return true;
        } else {
            console.error('Unexpected InfluxDB response:', response.status);
            return false;
        }
    } catch (err) {
        console.error('Error writing to InfluxDB:', err.message);
        return false;
    }
}

// REST fallback route
app.post('/data', async (req, res) => {
    const { gesture, distance } = req.body;

    if (gesture === undefined || distance === undefined) {
        return res.status(400).send('Missing required fields: gesture or distance');
    }

    const success = await writeToInfluxDB(gesture, distance);
    return res.status(success ? 200 : 500).send(
        success ? 'Data written to InfluxDB.' : 'Failed to write to InfluxDB.'
    );
});

// MQTT subscription logic
mqttClient.on('connect', () => {
    console.log('Connected to MQTT broker');
    mqttClient.subscribe(MQTT_TOPIC, (err) => {
        if (err) {
            console.error('MQTT subscription failed:', err.message);
        } else {
            console.log(`Subscribed to MQTT topic: ${MQTT_TOPIC}`);
        }
    });
});

mqttClient.on('message', async (topic, message) => {
    console.log(`MQTT Message Received on ${topic}: ${message.toString()}`);

    try {
        const data = JSON.parse(message.toString());
        const { gesture, distance } = data;

        console.log(`Parsed Gesture: ${gesture}, Distance: ${distance}`);

        if (gesture === undefined || distance === undefined) {
            console.warn('MQTT payload missing fields:', message.toString());
            return;
        }

        const { exec } = require('child_process');
        const curlCommand = `curl -i -XPOST "http://localhost:8086/api/v2/write?org=${ORG}&bucket=${BUCKET}&precision=ns" ` +
            `--header "Authorization: Token ${TOKEN}" ` +
            `--data-raw "gesture_detection value=${gesture}\nproximity_alert value=${distance}"`;

        exec(curlCommand, (error, stdout, stderr) => {
            if (error) {
                console.error(`Curl error: ${error.message}`);
                return;
            }
            console.log(`InfluxDB write result:\n${stdout}`);
        });

    } catch (err) {
        console.error('Failed to parse MQTT message:', err.message);
    }
});

app.listen(port, () => {
    console.log(`REST fallback running: http://localhost:${port}/data`);
});
