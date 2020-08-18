const express = require('express');

// Our HTTP server, serves as a 'hook' for the websocket server
const PORT = process.env.PORT || 3000;
const INDEX = '/index.html';

const server = express()
    .use((req, res) => res.sendFile(INDEX, { root: __dirname }))
    .listen(PORT, () => console.log('Listening on ${PORT}'))

// Our WebSocket server, takes an HTTP server as an argument
const { Server } = require('ws');
const wss = new Server({ server });

wss.on('connection', (ws) => {
    console.log('Client connected');

    // user sent a message
    ws.on('message', function(message) {
        if (message.type === 'utf8') {      // make sure the messages the server receives is a string
            wss.clients.forEach((client) => {
                client.sendUTF(message);    // we are simply the messenger
            });
        }
    });

    // user disconnected
    ws.on('close', () => console.log('Client disconnected'));
});
