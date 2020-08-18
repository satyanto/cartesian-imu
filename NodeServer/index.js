// Our HTTP server, serves as a 'hook' for the websocket server
const port = process.env.PORT || 3000;
const index = '/index.html';

const server = express()
    .use((req, res) => res.sendFile(INDEX, { root: __dirname }))
    .listen(port, () => console.log('Listening on ${port}'))

// Our WebSocket server, takes an HTTP server as an argument
const { Server } = require('ws');
const wss = new Server({ server });

wss.on('connection', (ws) => {
    console.log('Client connected');
    ws.on('close', () => console.log('Client disconnected'));
})

setInterval(() => {
    wss.clients.forEach((client) => {
        client.send(new Data().toString());
    });
}, 1000);   // every second

