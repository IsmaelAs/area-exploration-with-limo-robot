"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ClientSocketService = void 0;
const socket_io_client_1 = require("socket.io-client");
class ClientSocketService {
    constructor() {
        this.socket = (0, socket_io_client_1.io)('http://localhost:3000');
    }
    connectToServer() {
        this.socket.on('connect', () => {
            //receive a message form the server
            this.socket.on('server-message', (message) => {
                console.log('le client a recu un message du serveur');
                console.log(message);
                // send a message back to server
                setTimeout(() => {
                    this.socket.emit('client-message', 'Ceci est un message du client');
                }, 1000);
            });
        });
    }
}
exports.ClientSocketService = ClientSocketService;
//# sourceMappingURL=client-socket.js.map