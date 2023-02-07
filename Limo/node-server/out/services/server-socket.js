"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.ServerSocketService = void 0;
class ServerSocketService {
    constructor(io) {
        this.io = io;
    }
    init() {
        this.io.on('connection', (socket) => {
            console.log('Le server a recu une connection du socket:');
            // send a message to client
            setTimeout(() => {
                socket.emit('server-message', 'Ceci est un message du serveur');
            }, 1000);
            //receive a message from client
            socket.on('client-message', (message) => {
                console.log('Le serveur a recu un message du client');
                console.log(message);
            });
        });
    }
}
exports.ServerSocketService = ServerSocketService;
//# sourceMappingURL=server-socket.js.map