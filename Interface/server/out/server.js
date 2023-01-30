"use strict";
var __decorate = (this && this.__decorate) || function (decorators, target, key, desc) {
    var c = arguments.length, r = c < 3 ? target : desc === null ? desc = Object.getOwnPropertyDescriptor(target, key) : desc, d;
    if (typeof Reflect === "object" && typeof Reflect.decorate === "function") r = Reflect.decorate(decorators, target, key, desc);
    else for (var i = decorators.length - 1; i >= 0; i--) if (d = decorators[i]) r = (c < 3 ? d(r) : c > 3 ? d(target, key, r) : d(target, key)) || r;
    return c > 3 && r && Object.defineProperty(target, key, r), r;
};
var __metadata = (this && this.__metadata) || function (k, v) {
    if (typeof Reflect === "object" && typeof Reflect.metadata === "function") return Reflect.metadata(k, v);
};
var Server_1;
Object.defineProperty(exports, "__esModule", { value: true });
exports.Server = void 0;
const http = require("http");
const typedi_1 = require("typedi");
const app_1 = require("./app");
let Server = Server_1 = class Server {
    constructor(application) {
        this.application = application;
    }
    static normalizePort(val) {
        const port = typeof val === 'string' ? parseInt(val, this.baseDix) : val;
        if (isNaN(port)) {
            return val;
        }
        else if (port >= 0) {
            return port;
        }
        else {
            return false;
        }
    }
    init() {
        this.application.app.set('port', Server_1.appPort);
        this.server = http.createServer(this.application.app);
        this.server.listen(Server_1.appPort);
        this.server.on('error', (error) => this.onError(error));
        this.server.on('listening', () => this.onListening());
    }
    onError(error) {
        if (error.syscall !== 'listen') {
            throw error;
        }
        const bind = typeof Server_1.appPort === 'string' ? 'Pipe ' + Server_1.appPort : 'Port ' + Server_1.appPort;
        switch (error.code) {
            case 'EACCES':
                // eslint-disable-next-line no-console
                console.error(`${bind} requires elevated privileges`);
                process.exit(1);
                break;
            case 'EADDRINUSE':
                // eslint-disable-next-line no-console
                console.error(`${bind} is already in use`);
                process.exit(1);
                break;
            default:
                throw error;
        }
    }
    /**
     * Se produit lorsque le serveur se met à écouter sur le port.
     */
    onListening() {
        const addr = this.server.address();
        const bind = typeof addr === 'string' ? `pipe ${addr}` : `port ${addr.port}`;
        // eslint-disable-next-line no-console
        console.log(`Listening on ${bind}`);
    }
};
Server.appPort = Server_1.normalizePort(process.env.PORT || '3000');
// eslint-disable-next-line @typescript-eslint/no-magic-numbersz
Server.baseDix = 10;
Server = Server_1 = __decorate([
    (0, typedi_1.Service)(),
    __metadata("design:paramtypes", [app_1.Application])
], Server);
exports.Server = Server;
//# sourceMappingURL=server.js.map