import { io, Socket } from 'socket.io-client';
import { LIMO_URL } from '../constants/url';

export class ClientSocketLimo1 {
    private socket: Socket;

    constructor() {
        this.socket = io(LIMO_URL);
    }

    init() {
        this.socket.on("connect", () => {
            console.log('Limo 1 connected to the ROS server');
        })
    }

    emit<T>(event: string, data?: T) {
        data ? this.socket.emit(event, data) : this.socket.emit(event)
    }

}