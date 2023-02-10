import { io, Socket } from 'socket.io-client';
import { LIMO_URL_2 } from '../constants/url';

export class ClientSocketLimo2 {
    private socket: Socket;

    constructor() {
        this.socket = io(LIMO_URL_2);
    }

    init() {
        this.socket.on("connect", () => {
            console.log('Limo 2 connected to the ROS server');
        })
    }

    emit<T>(event: string, data: T) {
        data ? this.socket.emit(event, data) : this.socket.emit(event)
    }

}