import { io, Socket } from 'socket.io-client';
import { LIMO_URL } from '../constants/url';
import { Logger } from '@app/services/logger';
import LogLimo from '@app/interfaces/log-limo';

export class ClientSocketLimo1 {
    private socket: Socket;
    private logger: Logger

    constructor() {
        this.socket = io(LIMO_URL);
    }

    connectClientSocketToLimo1() {
        this.socket.on("connect", () => {
            console.log('Limo 1 connected to the ROS server');
            this.logger.saveLimoData({limoId: 1, data: "Limo 1 connected to the ROS server"})

            this.socket.on("save-log", (data: LogLimo) => {
                this.logger.saveLimoData(data)
            })
        })
    }

    emitToLimo1<T>(event: string, data?: T) {
        data ? this.socket.emit(event, data) : this.socket.emit(event)
    }

    startMission() {
        this.logger.startMission()
    }

}