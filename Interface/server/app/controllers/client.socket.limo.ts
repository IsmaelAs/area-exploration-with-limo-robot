import { io, Socket } from 'socket.io-client';
import { LIMO_URL } from '../constants/url';
import { Logger } from '../services/logger';
import LogLimo from '@app/interfaces/log-limo';

export class ClientSocketLimo {
    private socket: Socket;
    private logger: Logger
    private limoId: number

    constructor(limoId: number) {
        this.limoId = limoId
        this.socket = io(LIMO_URL);
        this.logger = new Logger()
    }

    connectClientSocketToLimo() {
        this.socket.on("connect", () => {
            console.log(`Limo ${this.limoId} connected to the ROS server`);
            this.logger.saveLimoData({limoId: this.limoId, data: `Limo ${this.limoId} connected to the ROS server`})
            
            this.socket.on("save-log", (data: LogLimo) => {
                this.logger.saveLimoData(data)
            })

            this.socket.on("disconnect", () => {
                this.socket.removeAllListeners()
            })

            this.socket.emit("login", this.limoId)
        })
    }

    emitToLimo<T>(event: string, data?: T) {
        data ? this.socket.emit(event, data) : this.socket.emit(event)
    }

    startMission() {
        this.logger.startMission()
    }

    stopMission() {
        this.logger.stopMission()
    }

}