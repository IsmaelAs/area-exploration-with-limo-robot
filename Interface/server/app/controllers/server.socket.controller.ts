import { Server as SocketServer } from 'socket.io';
import RobotMovement from '@app/interfaces/robots-movement-interface';

export class ServerSocketController {
    private io: SocketServer;

    constructor(io: SocketServer) {
        this.io = io;
    }

    init() {
        this.io.on("connection", (socket) => {
            
            socket.on('advance', (movement: RobotMovement) => {     
                this.io.emit(`${movement.robot}-move`, { direction: movement.direction, distance: movement.distance});
            })

            socket.on('start-mission', (movement: RobotMovement) => {
                this.io.emit(`${movement.robot}-move`, { direction: movement.direction, distance: movement.distance});
            })

            socket.on('stop-mission', (movement: RobotMovement) => {
                this.io.emit(`${movement.robot}-move`, { direction: movement.direction, distance: movement.distance});
            })
        })
    }

}