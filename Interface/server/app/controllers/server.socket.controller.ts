import { Server as SocketServer } from 'socket.io';
import RobotMovement from '@app/interfaces/robots-movement-interface';
import OnRobotMovement from '@app/interfaces/on-robots-movement-interface';
import { ClientSocketLimo1 } from './client.socket.limo';
import { ClientSocketLimo2 } from './client.socket.limo2';
import { Logger } from '@app/services/logger';

export class ServerSocketController {
    private io: SocketServer;
    private socketLimo?: ClientSocketLimo1
    private socketLimo2?: ClientSocketLimo2
    private logger: Logger

    constructor(io: SocketServer, socketLimo? : ClientSocketLimo1, socketLimo2? : ClientSocketLimo2) {
        this.io = io;
        this.socketLimo = socketLimo
        this.socketLimo2 = socketLimo2
    }

    initializeSocketServer() {
        this.io.on("connection", (socket) => {
            socket.on('identify', (movement: RobotMovement) => {   
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }
                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) 
                    this.socketLimo.emitToLimo1(`${movement.robot}-move`, data)
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && this.socketLimo2) 
                    this.socketLimo2.emitToLimo2(`${movement.robot}-move`, data)
            })

            socket.on('start-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                
                console.log('start mission received', data);
                console.log("emit on ", `${movement.robot}-move`);
                
                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) 
                    this.socketLimo.emitToLimo1(`${movement.robot}-move`, data)
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && this.socketLimo2) 
                    this.socketLimo2.emitToLimo2(`${movement.robot}-move`, data)
            })

            socket.on('stop-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                

                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) 
                    this.socketLimo.emitToLimo1(`${movement.robot}-move`, data)
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && 
                    this.socketLimo2) this.socketLimo2.emitToLimo2(`${movement.robot}-move`, data)
            })

            socket.on("save-log", (data: unknown) => {
                this.logger.saveUserData(data)
            })

            socket.on("get-all-logs", (missionNumber: number) => {
                this.logger.getAllData(missionNumber, socket)
            })
        })
    }
}
