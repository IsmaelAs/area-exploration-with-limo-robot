import { Server as SocketServer } from 'socket.io';
import RobotMovement from '@app/interfaces/robots-movement-interface';
import OnRobotMovement from '@app/interfaces/on-robots-movement-interface';
import { ClientSocketLimo } from './client.socket.limo';
import { Logger } from '../services/logger';
import { State } from '@app/types/States';

export class ServerSocketController {
    private io: SocketServer;
    private socketLimo?: ClientSocketLimo
    private socketLimo2?: ClientSocketLimo
    private logger: Logger

    constructor(io: SocketServer, socketLimo? : ClientSocketLimo, socketLimo2? : ClientSocketLimo) {
        this.io = io;
        this.socketLimo = socketLimo
        this.socketLimo2 = socketLimo2
        this.logger = new Logger()
    }

    initializeSocketServer() {
        this.io.on("connection", (socket) => {
            socket.on('identify', (movement: RobotMovement) => {   
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }
                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) 
                    this.socketLimo.emitToLimo("limo-move", data)
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && this.socketLimo2) 
                    this.socketLimo2.emitToLimo("limo-move", data)
            })

            socket.on('start-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                
                console.log('start mission received', data);
                console.log("emit on ", "limo-move");
                this.logger.startMission()

                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) {
                    this.socketLimo.emitToLimo("limo-move", data)
                    this.socketLimo.emitToLimo("start-mission")
                    this.socketLimo.startMission()
                }
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && this.socketLimo2) {
                    this.socketLimo2.emitToLimo("limo-move", data)
                    this.socketLimo2.emitToLimo("start-mission")
                    this.socketLimo2.startMission()
                }
            })

            socket.on('stop-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                

                if ((movement.robot == "limo-1" || movement.robot == 'robots') && this.socketLimo) {
                    this.socketLimo.emitToLimo("limo-move", data)
                    this.socketLimo.emitToLimo("stop-mission")
                }
                if ((movement.robot == "limo-2" || movement.robot == 'robots') && this.socketLimo2) {
                    this.socketLimo2.emitToLimo("limo-move", data)
                    this.socketLimo2.emitToLimo("stop-mission")
                }
            })

            socket.on("save-log", (data: unknown) => {
                this.logger.saveUserData(data)
            })


            socket.on("get-all-logs", (missionNumber: number) => {
                this.logger.getAllData(missionNumber, socket)
            })

            socket.on("save-state", (data: {limoId: number, state: State}) => {
                socket.emit("send-all-logs", `L'état du robot ${data.limoId} est: ${data.state}`)
            })
        })
    }
}
