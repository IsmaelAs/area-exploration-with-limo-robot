import { Server as SocketServer } from 'socket.io';
import RobotMovement from '@app/interfaces/robots-movement-interface';
import OnRobotMovement from '@app/interfaces/on-robots-movement-interface';

export class ServerSocketController {
    private io: SocketServer;

    constructor(io: SocketServer) {
        this.io = io;
    }

    init() {
        this.io.on("connection", (socket) => {
            
            socket.on('identify', (movement: RobotMovement) => {   
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                
                this.io.emit(`${movement.robot}-move`, data);
            })

            socket.on('start-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                
                console.log('start mission recived', data);
                console.log("emit on ", `${movement.robot}-move`);
                
                this.io.emit(`${movement.robot}-move`, data);
            })

            socket.on('stop-mission', (movement: RobotMovement) => {
                const data: OnRobotMovement = {
                    direction: movement.direction, 
                    distance: movement.distance
                }                

                this.io.emit(`${movement.robot}-move`, data);
            })
        })
    }

}