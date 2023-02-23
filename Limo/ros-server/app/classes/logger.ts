import { NodePosition } from "./ros/nodes/node-position";
import { SocketServer } from "./socket-server";

export class Logger {
    private socketServer: SocketServer
    private nodePosition: NodePosition
    private intervalLog: NodeJS.Timer

    constructor(socketServer: SocketServer) {
        this.socketServer = socketServer
        this.nodePosition.initNodePosition()
    }
    
    startLogs() {
        this.intervalLog = setInterval(this.callBack, 1000)
    }

    private callBack() {
        const position = this.positionLog()
        const limoId = this.socketServer.limoId
        this.socketServer.emit("save-log", {limoId: limoId, data: position})
    }

    private positionLog() {
        const data = this.nodePosition.getData()
        const distance = Math.sqrt(Math.pow(data.pose.pose.position.x, 2) + Math.pow(data.pose.pose.position.y, 2) + Math.pow(data.pose.pose.position.z, 2))
        const position = {
            position: data.pose.pose.position,
            distance: distance
        }
        return position
    }

    stopLog() {
        this.socketServer.emit("save-log", {limoId: this.socketServer.limoId, data: "Stop sending logs"})
        clearInterval(this.intervalLog)
    }
}