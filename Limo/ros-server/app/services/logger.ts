import { NodePosition } from "../classes/ros/nodes/node-position";
import { SocketServer } from "../controllers/socket-server";

export class Logger {
    private socketServer: SocketServer
    private intervalLog: NodeJS.Timer
    private nodePosition: NodePosition = new NodePosition()

    constructor(socketServer: SocketServer) {
        this.socketServer = socketServer
        this.nodePosition.initNodePosition()
    }
    
    startLogs() {
        this.intervalLog = setInterval(this.callBack.bind(this), 1000)
    }

    private callBack() {
        if (this.socketServer.numberSocketConnected === 0) return

        const position = this.positionLog()
        const limoId = this.socketServer.limoId
        this.socketServer.emit("save-log", {limoId: limoId, data: position})
    }

    private positionLog() {
        const data = this.nodePosition.getData()
        const distance = Math.sqrt(Math.pow(data.pose.pose.position.x, 2) + Math.pow(data.pose.pose.position.y, 2) + Math.pow(data.pose.pose.position.z, 2))
        return {
            x: Math.round(data.pose.pose.position.x * 100) / 100,
            y: Math.round(data.pose.pose.position.y * 100) / 100,
            z: Math.round(data.pose.pose.position.z * 100) / 100,
            distance: Math.round(distance * 100) / 100
        }
    }

    stopLog() {
        this.socketServer.emit("save-log", {limoId: this.socketServer.limoId, data: "Stop sending logs"})
        clearInterval(this.intervalLog)
    }
}