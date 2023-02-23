import { Server } from "socket.io";
import { NodePosition } from "./ros/nodes/node-position";

export class Logger {
    private socketServer: Server
    private nodePosition: NodePosition
    private intervalLog: NodeJS.Timer

    constructor(socketServer: Server) {
        this.socketServer = socketServer
        this.nodePosition.initNodePosition()
    }
    
    startLogs() {
        this.intervalLog = setInterval(this.callBack, 1000)
    }

    private callBack() {
        const data = this.nodePosition.getData()
        const limoId = process.env.LIMO_ID
        this.socketServer.emit("save-log", {limoId: limoId, data: data})
    }

    stopLog() {
        const limoId = process.env.LIMO_ID
        this.socketServer.emit("save-log", {limoId: limoId, data: "Stop sending logs"})
        clearInterval(this.intervalLog)
    }
}