import LogLimo from '@app/interfaces/log-limo'
import * as fs from 'fs'
import { Socket } from 'socket.io'

const LOGS_PATH = "../logs"


export class Logger {
    private currentMission = 0

    saveLimoData(data: LogLimo): void {
        const stringData = JSON.stringify(data.data)
        const log = `[${Date.now()} : LIMO-${data.limoId}] : ${stringData}\n`
        fs.appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        })
    }

    saveUserData<T>(data: T) {
        const stringData = JSON.stringify(data)
        const log = `[${Date.now()} : User] : ${stringData}\n`
        fs.appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        })
    }

    getAllData(missionNumber: number, socket: Socket): void {
        fs.readFile(`${LOGS_PATH}/logs-${missionNumber}.log`, (err: Error, data: Buffer) => {
            if (err) {
                console.error(err.stack)
                socket.emit("send-all-logs", `Erreur dans la lecture du fichier de log de la misson ${missionNumber} : ${err.message}`)

            } else {
                socket.emit("send-all-logs", JSON.stringify(data))
            }
        })
    }

    startMission() {
        this.currentMission += 1
    }
}