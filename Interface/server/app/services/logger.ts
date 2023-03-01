import LogLimo from '@app/interfaces/log-limo'
import {appendFile, readFile}from 'fs'
import { Socket } from 'socket.io'

const LOGS_PATH = "./app/logs"


export class Logger {
    private currentMission = 0

    saveLimoData(data: LogLimo): void {
        const stringData = JSON.stringify(data.data)
        const log = `[${(new Date()).toString()} : LIMO-${data.limoId}] : ${stringData}\n`
        appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        })
    }

    saveUserData<T>(data: T) {
        const stringData = JSON.stringify(data)
        const log = `[${(new Date()).toString()} : User] : ${stringData}\n`
        appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        }, )
    }

    getAllData(missionNumber: number, socket: Socket): void {
        readFile(`${LOGS_PATH}/logs-${missionNumber}.log`, (err: Error, data: Buffer) => {
            if (err) {
                console.error(err.stack)
                socket.emit("send-all-logs", `Erreur dans la lecture du fichier de log de la misson ${missionNumber} : ${err.message}`)

            } else {
                socket.emit("send-all-logs", data.toString('utf8'))
            }
        })
    }

    startMission() {
        this.currentMission += 1
    }
}