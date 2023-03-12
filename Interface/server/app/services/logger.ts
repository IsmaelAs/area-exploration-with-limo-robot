import LogLimo from '@app/interfaces/log-limo'
import {appendFile, readFile}from 'fs'
import { Socket } from 'socket.io'

const LOGS_PATH = "./app/logs"


export class Logger {
    private currentMission = 0
    private isMissionStop = true

    saveLimoData(data: LogLimo): void {
        if (this.isMissionStop &&  this.currentMission === 0) return

        const d = new Date()
        const dformat = [d.getMonth()+1,
            d.getDate(),
            d.getFullYear()].join('/')+' '+
        [   d.getHours(),
            d.getMinutes(),
            d.getSeconds()].join(':');



        const stringData = JSON.stringify(data.data)
        const log = `[${dformat} : LIMO-${data.limoId}] : ${stringData}\n`
        appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        })
    }

    saveUserData<T>(data: T) {
        if (this.isMissionStop &&  this.currentMission === 0) return

        const d = new Date()
        const dformat = [d.getMonth()+1,
            d.getDate(),
            d.getFullYear()].join('/')+' '+
        [   d.getHours(),
            d.getMinutes(),
            d.getSeconds()].join(':');



        const stringData = JSON.stringify(data)
        const log = `[${dformat} : User] : ${stringData}\n`
        appendFile(`${LOGS_PATH}/logs-${this.currentMission}.log`, log, (err: Error) => {
            if (err) console.error(err.stack)
        }, )
    }

    getAllData(missionNumber: number, socket: Socket): void {        
        if (this.currentMission === 0) {
            socket.emit("send-all-logs", "Aucune mission disponible. Lancer une mission pour commencer")
            return
        }

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
        this.isMissionStop = false
    }

    stopMission() {
        this.isMissionStop = true
    }
}