import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from 'src/app/constants/url';
import RobotTargetType from 'src/app/types/RobotType';
import { Subject } from 'rxjs';
import { StateType } from 'src/app/interfaces/state-limo';

@Injectable({
    'providedIn': 'root'
})
export class SocketCommunicationService {

    private socket: Socket;

    private logsOpen: Subject<string> = new Subject();

    private state: Subject<StateType> = new Subject();

    constructor () {

        this.socket = io(BACKEND_URL);
        this.initSocketSubscription();

    }


    identify (robot: RobotTargetType) {

        this.emit('identify', robot);

    }

    startMission (robot: RobotTargetType) {

        console.log('start mission', robot);
        this.emit('start-mission', robot);

    }

    stopMission (robot: RobotTargetType) {


        this.emit('stop-mission', robot);

    }

    showLog (missionNumber: number) {

        this.emit('get-all-logs', missionNumber);

    }

    sendLimoIps (limo1: string, limo2: string) {

        this.emit('send-limo-ips', {limo1, limo2});

    }

    getSubscribeOpenLogs () {

        return this.logsOpen.asObservable();

    }

    get subscribeState (){

        return this.state.asObservable();
    }
 
    private initSocketSubscription () {

        this.socket.on('connect', () => {

            this.socket.on('send-all-logs', (logs: string) => {

                this.logsOpen.next(logs);

            });

            this.socket.on('send-state', (state: StateType) => {
                this.state.next(state);
                console.log("ICI JE RECOIS L'ETAT DANS CLIENT-INTERFACE")
                console.log(state);

            });

            this.socket.on('reconnect', () => {
                window.location.reload();
            });
        });

    }

    private emit<T> (event: string, data?: T) {

        this.socket.emit('save-log', {
            event,
            'data': data ? data : ''
        });

        data ? this.socket.emit(event, data) : this.socket.emit(event);

    }

}
