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

    private p2pConnected: Subject<boolean> = new Subject();


    constructor () {

        this.socket = io(BACKEND_URL);
        this.initSocketSubscription();

    }

    updateLimo (limo: RobotTargetType) {
        this.emit('update', limo);
    }

    startP2P () {

        this.emit('p2p-start');

    }


    identify (robot: RobotTargetType) {

        this.emit('identify', robot);

    }

    startMission (robot: RobotTargetType) {

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

    get subscribeState () {

        return this.state.asObservable();
    }

    get subscribeP2PState () {

        return this.p2pConnected.asObservable();
    }

    private initSocketSubscription () {

        this.socket.on('connect', () => {

            this.socket.on('send-all-logs', (logs: string) => {

                this.logsOpen.next(logs);

            });

            this.socket.on('send-state', (state: StateType) => {
                this.state.next(state);

            });

            this.socket.on('p2p-connected', (isConnected: boolean) => {
                this.p2pConnected.next(isConnected);
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
