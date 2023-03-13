import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from 'src/app/constants/url';
import RobotTargetType from 'src/app/types/RobotType';
import { DISTANCE_MOVEMENT, DIRECTION_MOVEMENT } from 'src/app/constants/robots-movement';
import RobotMovement from 'src/app/interfaces/robots-movement-interface';
import { State } from 'src/app/types/States';
import { Subject } from 'rxjs';

@Injectable({
    'providedIn': 'root'
})
export class SocketCommunicationService {

    private socket: Socket;

    private logsOpen: Subject<string> = new Subject();
    private receivedState: Subject<string> = new Subject();

    constructor () {

        this.socket = io(BACKEND_URL);
        this.initSocketSubscription();

    }


    identify (robot: RobotTargetType) {

        const movement: RobotMovement = {
            robot,
            'direction': DIRECTION_MOVEMENT.LEFT_FORWARD,
            'distance': DISTANCE_MOVEMENT.FAR_AWAY
        };

        this.emit('identify', movement);

    }

    startMission (robot: RobotTargetType) {

        const movement: RobotMovement = {
            robot,
            'direction': DIRECTION_MOVEMENT.BACKWARD,
            'distance': DISTANCE_MOVEMENT.CLOSE
        };

        console.log('start mission', movement);
        this.emit('start-mission', movement);

    }

    stopMission (robot: RobotTargetType) {

        const movement: RobotMovement = {
            robot,
            'direction': DIRECTION_MOVEMENT.BACKWARD,
            'distance': DISTANCE_MOVEMENT.CLOSE
        };

        this.emit('stop-mission', movement);

    }

    showLog (missionNumber: number) {

        this.emit('get-all-logs', missionNumber);

    }

    get subscribeOpenLogs () {

        return this.logsOpen.asObservable();

    }

    get subscribeStates () {

      return this.receivedState.asObservable();

    }

    private initSocketSubscription () {

        this.socket.on('connect', () => {

            this.socket.on('send-all-logs', (logs: string) => {
                console.log(logs);
                this.logsOpen.next(logs);

            });

            this.socket.on('send-state', (state: State) => {

                console.log(state);
                this.receivedState.next(state);
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
