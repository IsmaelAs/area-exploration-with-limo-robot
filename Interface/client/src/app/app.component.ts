import { Component, OnDestroy } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';
import RobotTargetType from './types/RobotType';
import { MatDialog, MatDialogRef } from '@angular/material/dialog';
import { LogsDialogComponent } from './dialogs/logs-dialog/logs-dialog.component';
import { Subscription } from 'rxjs';

const FIRST_MISSION = 1;

@Component({
    'selector': 'app-root',
    'templateUrl': './app.component.html',
    'styleUrls': ['./app.component.scss']
})
export class AppComponent implements OnDestroy {

    list: RobotTargetType[] = [
        'limo-1',
        'limo-2',
        'robots'
    ];

    robotTarget: RobotTargetType = 'limo-1';

    missionNumber = FIRST_MISSION;

    private openLogsSubscription: Subscription;

    private ref: MatDialogRef<LogsDialogComponent> | undefined;

    constructor (
    // eslint-disable-next-line no-unused-vars
    private socketCommunication : SocketCommunicationService,
    private matDialogLogsOpen: MatDialog
    ) {

        this.openLogsSubscription = socketCommunication.subscribeOpenLogs.subscribe(this.openLogsDialog.bind(this));

    }

    ngOnDestroy () {

        this.openLogsSubscription.unsubscribe();

    }


    identify () {

        this.socketCommunication.identify(this.robotTarget);

    }

    startMission () {

        this.socketCommunication.startMission(this.robotTarget);

    }

    stopMission () {

        this.socketCommunication.stopMission(this.robotTarget);

    }


    setTarget (choice: RobotTargetType) {

        this.robotTarget = choice;

    }

    showLog () {

        this.socketCommunication.showLog(this.missionNumber);

    }

    private openLogsDialog (logs: string) {

        if (this.ref) {

            this.ref.close();

        }
        this.ref = this.matDialogLogsOpen.open(LogsDialogComponent, {
            'data': {
                logs
            }
        });

    }

}

