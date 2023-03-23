import { Component, OnDestroy } from '@angular/core';
import { MatDialog, MatDialogRef } from '@angular/material/dialog';
import { Subscription } from 'rxjs';
import { LogsDialogComponent } from 'src/app/dialogs/logs-dialog/logs-dialog.component';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';
const FIRST_MISSION = 1;

@Component({
    'selector': 'app-log-action',
    'templateUrl': './log-action.component.html',
    'styleUrls': ['./log-action.component.scss']
})
export class LogActionComponent implements OnDestroy {

    missionNumber: number = FIRST_MISSION;

    private openLogsSubscription: Subscription;

    private ref: MatDialogRef<LogsDialogComponent> | undefined;

    constructor (
      private socketCommunication : SocketCommunicationService,
      private matDialogLogsOpen: MatDialog
    ) {

        this.openLogsSubscription = this.socketCommunication.getSubscribeOpenLogs().subscribe(this.openLogsDialog.bind(this));

    }

    ngOnDestroy () {

        this.openLogsSubscription.unsubscribe();

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
