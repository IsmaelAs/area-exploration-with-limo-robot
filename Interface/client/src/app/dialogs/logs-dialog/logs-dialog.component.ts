import { Component, Inject, OnInit } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';

@Component({
    'selector': 'app-logs-dialog',
    'templateUrl': './logs-dialog.component.html',
    'styleUrls': ['./logs-dialog.component.scss']
})
export class LogsDialogComponent implements OnInit {

    constructor (@Inject(MAT_DIALOG_DATA) public data: {logs: string},
    private dialogRef: MatDialogRef<LogsDialogComponent>) {}

    get logsFormatted () {

        return this.data.logs;

    }

    ngOnInit (): void {

        this.dialogRef.updateSize('200 px', '300 px');

    }

    close () {

        this.dialogRef.close();

    }

}
