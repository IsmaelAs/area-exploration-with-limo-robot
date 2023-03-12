import { Component, Inject } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';

@Component({
  selector: 'app-logs-dialog',
  templateUrl: './logs-dialog.component.html',
  styleUrls: ['./logs-dialog.component.scss']
})
export class LogsDialogComponent {
    constructor(@Inject(MAT_DIALOG_DATA) public data: {logs: string},
    private dialogRef: MatDialogRef<LogsDialogComponent>) {}

    get logsFormatted() {
      return this.data.logs
    }

    close() {
      this.dialogRef.close()
    }
}
