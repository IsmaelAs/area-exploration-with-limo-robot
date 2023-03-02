import { Component, OnDestroy } from '@angular/core';
import { SocketCommunicationService } from './services/socket-communication/socket-communication.service';
import RobotTargetType from './types/RobotType';
import { MatDialog, MatDialogRef } from "@angular/material/dialog"
import { LogsDialogComponent } from './dialogs/logs-dialog/logs-dialog.component';
import {  Subscription } from 'rxjs';
import { exec } from 'child_process';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.scss']
})
export class AppComponent  implements OnDestroy {

  list: RobotTargetType[] = ["limo-1", "limo-2", "robots"];
  type: RobotTargetType = "limo-1";
  missionNumber: number = 1;
  private openLogsSubscription: Subscription;
  private ref: MatDialogRef<LogsDialogComponent> | undefined;
  
  constructor(
    private socketCommunication : SocketCommunicationService,
    private matDialogLogsOpen: MatDialog,
    ){
      this.openLogsSubscription = socketCommunication.subscribeOpenLogs.subscribe(this.openLogsDialog.bind(this))
    }
  
  
  ngOnDestroy () {
    this.openLogsSubscription.unsubscribe()
  }

  identify(){    
    this.socketCommunication.identify(this.type);
  }

  startMission() {
    this.socketCommunication.startMission(this.type)
  }

  stopMission() {
    this.socketCommunication.stopMission(this.type)
  }

  launchSimulation(): void {
    const commandPath = 'prj_ws'; // dossier ou se trouve la simu

    exec('source devel/setup.bash', { cwd: commandPath }, (error, stdout, stderr) => {
      if (error) {
        console.error(`exec erreur: ${error}`);
        return;
      }
      console.log(`stdout: ${stdout}`);
      console.error(`stderr: ${stderr}`);
    });

    exec('roslaunch limo_gazebo_sim limo_ackerman.launch &', { cwd: commandPath }, (error, stdout, stderr) => {
      if (error) {
        console.error(`exec erreur: ${error}`);
        return;
      }
      console.log(`stdout: ${stdout}`);
      console.error(`stderr: ${stderr}`);
    });
  }

  setType(choice: RobotTargetType) { 
    this.type = choice;
  }


  showLog() {
    this.socketCommunication.showLog(this.missionNumber)
  }

  private openLogsDialog(logs: string) {
    if (this.ref) this.ref.close()
    this.ref = this.matDialogLogsOpen.open(LogsDialogComponent, {
      data: {
        logs
      }
    })
  }

}

