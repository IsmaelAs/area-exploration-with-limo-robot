import { BRIDGE_URI } from '@app/constants/url';
import * as ROSLIB from 'roslib';
import { State } from '@app/types/States';
import { SocketServer } from './socket-server';
import { setInterval } from 'timers';

export class MyStateMachine {
  private currentState: State = "INIT";
  private ros: ROSLIB.Ros;
  private socketServer: SocketServer;
  
  batteryPercentage: any;
  onBatteryMessage: any;
  intervalState: any;

  constructor(socketServer: SocketServer) {
    this.socketServer = socketServer
    // Connexion au serveur ROS
    this.ros = new ROSLIB.Ros({
      url: BRIDGE_URI,
    });

      // Abonnement au topic "battery_state"
      const batterySubscriber = new ROSLIB.Topic({
        ros: this.ros,
        name: '/battery_state',
        messageType: 'sensor_msgs/BatteryState',
      });
    batterySubscriber.subscribe(this.callBack.bind(this));
    }

  private callBack(data: {percentage: number}) {
      this.onLowBattery(data.percentage)
      const limoId = this.socketServer.limoId
      this.socketServer.emit("save-state", {limoId: limoId, state: this.currentState})
  }

  startStates() {
    this.intervalState = setInterval(this.callBack, 1000)
  }


  onLowBattery(percentage: number): void {

    // Check if the battery is below 30%
    if (percentage < 30) {
      console.log('Battery level is below 30%');
      this.currentState = "STOPPED";
    } else if (this.currentState !== "ON_MISSION"  && this.currentState !== "WAITING") {
        this.currentState = "WAITING";
    } 

  }
      
  // execute() {
  //   setInterval (() => {
  //     switch (this.currentState) {
  //       case State.INIT:
  //         console.log('Initialisation');
  //         this.currentState = State.waiting;
  //         break;
  //       case State.waiting:
  //         console.log('En attente');
  //         or si fin de mission
  //         this.currentState = State.stopped;
  //         break;
  //       case State.onMission:
  //         console.log('En mission');
  //         if(onObstacle())
  //           this.currentState = State.stopped;
  //         break;
  //       case State.stopped:
  //         console.log('En arrêt');
  //         if (this.batteryPercentage >= 30)
  //         this.currentState = State.waiting;
  //         break;
  //       default:
  //         console.log(`Unknown state: ${this.currentState}`);
  //         break;
  //     }
  //   }, 1000);
  //}
  onMission() {
    this.currentState = "ON_MISSION";
  }
  onMissionEnd() {
    this.currentState = "STOPPED";
  }
 
}



