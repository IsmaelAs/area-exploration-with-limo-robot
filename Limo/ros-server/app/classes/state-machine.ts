
import { BRIDGE_URI } from '@app/constants/url';
import { Ros, Topic } from 'roslib';
import { State } from '@app/types/States';
import { SocketServer } from './socket-server';
import { setInterval } from 'timers';


export class MyStateMachine {

 
 private currentState: State = "INIT";
 private ros: Ros;
 private socketServer: SocketServer;
 

 BatteryPercentage: any;
 onBatteryMessage: any;
 intervalState: any;



 Constructor(socketServer: SocketServer) {
   this.socketServer = socketServer
   // Connexion au serveur ROS
   this.ros = new Ros({
     url: BRIDGE_URI,
   });
 


     // Abonnement au topic "battery_state"
     const batterySubscriber = new Topic({
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
 


 StartStates() {
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


 OnMission() {
  this.currentState = "ON_MISSION";
 }

 onMissionEnd() {
  this.currentState = "STOPPED";
 }
}


