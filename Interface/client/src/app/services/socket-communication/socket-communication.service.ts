import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from 'src/app/constants/url';
import RobotTargetType from 'src/app/types/RobotType';
import { DISTANCE_MOVEMENT, DIRECTION_MOVEMENT } from 'src/app/constants/robots-movement';
import RobotMovement from 'src/app/interfaces/robots-movement-interface';

@Injectable({
  providedIn: 'root'
})
export class SocketCommunicationService {

  private socket: Socket;

  constructor() {    
    this.socket = io(BACKEND_URL);
    this.initSocketSubscription()
  }

  identify(robot: RobotTargetType){
    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.LEFT_FORWARD,
      distance: DISTANCE_MOVEMENT.FAR_AWAY
    }
    
    this.emit("identify", movement);
  }

  startMission(robot: RobotTargetType) {

    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.FORWARD,
      distance: DISTANCE_MOVEMENT.CLOSE
    }
    console.log('start mission', movement);
    this.emit("start-mission", movement)
  }
  
  stopMission(robot: RobotTargetType) {
    const movement: RobotMovement = {
      robot: robot,
      direction: DIRECTION_MOVEMENT.BACKWARD,
      distance: DISTANCE_MOVEMENT.CLOSE
    }

    this.emit("stop-mission", movement)
  }

  private initSocketSubscription() {
    this.socket.on("connect", () => {
      this.socket.on("send-all-logs", (logs: string) => {
        console.log(logs);
      })
    })
  }

  private emit<T>(event: string, data?: T) {
    data ? this.socket.emit(event, data) : this.socket.emit(event)

    this.socket.emit('save-log', {
      event: event,
      data: data? data : ""
    })
  }
}
