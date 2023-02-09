import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { BACKEND_URL } from 'src/app/constants/url';
import RobotTargetType from 'src/app/types/robots';


@Injectable({
  providedIn: 'root'
})
export class SocketCommunicationService {

  private socket: Socket;

  constructor() {
    this.socket = io(BACKEND_URL);
  }

  advance(robot: RobotTargetType){
    this.socket.emit(`${robot}-advance`);
  }

  startMission(robot: RobotTargetType) {
    this.socket.emit(`${robot}-start-mission`)
  }
  
  stopMission(robot: RobotTargetType) {
    this.socket.emit(`${robot}-stop-mission`)
  }
}
