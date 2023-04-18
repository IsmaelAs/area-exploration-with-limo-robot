import NodeMap from '../classes/ros/nodes/node-map';
import { P2PPosition } from '../classes/p2p-position';
import { io, Socket } from 'socket.io-client';

const LIMO_ID = 2;

export class P2PSocketClient {
  private socket: Socket;

  private p2pUrl: string;

  private p2pActivated: boolean;

  private intervalPos: NodeJS.Timer;

  private intervalMap: NodeJS.Timer;

  private nodeMap: NodeMap = new NodeMap();

  private p2pPosition: P2PPosition = new P2PPosition(LIMO_ID);


  constructor(p2pUrl: string) {
    this.p2pActivated = false;
    this.p2pUrl = p2pUrl;
    this.socket = io(this.p2pUrl);

    this.nodeMap.initNodeMap();

    this.initP2P();
  }

  activateP2P() {
    this.p2pActivated = true;
    this.emit('p2p-activated');
  }

  deactivateP2P() {
    this.p2pActivated = false;
    this.emit('p2p-deactivated');
    clearInterval(this.intervalPos);
    clearInterval(this.intervalMap);
  }

  initP2P() {
    this.socket.on('connect', () => {
      this.socket.on('reconnect', () => {
        window.location.reload();
      });
      this.intervalPos = setInterval(this.callBackPos.bind(this), 1000);
      this.intervalMap = setInterval(this.callBackMap.bind(this), 500);
      this.socket.on('p2p-distance', (distance: number) => {
        this.p2pPosition.setP2PDistance(distance);
      });
    });
  }

  private callBackMap() {
    const map = this.nodeMap.getMap();
    console.log("Dans le call back Map de Limo2.... la Map est : ");
    console.log(map);
    if (map) this.emit('p2p-map', map);
  }

  private callBackPos() {
    const distance = this.p2pPosition.getDistance();
    console.log("Dans le call back Pos de Limo2.... la disstance est : " + distance)
    if (distance) this.emit('p2p-distance', distance);
  }


  private emit<T>(event: string, data?: T) {
    if (this.p2pActivated) data ? this.socket.emit(event, data) : this.socket.emit(event);
  }
}
