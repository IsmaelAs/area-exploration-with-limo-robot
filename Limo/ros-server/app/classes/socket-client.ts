// import { URL_NODE_SERVER } from '../constants/url';
import  { Socket } from 'socket.io-client';
import { NodeManager } from './nodes-manager';


export class SocketClient {
  private socket: Socket;
  private nodeManager: NodeManager;

  constructor() {
    this.nodeManager = new NodeManager();
  }

  connect() {
    this.nodeManager.start();

    this.socket.on('connect', () => {
      console.log('Connected to node server');
      this.socket.emit('request');
    });

    this.socket.on('response', (command) => {
      console.log(`Received response from node server: ${command}`);
      this.nodeManager.move(command)
    });

    this.socket.on('disconnect', () => {
      console.log('Disconnected from limo robot');
      this.nodeManager.stop();
    });
  }
}
