import { URL_NODE_SERVER } from '../constants/url';
import  { Socket, io } from 'socket.io-client';
import { NodeManager } from './nodes-manager';
import Command from '../types/Command';


export class SocketClient {
  private socket: Socket;
  private nodeManager: NodeManager;

  constructor() {
    this.nodeManager = new NodeManager();
    this.socket = io(URL_NODE_SERVER)
  }
  
  // Connect the socket to the limo node server ; subscribe to all limo command ; start all nodes
  connect(): void {
    this.socket.on('connect', () => {
      console.log('Connected to node server');

      // Start all nodes when socker is connected
      this.nodeManager.start();

      this.socket.on('move', (command: Command, nbrSendingMsg?: number) => {
        console.log(`Received response from node server: ${command}`);
        this.nodeManager.move(command, nbrSendingMsg)
      });

      this.socket.on('error', (err: Error) => {
        console.log(`Socket Client Error : ${err.stack}`);
        this.nodeManager.stop()
      })

      this.socket.on('disconnect', () => {
        console.log('Disconnected from limo robot');
        this.nodeManager.stop();

      });
    });
  }
}
