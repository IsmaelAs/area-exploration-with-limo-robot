import { NodeManager } from './nodes-manager';
import Command from '../types/Command';
import { Server } from 'socket.io';


export class SocketServer {
  private server: Server;
  private nodeManager: NodeManager;

  constructor(server: Server) {
    this.nodeManager = new NodeManager();
    this.server = server
  }
  
  // Connect the socket to the limo node server ; subscribe to all limo command ; start all nodes
  connectSocketServer(): void {
    this.server.on('connection', (socket) => {
      console.log('Connected to node server');

      // Start all nodes when socket is connected
      this.nodeManager.start();

      socket.on(`limo-${process.env.LIMO_ID}-move`, async (movement: {direction: Command, distance?: number}) => {
        console.log(`Received response from node server: ${movement.direction}`);
        await this.nodeManager.move(movement.direction, movement.distance)
      });

      socket.on('error', (err: Error) => {
        console.log(`Socket Client Error : ${err.stack}`);
        this.nodeManager.stop()
        this.server.removeAllListeners()
      })

      socket.on('disconnect', () => {
        console.log('Disconnected from limo robot');
        this.nodeManager.stop();
        this.server.removeAllListeners()

      });
    });
  }
}
