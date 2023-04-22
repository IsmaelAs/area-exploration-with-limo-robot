import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';
import * as sinon from "sinon";
import { expect } from 'chai';
import { SocketServer } from './socket-server';


describe('SocketServer', () => {
  let server: sinon.SinonStubbedInstance<Server<any, any, any, any>>;
  let nodeManager: sinon.SinonStubbedInstance<NodeManager>;
  let logger: sinon.SinonStubbedInstance<Logger>;

  let socketServer: SocketServer;
  beforeEach(() => {
    server = sinon.createStubInstance(Server);
    nodeManager = sinon.createStubInstance(NodeManager);
    logger = sinon.createStubInstance(Logger);
    socketServer = new SocketServer(server, nodeManager, logger)

  });

  afterEach(() => {
    sinon.restore();
  });


  it("emit", () => {
    socketServer.emit("1")
    expect(server.emit.called).to.be.true
  })

})



