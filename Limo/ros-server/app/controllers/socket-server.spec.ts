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
//   let socketStub: sinon.SinonStub;
//   let loggerStub: sinon.SinonStub;
//   let stateMachineStub: sinon.SinonStub;
//   let sandbox: sinon.SinonSandbox;
  let socketServer: SocketServer;
  beforeEach(() => {
    server = sinon.createStubInstance(Server);
    nodeManager = sinon.createStubInstance(NodeManager);
    logger = sinon.createStubInstance(Logger);
    // sandbox = sinon.createSandbox();
    socketServer = new SocketServer(server,nodeManager, logger)
    // socketStub = sandbox.stub(server, 'on');
    // loggerStub = sandbox.stub(logger, 'startLogs');
    // stateMachineStub = sandbox.stub(MyStateMachine.prototype, 'startStates');
  });

  afterEach(() => {
    sinon.restore();
  });


it("emit", () => {
    socketServer.emit("1")
    expect(server.emit.called).to.be.true
})

// it("should set send logs when we call sendLogs", () => {
//     const mockLog: LogType = {} as unknown as LogType
//     socketServer["sendLogs"](mockLog)
//     expect(server.emit.called).to.be.true

})
    


