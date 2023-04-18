import { Logger } from '../services/logger';
import { NodeManager } from '../classes/nodes-manager';
import { Server } from 'socket.io';

import { MyStateMachine } from '../classes/state-machine';

import sinon, { SinonSandbox, SinonStub } from 'sinon';
import { expect } from 'chai';

describe('SocketServer', () => {
  let server: Server;
  let nodeManager: NodeManager;
  let logger: Logger;
  let socketStub: SinonStub;
  let loggerStub: SinonStub;
  let stateMachineStub: SinonStub;
  let sandbox: SinonSandbox;

  beforeEach(() => {
    server = {} as Server;
    nodeManager = {} as NodeManager;
    logger = {} as Logger;
    sandbox = sinon.createSandbox();

    socketStub = sandbox.stub(server, 'on');
    loggerStub = sandbox.stub(logger, 'startLogs');
    stateMachineStub = sandbox.stub(MyStateMachine.prototype, 'startStates');
  });

  afterEach(() => {
    sandbox.restore();
  });

  describe('connectSocketServer', () => {
    it('should start nodes', () => {
      socketServer.connectSocketServer();
      expect(nodeManager.startNodes.calledOnce).to.be.true;
    });

    it('should subscribe to stateObservable', () => {
      const subscribeStub = sandbox.stub();
      const stateObservable = { subscribe: subscribeStub };
      sandbox.stub(MyStateMachine.prototype, 'stateObservable').get(() => stateObservable);

      socketServer.connectSocketServer();
      expect(subscribeStub.calledOnce).to.be.true;
    });

    it('should call startStates on MyStateMachine instance', () => {
      socketServer.connectSocketServer();
      expect(stateMachineStub.calledOnce).to.be.true;
    });

    describe('on login event', () => {
      let loginCallback: (limoId: number) => void;

      beforeEach(() => {
        socketServer.connectSocketServer();
        loginCallback = socketStub.getCall(0).args[1];
      });

      it('should set limoId on instance', () => {
        loginCallback(1);
        expect(socketServer.limoId).to.equal(1);
      });

      it('should call setLimoId on stateMachine', () => {
        const setLimoIdStub = sandbox.stub(MyStateMachine.prototype, 'setLimoId');
        loginCallback(1);
        expect(setLimoIdStub.calledWith(1)).to.be.true;
      });

      it('should subscribe to stateObservable', () => {
        const subscribeStub = sandbox.stub();
        const stateObservable = { subscribe: subscribeStub };
        sandbox.stub(MyStateMachine.prototype, 'stateObservable').get(() => stateObservable);

        loginCallback(1);
        expect(subscribeStub.calledOnce).to.be.true;
      });
    });
    });
    });
    
    


