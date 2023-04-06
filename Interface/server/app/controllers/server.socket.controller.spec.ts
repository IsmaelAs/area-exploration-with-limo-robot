import { Server as SocketServer } from 'socket.io';
import { Logger } from '../services/logger';
import { ClientSocketLimo } from './client.socket.limo';
import { ServerSocketController } from './server.socket.controller';
import * as sinon from 'sinon';
import chai, { expect } from 'chai';
import sinonChai from 'sinon-chai';
import StateLimo from '@app/interfaces/state-limo';
import RobotTargetType from '@app/types/RobotType';

chai.use(sinonChai);

describe('ServerSocketController', () => {
  let io: SocketServer;
  let logger: Logger;
  let serverSocketController: ServerSocketController;
  let socketLimo: ClientSocketLimo;
  let socketLimo2: ClientSocketLimo;
  let ioOnStub: sinon.SinonStub;

  beforeEach(() => {
    ioOnStub = sinon.stub();
    io = (sinon.createStubInstance(SocketServer) as unknown as SocketServer);
    (io as any).on = ioOnStub; // Replace the 'on' method with our custom stub
    logger = sinon.createStubInstance(Logger) as unknown as Logger;
    serverSocketController = new ServerSocketController(io, logger);
    socketLimo = sinon.createStubInstance(ClientSocketLimo) as unknown as ClientSocketLimo;
    socketLimo2 = sinon.createStubInstance(ClientSocketLimo) as unknown as ClientSocketLimo;
  });

  afterEach(() => {
    sinon.restore();
  });
    it('should call io.on method with "connection" event', () => {
      serverSocketController.initializeSocketServer();

      expect(ioOnStub).to.have.been.calledWith('connection');
    });


    it('should emit event to socketLimo if robotTarget is "limo-1" and socketLimo exists', () => {
      serverSocketController['socketLimo'] = socketLimo;

      serverSocketController['sendEventToLimo']('limo-1', 'test-event');

      expect(socketLimo.emitToLimo).to.have.been.calledWith('test-event');
    });

    it('should emit event with data to socketLimo if robotTarget is "limo-1" and socketLimo exists', () => {
      serverSocketController['socketLimo'] = socketLimo;

      serverSocketController['sendEventToLimo']('limo-1', 'test-event', { testData: 'test' });

      expect(socketLimo.emitToLimo).to.have.been.calledWith('test-event', { testData: 'test' });
    });

    it('should emit event to socketLimo2 if robotTarget is "limo-2" and socketLimo2 exists', () => {
      serverSocketController['socketLimo2'] = socketLimo2;

      serverSocketController['sendEventToLimo']('limo-2', 'test-event');

      expect(socketLimo2.emitToLimo).to.have.been.calledWith('test-event');
    });

    it('should emit event with data to socketLimo2 if robotTarget is "limo-2" and socketLimo2 exists', () => {
      serverSocketController['socketLimo2'] = socketLimo2;

      serverSocketController['sendEventToLimo']('limo-2', 'test-event', { testData: 'test' });

      expect(socketLimo2.emitToLimo).to.have.been.calledWith('test-event', { testData: 'test' });
    });

      it('should call io.on method with "connection" event and attach event listeners', () => {

        let io: any;
        io = {
          on: sinon.stub(),
        };
        
        let socketStub:any;

        socketStub = {
          on: sinon.stub(),
          disconnect: sinon.stub(),
        };
        const logger = sinon.createStubInstance(Logger) as unknown as Logger;
        const serverSocketController = new ServerSocketController(io, logger);

        //const sendEventToLimoSpy = sinon.spy(serverSocketController, <any>'sendEventToLimo');
        //const data: RobotTargetType = "limo-1";

        const connectionCallbackStub = sinon.stub();

        io.on.onCall(0).callsFake((eventName: string, callback: any) => {
          connectionCallbackStub.callsFake(callback);
        });
        serverSocketController.initializeSocketServer();
        expect(io.on.getCall(0).args[0]).to.equal('connection');
        connectionCallbackStub(socketStub);
      
        // Check that event listeners were attached to the socket object
        expect(socketStub.on.calledWith('identify')).to.be.true;
        expect(socketStub.on.calledWith('start-mission')).to.be.true;
        expect(socketStub.on.calledWith('stop-mission')).to.be.true;
        expect(socketStub.on.calledWith('save-log')).to.be.true;
        expect(socketStub.on.calledWith('get-all-logs')).to.be.true;
        expect(socketStub.on.calledWith('send-limo-ips')).to.be.true;
        expect(socketStub.on.calledWith('disconnect')).to.be.true;



        
      });


        it('should call stopMission on logger and socketLimo instances', () => {
          serverSocketController['socketLimo'] = socketLimo;
          serverSocketController['socketLimo2'] = socketLimo2;

          serverSocketController['stopMission']();

          expect(logger.stopMission).to.have.been.calledOnce;
          expect(socketLimo.stopMission).to.have.been.calledOnce;
          expect(socketLimo2.stopMission).to.have.been.calledOnce;
        });

        it('should call startMission on logger and socketLimo instances', () => {
          serverSocketController['socketLimo'] = socketLimo;
          serverSocketController['socketLimo2'] = socketLimo2;

          serverSocketController['startMission']();

          expect(logger.startMission).to.have.been.calledOnce;
          expect(socketLimo.startMission).to.have.been.calledOnce;
          expect(socketLimo2.startMission).to.have.been.calledOnce;
        });

        it('should call sendEventToFrontend with "send-state" event and data', () => {
          //const sendEventToLimo = ServerSocketControllerModule.__get__('sendEventToLimo');
          const sendEventToFrontendSpy = sinon.spy(serverSocketController, <any>'sendEventToFrontend');
          const data: StateLimo = { limoId: 1, state:''};

          serverSocketController['sendStateToClient'](data);

          expect(sendEventToFrontendSpy).to.have.been.calledWith('send-state', data);
        });

        it('should call io.emit with event and data', () => {
          const event = 'test-event';
          const data = { testData: 'test' };

          serverSocketController['sendEventToFrontend'](event, data);

          expect(io.emit).to.have.been.calledWith(event, data);
        });

        it('should call io.emit with event only', () => {
          const event = 'test-event';

          serverSocketController['sendEventToFrontend'](event);

          expect(io.emit).to.have.been.calledWith(event);
        });

    describe('Socket events', () => {
      let socketStub: any;
      let sendEventToLimoSpy: sinon.SinonSpy;
      let connectionCallback: any;
    
      beforeEach(() => {
        socketStub = {
          on: sinon.stub(),
          disconnect: sinon.stub(),
        };
    
        // Custom stub for io.on
        (io as any).on = (event: string, callback: any) => {
          if (event === 'connection') {
            connectionCallback = callback;
          }
        };
    
        sendEventToLimoSpy = sinon.spy(serverSocketController, <any>'sendEventToLimo');
        serverSocketController.initializeSocketServer();
    
        connectionCallback(socketStub);
        socketLimo = sinon.createStubInstance(ClientSocketLimo) as unknown as ClientSocketLimo;
    socketLimo2 = sinon.createStubInstance(ClientSocketLimo) as unknown as ClientSocketLimo;
      });

      afterEach(() => {
        sendEventToLimoSpy.restore();
        sinon.restore();
      });
    
      it('should call sendEventToLimo when "identify" event is received', () => {
        const data: RobotTargetType = 'limo-1';
        const identifyCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'identify').args[1];
        identifyCallback(data);
    
        expect(sendEventToLimoSpy).to.have.been.calledWith(data, 'identify');
      });
    
      it('should call sendEventToLimo when "start-mission" event is received', () => {
        serverSocketController["isMissionStarted"] = false;
        const data: RobotTargetType = 'limo-1';
        const startMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'start-mission').args[1];
        startMissionCallback(data);
    
        expect(sendEventToLimoSpy).to.have.been.calledWith(data, 'start-mission');
      });
      it('should call startMission when "start-mission" event is received and isMissioStarted is false', () => {
        serverSocketController["isMissionStarted"] = false;
        const data: RobotTargetType = 'limo-1';
        const startMissionSpy = sinon.spy(serverSocketController, <any>"startMission");
        const startMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'start-mission').args[1];
        startMissionCallback(data);
    
        expect(startMissionSpy).to.have.been.called;
      });
      it('should not call startMission when "start-mission" event is received and isMissioStarted is true', () => {
        serverSocketController["isMissionStarted"] = true;
        const data: RobotTargetType = 'limo-1';
        const startMissionSpy = sinon.spy(serverSocketController, <any>"startMission")
        const startMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'start-mission').args[1];
        startMissionCallback(data);
    
        expect(startMissionSpy).to.not.have.been.called;
      });
      it('should call stopMission when "stop-mission" event is received and isMissionStarted is false', () => {
        serverSocketController["isMissionStarted"] = true;
        const data: RobotTargetType = 'limo-1';
        const stopMissionSpy = sinon.spy(serverSocketController, <any>"stopMission");
        const stopMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'stop-mission').args[1];
        stopMissionCallback(data);
    
        expect(stopMissionSpy).to.have.been.called;
      });
      it('should not call startMission when "start-mission" event is received and isMissionStarted is true', () => {
        serverSocketController["isMissionStarted"] = false;
        const data: RobotTargetType = 'limo-1';
        const stopMissionSpy = sinon.spy(serverSocketController, <any>"stopMission")
        const stopMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'stop-mission').args[1];
        stopMissionCallback(data);
    
        expect(stopMissionSpy).to.not.have.been.called;
      });
    
      it('should call sendEventToLimo when "stop-mission" event is received', () => {
        const data: RobotTargetType = 'limo-1';
        const stopMissionCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'stop-mission').args[1];
        stopMissionCallback(data);
    
        expect(sendEventToLimoSpy).to.have.been.calledWith(data, 'stop-mission');
      });
      it('should call logger.saveUserData when "save-log" event is received', () => {
        const saveLogData = { test: 'data' };
        const saveLogCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'save-log').args[1];
        saveLogCallback(saveLogData);
    
        expect(logger.saveUserData).to.have.been.calledWith(saveLogData);
      });
    
      it('should call logger.getAllData when "get-all-logs" event is received', () => {
        const missionNumber = 1;
        const getAllLogsCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'get-all-logs').args[1];
        getAllLogsCallback(missionNumber);
    
        expect(logger.getAllData).to.have.been.calledWith(missionNumber, socketStub);
      });
    
      it('should create new ClientSocketLimo instances when "send-limo-ips" event is received', () => {
        const ips = { limo1: '192.168.1.1', limo2: '192.168.1.2' };
        const sendLimoIpsCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'send-limo-ips').args[1];
        sendLimoIpsCallback(ips);
    
        expect(serverSocketController["socketLimo"]).to.be.instanceOf(ClientSocketLimo);
        expect(serverSocketController["socketLimo2"]).to.be.instanceOf(ClientSocketLimo);
      });
    
      it('should call sendEventToLimo with "stop-mission', () => {
        serverSocketController['socketLimo'] = socketLimo;
        serverSocketController['socketLimo2'] = socketLimo2;

        serverSocketController["clientCounter"] = 0;
        const disconnectCallback = socketStub.on.getCalls().find((call: { args: string[]; }) => call.args[0] === 'disconnect').args[1];
        disconnectCallback();
        expect(sendEventToLimoSpy).to.have.been.calledWith('robots', 'stop-mission');
        expect(socketLimo.disconnect).to.have.been.calledOnce;
        expect(socketLimo2.disconnect).to.have.been.calledOnce;
    });
    });
    });
