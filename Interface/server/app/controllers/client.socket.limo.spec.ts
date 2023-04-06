
import { expect } from 'chai';
import * as sinon from 'sinon';
import { ClientSocketLimo } from './client.socket.limo';
//import { Socket } from 'socket.io';


export class RosMock {
    url: string;

    constructor(data: {url: string}) {
        this.url = data.url;
    }

    on(event: string, callBack: () => void) {
        if (event === 'connection') callBack();
    }

    close() {
    // Pass
    }
}

describe('ClientSocketLimo', () => {
    describe('ClientSocketLimo', () => {
        let clientSocketLimo: ClientSocketLimo;
        let socketStub: any;
      
        beforeEach(() => {
          socketStub = {
            on: sinon.stub(),
            emit: sinon.stub(),
            removeAllListeners: sinon.stub(),
          };
          clientSocketLimo = new ClientSocketLimo(1, 'limoUrl');
          clientSocketLimo["socket"] = socketStub;
        });
      
        it('should listen to socket events', () => {
          clientSocketLimo.connectClientSocketToLimo();
      
          expect(socketStub.on.calledWith('connect')).to.be.true;
          expect(socketStub.on.calledWith('save-log')).to.be.true;
          expect(socketStub.on.calledWith('save-state')).to.be.true;
          expect(socketStub.on.calledWith('disconnect')).to.be.true;
        });
      
        it('should emit login event on connect', () => {
          clientSocketLimo.connectClientSocketToLimo();
      
          const connectCallback = socketStub.on.getCall(0).args[1];
          connectCallback();
      
          expect(socketStub.emit.calledWith('login', 1)).to.be.true;
        });
      
        it('should remove all listeners on disconnect', () => {
          clientSocketLimo.connectClientSocketToLimo();
      
          const disconnectCallback = socketStub.on.getCall(3).args[1];
          disconnectCallback();
      
          expect(socketStub.removeAllListeners.called).to.be.true;
        });
      });
      
    describe('connectClientSocketToLimo', () => {
      let clientSocketLimo: ClientSocketLimo;

      beforeEach(() => {
        clientSocketLimo = new ClientSocketLimo(1, 'http://localhost:3000');
      });
  
      afterEach(() => {
        sinon.restore();
        clientSocketLimo.disconnect();
      });

      it("should create", () => {
        expect(clientSocketLimo).to.exist
      });
  
      it('should emit an event to the limo with data', () => {
        const emitStub = sinon.stub(clientSocketLimo["socket"], 'emit');
        const eventData = {eventData: 'some data'};
        clientSocketLimo.emitToLimo('some-event', eventData);
        expect(emitStub.calledWith('some-event', eventData)).to.be.true;
      });
  
      it('should emit an event to the limo without data', () => {
        const emitStub = sinon.stub(clientSocketLimo["socket"], 'emit');
        clientSocketLimo.emitToLimo('some-other-event');
        expect(emitStub.calledWith('some-other-event')).to.be.true;
      });
    });
  });

