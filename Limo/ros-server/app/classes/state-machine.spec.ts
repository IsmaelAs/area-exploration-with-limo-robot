import { MyStateMachine } from './state-machine';
import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import { expect } from "chai";

describe('MyStateMachine Unittest', () => {
  let stateMachine: MyStateMachine;

  beforeEach(() => {
    stateMachine = new MyStateMachine();
  });

  afterEach(() => {
    sinon.restore()
  })


  it('setLimoId', () => {
    stateMachine.setLimoId(5)
    expect(stateMachine["limoId"]).to.deep.equal(5);
  });


  it('should initialize with the "INIT" state', () => {
    expect(stateMachine["currentState"]).to.deep.equal('INIT');
  });

  it('should change the current state when we call onMission', () => {
    stateMachine["currentState"] = 'WAITING'
    stateMachine.onMission()
    expect(stateMachine["currentState"]).to.deep.equal('ON_MISSION');
  });

  it('should set the current state for ON_MISSION when we call onMission', () => {
    stateMachine["currentState"] = 'ON_MISSION'
    stateMachine.onMission()
    expect(stateMachine["currentState"]).to.deep.equal('ON_MISSION');
  });

  it('should set the current state on WAITING when we call onReady', () => {
    stateMachine.onReady()
    expect(stateMachine["currentState"]).to.deep.equal('WAITING');
  });

  it('should modify the current state and log a message when we call onMissionEnd', () => {
    const consoleSpy = sinon.spy(console, 'log')
    stateMachine.onMissionEnd()
    expect(consoleSpy.calledWith('ICI JE MET FIN A LA MISSION-STOPPED')).to.be.true
    expect(stateMachine["currentState"]).to.deep.equal('STOPPED');
  });

  it('should call clearInterval when we call stopStates', () => {
    const spyOn = sinon.spy(global, 'clearInterval');
    stateMachine.stopStates()
    expect(spyOn.called).to.be.true;
  });

  it('should call asObservable when we call stateObservable', () => {
    const spyOn = sinon.spy(stateMachine["statesObservable"], "asObservable")
    stateMachine.stateObservable
    expect(spyOn.called).to.be.true;
  });

  it('should call next when we call callback', () => {
    const spyOn = sinon.spy(stateMachine["statesObservable"], "next")
    stateMachine["callBack"]()
    expect(spyOn.called).to.be.true;
  });

  it('should set intervalState when we call startStates', () => {
    expect(stateMachine["intervalState"]).to.be.undefined
    stateMachine.startStates()
    expect(stateMachine["intervalState"]).to.be.not.undefined
    stateMachine.stopStates()

  });
});
