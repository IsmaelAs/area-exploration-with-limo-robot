// import { MyStateMachine } from './state-machine';
// import { SocketServer } from './socket-server';

// describe('MyStateMachine', () => {
//   let socketServer: SocketServer;
//   let stateMachine: MyStateMachine;

//   beforeEach(() => {
//     stateMachine = new MyStateMachine(socketServer);
//   });

//   it('should initialize with the "INIT" state', () => {
//     expect(stateMachine.currentState).toBe('INIT');
//   });

//   it('should update state to "WAITING" when battery level is above 30%', () => {
//     const percentage = 50;
//     stateMachine.onLowBattery(percentage);
//     expect(stateMachine.currentState).toBe('WAITING');
//   });

//   it('should update state to "STOPPED" when battery level is below 30%', () => {
//     const percentage = 20;
//     stateMachine.onLowBattery(percentage);
//     expect(stateMachine.currentState).toBe('STOPPED');
//   });

//   it('should update state to "ON_MISSION" when onMission() method is called', () => {
//     stateMachine.onMission();
//     expect(stateMachine.currentState).toBe('ON_MISSION');
//   });

//   it('should update state to "STOPPED" when onMissionEnd() method is called', () => {
//     stateMachine.onMissionEnd();
//     expect(stateMachine.currentState).toBe('STOPPED');
//   });
// });
