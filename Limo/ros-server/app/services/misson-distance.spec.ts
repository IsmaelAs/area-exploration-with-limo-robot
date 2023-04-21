// import { describe, beforeEach } from "mocha";
// import * as sinon from 'sinon'
// import { expect } from "chai";
// import { MissionDistance } from "./mission-distance";
// import { Server } from 'socket.io';



// describe("Mission Distance  Unittest's", ()=> {
//     let missionDistance: MissionDistance
//     let server: Server

//     beforeEach(() => {
//         missionDistance = new MissionDistance(server)

//     })

//     afterEach(() => {
//         sinon.restore()
//     })

//     it("should work", () => {
//         expect(missionDistance).to.exist
//     })

//     it("should call closeNodePosition when stopMission called", () => {
//         const closeNodePositionSpy = sinon.spy(missionDistance["nodePosition"], 'closeNodePosition')
//         missionDistance.stopMission()
//         expect(closeNodePositionSpy.called).to.be.true
//     })

//     it("should call emitTotalDistance when stopMission called", () => {
//         const emitTotalDistanceSpy = sinon.spy((missionDistance as any), 'emitTotalDistance')
        
//         missionDistance.stopMission()
//         expect(emitTotalDistanceSpy.called).to.be.true
//     })

// })