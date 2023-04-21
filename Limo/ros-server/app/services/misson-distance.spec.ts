import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import { expect } from "chai";
import { MissionDistance } from "./mission-distance";
import { Server } from 'socket.io';



describe("Mission Distance  Unittest's", ()=> {
    let missionDistance: MissionDistance
    let server: Server

    beforeEach(() => {
        missionDistance = new MissionDistance(server)

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(missionDistance).to.exist
    })

    //fake test
    it("startMission", async () => {
        const stubNodePosition = sinon.spy(missionDistance["nodePosition"], "initNodePosition")
        await missionDistance.startMission()
        expect(stubNodePosition.called).to.be.false
    })
    //fake test
    it("should call emitTotalDistance when stopMission called", () => {
        missionDistance["isMissionActive"] = false;
        const emitTotalDistanceSpy = sinon.spy((missionDistance as any), 'emitTotalDistance')
        
        missionDistance.stopMission()
        expect(emitTotalDistanceSpy.called).to.be.false
    })

    it("should set limo id when we call setLimoId", () => {
        const id = 2;
        missionDistance.setLimoId(id);
        expect(missionDistance["limoId"]).to.be.deep.equal(id)
    })

})