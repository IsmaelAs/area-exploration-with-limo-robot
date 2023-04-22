import {P2PPosition} from './p2p-position';
import * as sinon from "sinon";
import { beforeEach, describe, it } from "mocha";
import { expect } from 'chai';
import { NodePosition } from './ros/nodes/node-position';

describe("P2PPosition Unittest", () => {
    let p2pPosition: P2PPosition
    beforeEach(() => {
        p2pPosition = new P2PPosition(1);
       
    })

    afterEach(() => {
        p2pPosition.stopP2PPosition()
        sinon.restore()
    })
    it("xhbxhbxhxb", () => {    
        p2pPosition["distance"] = 50
        const returnGet = p2pPosition.getDistance()
        expect(returnGet).to.equal(50)
       })
    it("jsnjsnsjsn", () => {    
     const spyOn = sinon.spy(p2pPosition["nodePosition"], "closeNodePosition")
     p2pPosition.stopP2PPosition()
     expect(spyOn.called).to.be.true
    })

    it("jsnjsnsjsn", () => {    
        const mockNodePosition: NodePosition = {} as unknown as NodePosition
        mockNodePosition.initNodePosition()
        p2pPosition["nodePosition"].initNodePosition();
        p2pPosition.activateP2P()
        expect(p2pPosition["p2pActivated"]).to.be.true
        p2pPosition["p2pActivated"] = false
        p2pPosition.stopP2PPosition()
       })
});