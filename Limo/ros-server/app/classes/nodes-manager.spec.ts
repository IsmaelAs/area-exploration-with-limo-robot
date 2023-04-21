import { expect } from "chai";
import { NodeManager} from "./nodes-manager";
import { beforeEach, describe, it } from "mocha";
import * as sinon from "sinon";
import { NodeMovement } from "./ros/nodes/node-movement";
import { NodeExplorationState } from './ros/nodes/node-exploration-state';
import { NodeUpdate } from "./ros/nodes/node-update";
import { NodeBattery } from "./ros/nodes/node-battery";

describe("Node Manager Unittest's", () => {
    
    let nodeManager: NodeManager
    let nodeExplorationState: NodeExplorationState
    let stubNodeMovement: sinon.SinonStubbedInstance<NodeMovement>
    let stubNodeExplorationState: sinon.SinonStubbedInstance<NodeExplorationState>
    let stubNodeUpdate: sinon.SinonStubbedInstance<NodeUpdate>
    let stubNodeBattery: sinon.SinonStubbedInstance<NodeBattery>

    beforeEach(() => {
        stubNodeMovement =  sinon.createStubInstance(NodeMovement)
        stubNodeExplorationState = sinon.createStubInstance(NodeExplorationState)
        stubNodeUpdate = sinon.createStubInstance(NodeUpdate)
        stubNodeBattery = sinon.createStubInstance(NodeBattery)
        nodeManager = new NodeManager(stubNodeExplorationState,stubNodeMovement, stubNodeUpdate, stubNodeBattery)
        nodeExplorationState = new NodeExplorationState()
    })

    afterEach(() => {
        sinon.restore()
    })

    it("should set node movement when creating node manager", () => {
        expect(nodeManager["nodeMovement"]).to.deep.equal(stubNodeMovement)
    })

    it("should call initNodeMovement when we call startNodes", () => {    
        nodeManager.startNodes()

        expect(stubNodeMovement.initNodeMovement.called).to.be.true
    })

    it("should  call move of node movement with good parameters when we call move", async () => {
        await nodeManager.move('backward', 5)

        expect(stubNodeMovement.move.called).to.be.true
        expect(stubNodeMovement.move.calledWith('backward', 5)).to.be.true

        await nodeManager.move('right-backward')

        expect(stubNodeMovement.move.called).to.be.true
        expect(stubNodeMovement.move.calledWith('right-backward')).to.be.true

    })

    it("should  call move of node movement with good parameters when we call identify", async () => {
        await nodeManager.identify()

        expect(stubNodeMovement.move.called).to.be.true
        expect(stubNodeMovement.move.calledWith('left-forward')).to.be.true
    })

    it("should closeNodeMovement when we call stop function", () => {
        nodeManager.stop()

        expect(stubNodeMovement.closeNodeMovement.called).to.be.true
    })

    it("should call sendMessage when the misssion start", () => {
        const spy = sinon.spy(nodeExplorationState, 'sendMessage');
        nodeManager.startMission();
        expect(spy.calledOnce);
    })

    it("should call sendMessage when the misssion stop", () => {
        const spy = sinon.spy(nodeExplorationState, 'sendMessage');
        nodeManager.stopMission();
        expect(spy.calledOnce);
    })

    it("should call move and restartContainers of nodeMovement and nodeUpdate when we call update", async () => {
        await nodeManager.update();
        
        expect(stubNodeMovement.move.calledTwice).to.be.true;
        expect(stubNodeMovement.move.firstCall.calledWith('forward', 2)).to.be.true;
        expect(stubNodeMovement.move.secondCall.calledWith('backward', 2)).to.be.true;
        expect(stubNodeUpdate.restartContainers.calledOnce).to.be.true;
    })
    
})