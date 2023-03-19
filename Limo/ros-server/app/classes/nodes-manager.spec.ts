import { expect } from "chai";
import { NodeManager} from "./nodes-manager";
import { beforeEach, describe, it } from "mocha";
import * as sinon from "sinon"
import { NodeMovement } from "./ros/nodes/node-movement";

describe("Node Manager Unittest's", () => {
    
    let nodeManager: NodeManager
    let stubNodeMovement: sinon.SinonStubbedInstance<NodeMovement>

    beforeEach(() => {
        stubNodeMovement =  sinon.createStubInstance(NodeMovement)
        nodeManager = new NodeManager(stubNodeMovement)
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
})