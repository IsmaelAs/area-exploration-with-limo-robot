import { describe, beforeEach } from "mocha";
import { NodeExplorationState } from "./node-exploration-state";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import Bool from '../../../types/Bool';


describe("Note exploration state Unittest's", ()=> {
    let nodeExplorationState: NodeExplorationState
    let rosMock: RosMock
    let topicMock: TopicMock

    beforeEach(() => {
        rosMock = new RosMock({url: BRIDGE_URI})

        topicMock = new TopicMock({
            ros: rosMock,
            name: 'exploration_state',
            messageType: 'std_msgs/Bool',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })

        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })  

        nodeExplorationState = new NodeExplorationState()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeExplorationState).to.exist
    })

    it("should initialize node when we call initNodeExplorationState", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeExplorationState.initNodeExplorationState()
        expect(spyOn.called).to.be.true
        expect(nodeExplorationState["ros"]).to.deep.equal(rosMock)
        expect(nodeExplorationState["publisherExplorationState"]).to.deep.equal(topicMock)

    })

    it("should call close when we call closeNodeExplorationState", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeExplorationState["ros"] = rosMock as unknown as roslibjs.Ros
        nodeExplorationState.closeNodeExplorationState()
        expect(spyClose.called).to.be.true
    })

    it("should pass by when we call closeNodeExporationState on ros undefined", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeExplorationState["ros"] = undefined as unknown as roslibjs.Ros
        nodeExplorationState.closeNodeExplorationState()
        expect(spyClose.called).to.be.false
    })

    it('should publish a message with the provided boolean data', () => {
        const msg: Bool = { data: true };
        nodeExplorationState.initNodeExplorationState()
        const spyPublish = sinon.spy(nodeExplorationState["publisherExplorationState"], 'publish');
        nodeExplorationState.sendMessage(msg);
        expect(spyPublish.called).to.be.true;
      });
      
})