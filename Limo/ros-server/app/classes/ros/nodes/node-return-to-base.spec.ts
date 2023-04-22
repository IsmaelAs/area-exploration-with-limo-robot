import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import { describe, beforeEach } from "mocha";
import { NodeReturnToBase } from './node-return-to-base';
import Bool from '../../../types/Bool';

describe("Node Return to Base Unittest's", () => {
    let nodeReturnToBase: NodeReturnToBase
    let rosMock: RosMock
    let topicMock: TopicMock

    beforeEach(() => {
        rosMock = new RosMock({ url: BRIDGE_URI })

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

        nodeReturnToBase = new NodeReturnToBase()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeReturnToBase).to.exist
    })

    it("should init node return to base", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeReturnToBase.initNodeReturnToBase()
        expect(spyOn.called).to.be.true
        expect(nodeReturnToBase["ros"]).to.deep.equal(rosMock)
        expect(nodeReturnToBase["publisherReturnToBase"]).to.deep.equal(topicMock)
    })

    it("should call sendMessage", () => {
        const msg: Bool = { data: true };
        nodeReturnToBase.initNodeReturnToBase()
        const spyPublish = sinon.spy(nodeReturnToBase["publisherReturnToBase"], 'publish');
        nodeReturnToBase.sendMessage(msg);
        expect(spyPublish.called).to.be.true;
    })

    it("should close on closeNodeReturnToBase", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeReturnToBase["ros"] = rosMock as unknown as roslibjs.Ros
        nodeReturnToBase.closeNodeReturnToBase()
        expect(spyClose.called).to.be.true
    })

    it("should pass by on closeNodeReturnToBase and ros undefined", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeReturnToBase["ros"] = undefined as unknown as roslibjs.Ros
        nodeReturnToBase.closeNodeReturnToBase()
        expect(spyClose.called).to.be.false
    })

})