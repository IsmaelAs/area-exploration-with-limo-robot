import { describe, beforeEach } from "mocha";
import { NodePosition } from "./node-position";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import Odometry from "../../../types/Odometry/Odometry";
import * as deepCopy from "../../../utilities/DeepCopy";

describe("Node Position Unittest's", () => {

    let nodePosition: NodePosition
    const rosMock = new RosMock({url: BRIDGE_URI})
    const topicMock = new TopicMock({
        ros: rosMock,
        name: 'odom',
        messageType: 'nav_msgs/Odometry',
        queue_size: 10,
    })

    sinon.stub(roslibjs, 'Ros').callsFake((args) => {
        return rosMock
    })

    sinon.stub(roslibjs, 'Topic').callsFake((args) => {
        return topicMock
    })

    sinon

    beforeEach(() => {
        nodePosition = new NodePosition()
    })

    it("should work", () => {
        expect(nodePosition).to.exist
    })

    it("should initialize node when we call initNodePosition", () => {
        const spyOn = sinon.spy(rosMock, "on")
        const spySubscribe = sinon.spy(topicMock, "subscribe")

        nodePosition.initNodePosition()

        expect(spyOn.called).to.be.true
        expect(spySubscribe.called).to.be.true

        expect(nodePosition["ros"]).to.deep.equal(rosMock)
        expect(nodePosition["subscriberMovement"]).to.deep.equal(topicMock)
    })

    it("should set data when we call callback", () => {
        const mockOdom: Odometry = {} as unknown as Odometry

        nodePosition["callBack"](mockOdom)

        expect(nodePosition["data"]).to.deep.equal(mockOdom)
    })

    it('should call deepCopy with data when we call getData', () => {
        const mockOdom: Odometry = {} as unknown as Odometry
        const returnOdom: Odometry = {} as unknown as Odometry
        const spyDeepCopy = sinon.stub(deepCopy, "default").callsFake(() => {return returnOdom})
    
        nodePosition["data"] = mockOdom

        const returnOfFunction = nodePosition.getData()

        expect(returnOfFunction).to.deep.equal(returnOdom)
        expect(spyDeepCopy.called).to.be.true
        expect(spyDeepCopy.calledWith(mockOdom)).to.be,true
    })

    it("should call close when we call closeNodePosition", () => {
        const spyClose = sinon.spy(rosMock, "close")

        // to init ros in the node
        nodePosition["ros"] = rosMock as unknown as roslibjs.Ros

        nodePosition.closeNodePosition()

        expect(spyClose.called).to.be.true
    })


})