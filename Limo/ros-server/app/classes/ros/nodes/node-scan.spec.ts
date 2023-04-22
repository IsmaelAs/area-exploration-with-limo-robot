import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import NodeScan from "./node-scan";
import * as deepCopy from "../../../utilities/DeepCopy";
import LaserScan from '../../../types/LaserScan';


describe("Node scan Unittest's", () => {
    let nodeScan: NodeScan
    let rosMock: RosMock
    let topicMock: TopicMock


    beforeEach(() => {
        rosMock = new RosMock({ url: BRIDGE_URI })

        topicMock = new TopicMock({
            ros: rosMock,
            name: 'exploration_state',
            messageType: process.env.IS_SIMULATION ? `limo${process.env.LIMO_ID}/scan` : 'scan',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })

        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })

        nodeScan = new NodeScan()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeScan).to.exist
    })

    it("should initialize node when we call initNodeScan", () => {
        const spyOn = sinon.spy(rosMock, "on")
        const spySubscribe = sinon.spy(topicMock, "subscribe")

        nodeScan.initNodeScan()

        expect(spyOn.called).to.be.true
        expect(spySubscribe.called).to.be.true

        expect(nodeScan["ros"]).to.deep.equal(rosMock)
        expect(nodeScan["subscriberScan"]).to.deep.equal(topicMock)
    })

    it("should set data when we call callback", () => {
        const mockLaserScan: LaserScan = {} as unknown as LaserScan

        nodeScan["callBack"](mockLaserScan)

        expect(nodeScan["data"]).to.deep.equal(mockLaserScan)
    })

    it('should call deepCopy with data when we call getData', () => {
        const mockLaserScan: LaserScan = {} as unknown as LaserScan
        const returnLaserScan: LaserScan = {} as unknown as LaserScan
        const spyDeepCopy = sinon.stub(deepCopy, "default").callsFake(() => { return returnLaserScan })

        nodeScan["data"] = mockLaserScan

        const returnOfFunction = nodeScan.getData()

        expect(returnOfFunction).to.deep.equal(returnLaserScan)
        expect(spyDeepCopy.called).to.be.true
        expect(spyDeepCopy.calledWith(mockLaserScan)).to.be, true
    })


    it("should call close when we call closeNodeScan", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeScan["ros"] = rosMock as unknown as roslibjs.Ros
        nodeScan.closeNodeScan()
        expect(spyClose.called).to.be.true
    })


})