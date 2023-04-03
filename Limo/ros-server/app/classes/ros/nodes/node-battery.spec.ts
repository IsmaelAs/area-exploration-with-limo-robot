import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import NodeBattery from "./node-battery";


describe("Battery Unittest's", ()=> {
    let nodeBattery: NodeBattery
    let rosMock: RosMock
    let topicMock: TopicMock

    beforeEach(() => {
        rosMock = new RosMock({url: BRIDGE_URI})

        topicMock = new TopicMock({
            ros: rosMock,
            name: '/battery_state',
            messageType: 'sensor_msgs/BatteryState',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })

        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })  

        nodeBattery = new NodeBattery()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeBattery).to.exist
    })

    it("should initialize node when we call initNodeScan", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeBattery.initNodeScan()
        expect(spyOn.called).to.be.true
        expect(nodeBattery["ros"]).to.deep.equal(rosMock)
        expect(nodeBattery["batterySubscriber"]).to.deep.equal(topicMock)

    })

    it("should set data when we call callback", () => {
        const data = {"percentage":40} 
        nodeBattery["callBack"](data)
        expect(nodeBattery["data"]).to.deep.equal(data)
    })

    it("should log a message if battery percentage is below 30", () => {
        const percentage = 15
        const consoleSpy = sinon.spy(console, 'log')
        nodeBattery.onLowBattery(percentage)
        expect(consoleSpy.calledWith('Battery level is below 30%')).to.be.true
      
    })

    it("should return the correct data", () => {
        const data = {"percentage":40} 
        nodeBattery["data"] = data
        expect(nodeBattery.getData()).to.deep.equal(data)
    })
    
    it("should call close when we call closeNodeBattery", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeBattery["ros"] = rosMock as unknown as roslibjs.Ros
        nodeBattery.closeNodeBattery()
        expect(spyClose.called).to.be.true
    })
})