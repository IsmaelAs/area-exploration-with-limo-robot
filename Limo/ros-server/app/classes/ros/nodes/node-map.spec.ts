import { describe, beforeEach } from "mocha";
import NodeMap from "./node-map";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import  Map from '../../../types/Map';


describe("Node map state Unittest's", ()=> {
    let nodeMap: NodeMap
    let rosMock: RosMock
    let topicMock: TopicMock


    beforeEach(() => {
        rosMock = new RosMock({url: BRIDGE_URI})

        topicMock = new TopicMock({
            ros: rosMock,
            name: 'p2p/map',
            messageType: 'nav_msgs/OccupancyGrid',
            queue_size: 10,
        })

        sinon.stub(roslibjs, 'Ros').callsFake((args) => {
            return rosMock
        })

        sinon.stub(roslibjs, 'Topic').callsFake((args) => {
            return topicMock
        })  

        nodeMap = new NodeMap()

    })

    afterEach(() => {
        sinon.restore()
    })

    it("should work", () => {
        expect(nodeMap).to.exist
    })

    it("should set data when we call callback", () => {
        const mockLaserScan: Map = {} as unknown as Map

        nodeMap["callBack"](mockLaserScan)

        expect(nodeMap["data"]).to.deep.equal(mockLaserScan)
    })

    it("should call print message  when we call sendMap", () => {
        nodeMap.initNodeMap()
        const consoleLogSpy = sinon.spy(console, "log")
        const map = nodeMap.getMap()
        nodeMap.sendMap(map)
        expect(consoleLogSpy.called).to.be.true
    })

    
    it("should call close when we call closeNodeMap", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeMap["ros"] = rosMock as unknown as roslibjs.Ros
        nodeMap.closeNodeMap()
        expect(spyClose.called).to.be.true
    })


   
      
})