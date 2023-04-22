import { describe, beforeEach } from "mocha";
import * as sinon from 'sinon'
import * as roslibjs from 'roslib'
import { expect } from "chai";
import { RosMock } from "../../../classes/RosMock";
import { TopicMock } from "../../../classes/TopicMock";
import { BRIDGE_URI } from "../../../constants/url";
import { NodeBattery } from "./node-battery";
import { Observable } from "rxjs";


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

    it("should initialize node when we call initNodeBattery", () => {
        const spyOn = sinon.spy(rosMock, "on")
        nodeBattery.initNodeBattery()
        expect(spyOn.called).to.be.true
        expect(nodeBattery["ros"]).to.deep.equal(rosMock)
        expect(nodeBattery["batterySubscriber"]).to.deep.equal(topicMock)

    })

    it("should return the correct data", () => {
        const data = {"percentage":40} 
        nodeBattery["data"] = data
        expect(nodeBattery.getData()).to.deep.equal(data)
    })

    it("should return an Observable that emits objects with percentage property", () => {
        const observable = nodeBattery.getBatteryObservable();
        expect(observable).to.be.instanceOf(Observable);
      
        const testValues = [    { percentage: 10 },    { percentage: 20 },    { percentage: 30 },  ];
      
        const onNextSpy = sinon.spy();
        observable.subscribe(onNextSpy);
      
        testValues.forEach((value) => {
          nodeBattery["batterySubject"].next(value);
          expect(onNextSpy.calledWith(value)).to.be.true;
        });
      });
    
    
    it("should call close when we call closeNodeBattery", () => {
        const spyClose = sinon.spy(rosMock, "close")
        // to init ros in the node
        nodeBattery["ros"] = rosMock as unknown as roslibjs.Ros
        nodeBattery.closeNodeBattery()
        expect(spyClose.called).to.be.true
    })

 
      
})