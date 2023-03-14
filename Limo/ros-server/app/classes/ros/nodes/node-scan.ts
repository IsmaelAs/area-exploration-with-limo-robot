import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';
import LaserScan from '../../../types/LaserScan';
import deepCopy from '../../../utilities/DeepCopy';

export default class NodeScan {
  private ros: Ros;

  private subscriberScan: Topic;

  private data: LaserScan;

  private name = 'Node Scan';

  initNodeScan() {
    this.ros = new Ros({ url: BRIDGE_URI });

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      // Initialize subscriber
      this.subscriberScan = new Topic({
        ros: this.ros,
        messageType: 'sensor_msgs/LaserScan',
        name: process.env.IS_SIMULATION ? `limo-${process.env.LIMO_ID}/limo/scan` : 'scan',
        queue_size: 10,
      });
      this.subscriberScan.subscribe(this.callBack.bind(this));
    });
  }

  private callBack(data: LaserScan): void {
    this.data = data;
  }

  getData(): LaserScan {
    return deepCopy(this.data);
  }

  closeNodeScan() {
    this.ros.close();
  }
}
