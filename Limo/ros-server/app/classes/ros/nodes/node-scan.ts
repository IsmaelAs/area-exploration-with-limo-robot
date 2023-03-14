import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '@app/constants/url';
import LaserScan from '@app/types/LaserScan';
import deepCopy from '@app/utilities/DeepCopy';

export default class NodeScan {
  private ros: Ros;

  private subscriberScan: Topic;

  private data: LaserScan;

  initNodeScan() {
    this.ros = new Ros({ url: BRIDGE_URI });
    this.subscriberScan = new Topic({
      ros: this.ros,
      messageType: 'sensor_msgs/LaserScan',
      name: process.env.IS_SIMULATION ? 'limo/scan' : 'scan',
      queue_size: 10,
    });

    this.subscriberScan.subscribe(this.callBack.bind(this));
  }

  private callBack(data: LaserScan): void {
    this.data = data;
  }

  getData(): LaserScan {
    return deepCopy(this.data);
  }
}
