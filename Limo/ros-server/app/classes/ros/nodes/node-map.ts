import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';
import  Map from '../../../types/Map';

export default class NodeMap {
  private ros: Ros;

  private mapSubscriber: Topic;

  private data: Map;

  private name = 'Node Map';

  initNodeMap() {
    this.ros = new Ros({ url: BRIDGE_URI });

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      // Initialize subscriber
    this.mapSubscriber = new Topic({
        ros: this.ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid',
      });
    this.mapSubscriber.subscribe(this.callBack.bind(this));
    });
  }

  private callBack(data: Map): void {
    this.data = data;
  }

  
  getData(): Map {
    return this.data;
  }

  closeNodeBattery() {
    if (this.ros) this.ros.close();
  }
}
