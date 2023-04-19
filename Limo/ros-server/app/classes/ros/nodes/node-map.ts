import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';
import  Map from '../../../types/Map';
import { Message } from 'roslib';

export default class NodeMap {
  private ros: Ros;

  private mapSubscriber: Topic;

  private mapPublisher: Topic;

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
        name: process.env.IS_SIMULATION ? '/limo2/map': '/map',
        messageType: 'nav_msgs/OccupancyGrid',
        queue_size: 10,
        queue_length: 10
      });

    this.mapPublisher = new Topic({
      ros: this.ros,
      name: '/p2p/map',
      messageType: 'nav_msgs/OccupancyGrid',
      queue_size: 10,
      queue_length: 10
    });

    this.mapSubscriber.subscribe(this.callBack.bind(this));
    });

  }

  private callBack(data: Map): void {
    this.data = data;
  }

  
  getMap(): Map {
    return this.data;
  }

  sendMap(map: Map) {
    const mapToSend = new Message(map);
    console.log('Le ros-server Limo 1 envoie la map :')
    this.mapPublisher.publish(mapToSend);
  }

  closeNodeMap() {
    if (this.ros) this.ros.close();
  }
}
