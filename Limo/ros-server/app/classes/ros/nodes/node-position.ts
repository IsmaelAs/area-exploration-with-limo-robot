import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';
import Odometry from '../../../types/Odometry/Odometry';
import deepCopy from '../../../utilities/DeepCopy';

export class NodePosition {
  private ros: Ros;

  private subscriberMovement: Topic;

  private data: Odometry;

  private name = 'Node Position';

  private namespace = '';


  initNodePosition() {
    this.ros = new Ros({url: BRIDGE_URI});

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      // Initialize publisher
      this.subscriberMovement = new Topic({
        ros: this.ros,
        name: `${this.namespace}/odom`,
        messageType: 'nav_msgs/Odometry',
        queue_size: 10,
      });
      this.subscriberMovement.subscribe(this.callBack.bind(this));
    });
  }

  setNamespace(namespace:string) {
    this.namespace = namespace;
    console.log(`namespace for Node Position set to : ${this.namespace}`);
  }


  private callBack(data: Odometry): void {
    this.data = data;
  }

  getData(): Odometry {
    return deepCopy(this.data);
  }

  closeNodePosition() {
    if (this.ros) this.ros.close();
  }
}
