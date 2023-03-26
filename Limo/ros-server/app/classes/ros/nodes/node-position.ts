import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';
import Odometry from '../../../types/Odometry/Odometry';
import deepCopy from '../../../utilities/DeepCopy';

export class NodePosition {
  private ros: Ros;

  private subscriberMovement: Topic;

  private data: Odometry;

  private name = 'Node Position';


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
        name: 'odom',
        messageType: process.env.IS_SIMULATION ? `limo${process.env.LIMO_ID}/nav_msgs/Odometry` : 'nav_msgs/Odometry',
        queue_size: 10,
      });
      this.subscriberMovement.subscribe(this.callBack.bind(this));
    });
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
