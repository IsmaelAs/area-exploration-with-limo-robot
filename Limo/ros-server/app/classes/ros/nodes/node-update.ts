import { Message, Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';

export class NodeUpdate {
  private ros: Ros;

  private publisherUpdate: Topic;

  private name = 'Node Update';

  initNodeScan() {
    this.ros = new Ros({ url: BRIDGE_URI });

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      // Initialize subscriber
      this.publisherUpdate = new Topic({
        ros: this.ros,
        messageType: 'std_msgs/Bool',
        name: process.env.IS_SIMULATION ? `limo${process.env.LIMO_ID}/restartPackagesContainer` : 'restartPackagesContainer',
        queue_size: 10,
      });
    });
  }

  restartContainers() {
    console.log('redemarage des conteneur ...');
    const msg = new Message({ data: true });
    this.publisherUpdate.publish(msg);
    process.exit(0);
  }

  closeNodeUpdate() {
    if (this.ros) this.ros.close();
  }
}
