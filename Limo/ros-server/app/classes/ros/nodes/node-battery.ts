import { Ros, Topic } from 'roslib';
import { BRIDGE_URI } from '../../../constants/url';

export class NodeBattery {
  private ros: Ros;

  private batterySubscriber: Topic;

  private data: {percentage: number};

  private name = 'Node Battery';

  isLowBattery = false;

  initNodeBattery() {
    this.ros = new Ros({ url: BRIDGE_URI });

    this.ros.on('error', (err: Error) => {
      console.error(`${this.name} : ${err.message}`);
    });

    // Wait for ROS to connect to the bridge
    this.ros.on('connection', () => {
      console.log(`${this.name} : ROS connected`);
      // Initialize subscriber
      this.batterySubscriber = new Topic({
        ros: this.ros,
        name: '/battery_state',
        messageType: 'sensor_msgs/BatteryState',
      });
      this.batterySubscriber.subscribe(this.callBack.bind(this));
    });
  }

  private callBack(data: {percentage: number} = {percentage: 100}): void {
    this.data = data;
    // Check if the battery is below 30%
    if (data.percentage < 30) {
      console.log(data.percentage);
      console.log('Battery level is below 30%');
      this.isLowBattery = true;
    }
  }


  getData(): {percentage: number} {
    return this.data;
  }

  closeNodeBattery() {
    if (this.ros) this.ros.close();
  }
}
