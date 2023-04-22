export class TopicMock {
  name: string;

  messageType: string;

  ros: unknown;

  constructor(data: { ros: unknown, name: string, messageType: string, queue_size: number }) {
    this.name = data.name;
    this.messageType = data.messageType;
    this.ros = data.ros;
  }

  subscribe() {
    // Pass
  }

  publish() {
    // Pass
  }
}
