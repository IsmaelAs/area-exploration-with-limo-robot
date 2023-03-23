export class RosMock {
  url: string;

  constructor(data: {url: string}) {
    this.url = data.url;
  }

  on(event: string, callBack: () => void) {
    if (event === 'connection') callBack();
  }

  close() {
    // Pass
  }
}
