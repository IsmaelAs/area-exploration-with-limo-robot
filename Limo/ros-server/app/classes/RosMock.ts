export class RosMock {
  url: string;

  constructor(data: { url: string }) {
    this.url = data.url;
  }

  on(event: string, callBack: (error?: any) => void) {
    if (event === 'connection') callBack();
    if (event === 'error') callBack(new Error());
    if (event === 'close') callBack();
  }

  close() {
    // Pass
  }
}
