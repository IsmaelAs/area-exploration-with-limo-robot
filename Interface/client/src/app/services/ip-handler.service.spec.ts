import { TestBed } from '@angular/core/testing';

import { IpHandlerService } from './ip-handler.service';
import { SocketCommunicationService } from './socket-communication/socket-communication.service';

describe('IpHandlerService', () => {
  let service: IpHandlerService;
  let  communicationSocketSpy: jasmine.SpyObj<SocketCommunicationService>

  beforeEach(() => {
    communicationSocketSpy = jasmine.createSpyObj("SocketCommunicationService", ["sendLimoIps"])

    TestBed.configureTestingModule({
      providers: [
        {provide: SocketCommunicationService, useValue: communicationSocketSpy}
      ]
    });
    service = TestBed.inject(IpHandlerService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it("should set the ips and call sendLimoIps when we call setIps", () => {
    service["ipLimo1"] = ""
    service["ipLimo2"] = ""

    service.setIps("ip1", "ip2")

    expect(communicationSocketSpy.sendLimoIps).toHaveBeenCalled()
    expect(service["ipLimo1"]).toBe("ip1")
    expect(service["ipLimo2"]).toBe("ip2")
  })

  it("should return ipLimo1 when we get ipAddressLimo1", () => {
    service["ipLimo1"] = "ip1"
    expect(service.ipAddressLimo1).toBe("ip1")
  })

  it("should return ipLimo2 when we get ipAddressLimo2", () => {
    service["ipLimo2"] = "ip2"
    expect(service.ipAddressLimo2).toBe("ip2")
  })
});
