import { TestBed } from '@angular/core/testing';
import { SocketCommunicationService } from './socket-communication.service';
import { of } from 'rxjs';
import { Socket } from 'socket.io-client';

describe('SocketCommunicationService', () => {
  let service: SocketCommunicationService;

  beforeEach(() => {
    
    TestBed.configureTestingModule({});
    service = TestBed.inject(SocketCommunicationService);
    
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it("should call emit when we call identify", () => {
    const emitSpy = spyOn<any>(service, "emit").and.callFake(() => {})

    service.identify('limo-1')

    expect(emitSpy).toHaveBeenCalled()
  })

  it("should call emit when we call startMission", () => {
    const emitSpy = spyOn<any>(service, "emit").and.callFake(() => {})

    service.startMission('limo-1')

    expect(emitSpy).toHaveBeenCalled()
  })

  it("should call emit when we call stopMission", () => {
    const emitSpy = spyOn<any>(service, "emit").and.callFake(() => {})

    service.stopMission('limo-1')

    expect(emitSpy).toHaveBeenCalled()
  })

  it("should call emit when we call showLog", () => {
    const emitSpy = spyOn<any>(service, "emit").and.callFake(() => {})

    service.showLog(1)

    expect(emitSpy).toHaveBeenCalled()
  })

  it("should call emit when we call sendLimoIps", () => {
    const emitSpy = spyOn<any>(service, "emit").and.callFake(() => {})

    service.sendLimoIps("ips1", "ips2")

    expect(emitSpy).toHaveBeenCalled()
  })

  it("should return observable when we call getSubscribeOpenLogs", () => {
    const spyObservable  = spyOn(service["logsOpen"], "asObservable").and.callFake(() => {return of("")})

    service.getSubscribeOpenLogs()

    expect(spyObservable).toHaveBeenCalled()
  })

  it("should call on when we call initSocketSubscription", () => {
    const spyOnSocket = spyOn(service["socket"], "on").and.callFake(() => {return {} as unknown as Socket})

    service["initSocketSubscription"]()

    expect(spyOnSocket).toHaveBeenCalled()
  })

  it("should call emit from socket when we pass data", () => {
    const spyEmit = spyOn(service["socket"], "emit").and.callFake(() => {return {} as unknown as Socket<any, any>})
  
    service["emit"]("test", "test")

    expect(spyEmit).toHaveBeenCalledTimes(2)
  })

  it("should call emit from socket when we dont pass data", () => {
    const spyEmit = spyOn(service["socket"], "emit").and.callFake(() => {return {} as unknown as Socket<any, any>})
  
    service["emit"]("test")

    expect(spyEmit).toHaveBeenCalledTimes(2)
  })
});
