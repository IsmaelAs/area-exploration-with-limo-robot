import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ActionButtonsComponent } from './action-buttons.component';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';


describe('ActionButtonsComponent', () => {
  let component: ActionButtonsComponent;
  let fixture: ComponentFixture<ActionButtonsComponent>;
  let socketCommunicationSpy: jasmine.SpyObj<SocketCommunicationService>

  beforeEach(async () => {
  
    socketCommunicationSpy = jasmine.createSpyObj("SocketCommunicationService", ["identify", "startMission", "stopMission"])
    await TestBed.configureTestingModule({
      declarations: [ ActionButtonsComponent ],
      providers: [
        {provide: SocketCommunicationService, useValue: socketCommunicationSpy}
      ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(ActionButtonsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it("should call socket communication identify function when we call identify", () => {
    component.identify()

    expect(socketCommunicationSpy.identify).toHaveBeenCalled()
  })

  it("should call socket communication startMission function when we call startMission", () => {
    component.startMission()
    expect(socketCommunicationSpy.startMission).toHaveBeenCalled()
  })

  it("should call socket communication stopMission function when we call stopMission", () => {
    component.stopMission()
    expect(socketCommunicationSpy.stopMission).toHaveBeenCalled()
  })
});
