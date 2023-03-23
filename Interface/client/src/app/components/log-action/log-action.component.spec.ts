import { ComponentFixture, TestBed } from '@angular/core/testing';
import { FormsModule } from '@angular/forms';
import { LogActionComponent } from './log-action.component';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';
import { MatDialog, MatDialogRef } from '@angular/material/dialog';
import { Subject } from 'rxjs';
import { LogsDialogComponent } from 'src/app/dialogs/logs-dialog/logs-dialog.component';

describe('LogActionComponent', () => {
  let component: LogActionComponent;
  let fixture: ComponentFixture<LogActionComponent>;
  let socketCommunicationServiceSpy: jasmine.SpyObj<SocketCommunicationService>
  let matDialogSpy: jasmine.SpyObj<MatDialog>
  let logsObservable: Subject<string> = new Subject()

  beforeEach(async () => {
    socketCommunicationServiceSpy = jasmine.createSpyObj("SocketCommunicationService", ["getSubscribeOpenLogs", "showLog"])
    matDialogSpy = jasmine.createSpyObj("MatDialog", ["open"])

    socketCommunicationServiceSpy.getSubscribeOpenLogs.and.returnValue(logsObservable.asObservable())


    await TestBed.configureTestingModule({
      imports: [FormsModule],
      declarations: [ LogActionComponent ],
      providers: [
        {provide: SocketCommunicationService, useValue: socketCommunicationServiceSpy},
        {provide: MatDialog, useValue: matDialogSpy}
      ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(LogActionComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it("should ca;; unsubscribe when we call ngOnDestroy", () => {
    const spyUnsubscribe = spyOn(component["openLogsSubscription"], "unsubscribe").and.callFake(() => {})

    component.ngOnDestroy()

    expect(spyUnsubscribe).toHaveBeenCalled()
  })

  it("should have call getSubscribeOpenLogs when the component is created", () => {
    expect(socketCommunicationServiceSpy.getSubscribeOpenLogs).toHaveBeenCalled()
  })

  it("should call showLog of communication socket when we call showLog", () => {
    component.showLog()

    expect(socketCommunicationServiceSpy.showLog).toHaveBeenCalled()
  })

  it("should only call open when the ref is undefined", () => {
  
    component["ref"] = undefined
    component["openLogsDialog"]("test")
    expect(matDialogSpy.open).toHaveBeenCalled()
  })

  it("should call close when ref is defined", () => {
    component["ref"] = {
      close: () => {}
    } as unknown as MatDialogRef<LogsDialogComponent, any>

      const spyClose = spyOn(component["ref"], "close")

          component["openLogsDialog"]("test")
      expect(matDialogSpy.open).toHaveBeenCalled()
      expect(spyClose).toHaveBeenCalled()


  })
});
