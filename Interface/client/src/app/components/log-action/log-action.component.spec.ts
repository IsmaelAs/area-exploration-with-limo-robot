import { ComponentFixture, TestBed } from '@angular/core/testing';
import { FormsModule } from '@angular/forms';
import { LogActionComponent } from './log-action.component';
import { SocketCommunicationService } from 'src/app/services/socket-communication/socket-communication.service';
import { MatDialog } from '@angular/material/dialog';
import { Subject } from 'rxjs';

describe('LogActionComponent', () => {
  let component: LogActionComponent;
  let fixture: ComponentFixture<LogActionComponent>;
  let socketCommunicationServiceSpy: jasmine.SpyObj<SocketCommunicationService>
  let matDialogSpy: jasmine.SpyObj<MatDialog>
  let logsObservable: Subject<string> = new Subject()

  beforeEach(async () => {
    socketCommunicationServiceSpy = jasmine.createSpyObj("SocketCommunicationService", ["getSubscribeOpenLogs"])
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
});
