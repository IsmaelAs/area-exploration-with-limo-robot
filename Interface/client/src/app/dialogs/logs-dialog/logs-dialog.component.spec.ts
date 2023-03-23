import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MatDialogRef, MAT_DIALOG_DATA, MatDialogModule } from '@angular/material/dialog';
import { LogsDialogComponent } from './logs-dialog.component';

describe('LogsDialogComponent', () => {
  let component: LogsDialogComponent;
  let fixture: ComponentFixture<LogsDialogComponent>;
  let dialogsRefSpy: jasmine.SpyObj<MatDialogRef<LogsDialogComponent>>
  let dataSpy: jasmine.SpyObj<{logs: string}>
  beforeEach(async () => {
    dialogsRefSpy = jasmine.createSpyObj(MatDialogRef, ["updateSize", "close"])
    dataSpy = {logs: "test"}

    await TestBed.configureTestingModule({
      imports: [MatDialogModule],
      declarations: [ LogsDialogComponent ],
      providers: [
        {provide: MatDialogRef, useValue: dialogsRefSpy},
        {provide: MAT_DIALOG_DATA, useValue: dataSpy}
      ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(LogsDialogComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it("should call updateSize when we call ngOnInit", () => {
    component.ngOnInit()

    expect(dialogsRefSpy.updateSize).toHaveBeenCalled()
  })

  it("should call close when we call close", () => {
    component.close()

    expect(dialogsRefSpy.close).toHaveBeenCalled()
  })

  it("should return logs when we get logsFormatted", () => {
    dataSpy.logs = "test_v2"

    expect(component.logsFormatted).toBe("test_v2")
  })
});
