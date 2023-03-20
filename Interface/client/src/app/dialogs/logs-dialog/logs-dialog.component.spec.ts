import { ComponentFixture, TestBed } from '@angular/core/testing';
import { MatDialogRef, MAT_DIALOG_DATA, MatDialogModule } from '@angular/material/dialog';
import { LogsDialogComponent } from './logs-dialog.component';

describe('LogsDialogComponent', () => {
  let component: LogsDialogComponent;
  let fixture: ComponentFixture<LogsDialogComponent>;
  let dialogsRefSpy: jasmine.SpyObj<MatDialogRef<LogsDialogComponent>>

  beforeEach(async () => {
    dialogsRefSpy = jasmine.createSpyObj(MatDialogRef, ["updateSize", "close"])

    await TestBed.configureTestingModule({
      imports: [MatDialogModule],
      declarations: [ LogsDialogComponent ],
      providers: [
        {provide: MatDialogRef, useValue: dialogsRefSpy},
        {provide: MAT_DIALOG_DATA, useValue: {}}
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
});
