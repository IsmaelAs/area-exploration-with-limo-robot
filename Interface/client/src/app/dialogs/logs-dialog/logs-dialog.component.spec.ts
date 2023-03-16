import { ComponentFixture, TestBed } from '@angular/core/testing';

import { LogsDialogComponent } from './logs-dialog.component';

describe('LogsDialogComponent', () => {
  let component: LogsDialogComponent;
  let fixture: ComponentFixture<LogsDialogComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ LogsDialogComponent ]
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
