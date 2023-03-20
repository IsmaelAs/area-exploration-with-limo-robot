import { ComponentFixture, TestBed } from '@angular/core/testing';
import { RobotsActionsComponent } from './robots-actions.component';
import { ActionButtonsComponent } from '../action-buttons/action-buttons.component';
import { MatSelectModule } from '@angular/material/select';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';

describe('RobotsActionsComponent', () => {
  let component: RobotsActionsComponent;
  let fixture: ComponentFixture<RobotsActionsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [MatSelectModule, BrowserAnimationsModule],
      declarations: [ RobotsActionsComponent, ActionButtonsComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotsActionsComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
