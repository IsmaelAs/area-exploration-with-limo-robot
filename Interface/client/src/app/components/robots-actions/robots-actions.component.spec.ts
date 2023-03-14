import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RobotsActionsComponent } from './robots-actions.component';

describe('RobotsActionsComponent', () => {
  let component: RobotsActionsComponent;
  let fixture: ComponentFixture<RobotsActionsComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ RobotsActionsComponent ]
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
