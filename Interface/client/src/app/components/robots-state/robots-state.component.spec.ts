import { ComponentFixture, TestBed } from '@angular/core/testing';

import { RobotsStateComponent } from './robots-state.component';

describe('RobotsStateComponent', () => {
  let component: RobotsStateComponent;
  let fixture: ComponentFixture<RobotsStateComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ RobotsStateComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(RobotsStateComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
