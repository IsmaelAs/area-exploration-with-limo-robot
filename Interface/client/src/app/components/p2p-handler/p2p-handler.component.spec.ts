import { ComponentFixture, TestBed } from '@angular/core/testing';

import { P2pHandlerComponent } from './p2p-handler.component';

describe('P2pHandlerComponent', () => {
  let component: P2pHandlerComponent;
  let fixture: ComponentFixture<P2pHandlerComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ P2pHandlerComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(P2pHandlerComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
