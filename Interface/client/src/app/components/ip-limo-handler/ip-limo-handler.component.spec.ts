import { ComponentFixture, TestBed } from '@angular/core/testing';

import { IpLimoHandlerComponent } from './ip-limo-handler.component';

describe('IpLimoHandlerComponent', () => {
  let component: IpLimoHandlerComponent;
  let fixture: ComponentFixture<IpLimoHandlerComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ IpLimoHandlerComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(IpLimoHandlerComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
