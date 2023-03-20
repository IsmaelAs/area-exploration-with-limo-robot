import { ComponentFixture, TestBed } from '@angular/core/testing';
import { IpLimoHandlerComponent } from './ip-limo-handler.component';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
import { FormsModule } from '@angular/forms';

describe('IpLimoHandlerComponent', () => {
  let component: IpLimoHandlerComponent;
  let fixture: ComponentFixture<IpLimoHandlerComponent>;
  let ipHandlerServiceSpy: jasmine.SpyObj<IpHandlerService>;


  beforeEach(async () => {

    ipHandlerServiceSpy = jasmine.createSpyObj("IpHandlerService", ["setIps"])

    await TestBed.configureTestingModule({
      imports: [ FormsModule ],
      declarations: [ IpLimoHandlerComponent ],
      providers: [
        {provide: IpHandlerService, useValue: ipHandlerServiceSpy}
      ]
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
