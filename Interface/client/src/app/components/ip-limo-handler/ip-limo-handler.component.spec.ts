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

  it("should call setIps of ip handler service and emit when we call sendLimoIps", () => {
    const spyOutput = spyOn(component.isIpSet, "emit").and.callFake(() => {})

    component.sendLimoIps()

    expect(ipHandlerServiceSpy.setIps).toHaveBeenCalled()
    expect(spyOutput).toHaveBeenCalled()

  })

  it("should return false when the ip1 is false", () => {
    component.ipAddressLimo1 = "127."
    component.ipAddressLimo2 = ""

    expect(component.ipPatternRight).toBeFalsy()
  })

  it("should be false when the ip1 is not set", () => {
    component.ipAddressLimo1 = ""
    component.ipAddressLimo2 = "127.0.0.2"

    expect(component.ipPatternRight).toBeFalsy()
  })

  it("should be true when ip1 and ip2 are good", () => {
    component.ipAddressLimo1 = "192.2.4.66"
    component.ipAddressLimo2 = "127.0.0.2"

    expect(component.ipPatternRight).toBeTrue()
  })
});
