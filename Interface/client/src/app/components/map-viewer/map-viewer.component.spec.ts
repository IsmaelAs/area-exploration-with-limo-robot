import { ComponentFixture, TestBed } from '@angular/core/testing';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
const ROS3D = require('ros3d');
import { Ros } from 'roslib';
import { environment } from 'src/environments/environment';
//import { GridClient, MapViewer } from 'src/app/types/ros3d';
import { MapViewerComponent } from './map-viewer.component';

describe('MapViewerComponent', () => {
  let component: MapViewerComponent;
  let fixture: ComponentFixture<MapViewerComponent>;
  let ipHandlerService: IpHandlerService;

  beforeEach(async () => {
    const rosMock = {
      on: jasmine.createSpy(),
      close: jasmine.createSpy(),
      advertise: jasmine.createSpy(),
      subscribe: jasmine.createSpy(),
      unsubscribe: jasmine.createSpy(),
      callOnConnection: jasmine.createSpy(),
      getServiceType: jasmine.createSpy(),
      getTopicType: jasmine.createSpy(),
      idCounter: 0
    };
    
    const rosLibMock = {
      Ros: function() {
        return rosMock;
      },
      Topic: jasmine.createSpy(),
      Message: jasmine.createSpy(),
      ServiceRequest: jasmine.createSpy(),
      ServiceResponse: jasmine.createSpy(),
      Param: jasmine.createSpy()
    };

    await TestBed.configureTestingModule({
      declarations: [MapViewerComponent],
      providers: [
        IpHandlerService,
        { provide: Ros, useValue: rosMock },
        { provide: 'Roslib', useValue: rosLibMock }
      ]
    }).compileComponents();

    fixture = TestBed.createComponent(MapViewerComponent);
    component = fixture.componentInstance;
    ipHandlerService = TestBed.inject(IpHandlerService);

    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });

  it('should initialize the viewers and clients', () => {
    spyOn(ROS3D.Viewer.prototype, 'constructor');
    spyOn(ROS3D.OccupancyGridClient.prototype, 'constructor');

    component.init();

    expect(ROS3D.Viewer.prototype.constructor).toHaveBeenCalled();
    expect(ROS3D.OccupancyGridClient.prototype.constructor).toHaveBeenCalled();

    expect(component["viewerMerged"]).toBeDefined();
    expect(component["gridClientMerged"]).toBeDefined();
  });

  it('should set the ROS URLs correctly based on the IpHandlerService', () => {
    const ipAddressLimo1 = '';
    const ipAddressLimo2 = '';

    component.init();

    expect(component["ros1"]).toBeDefined();

    expect(component["ros1"].on).toHaveBeenCalledWith('close', jasmine.any(Function));

    expect(component["ros1"].constructor).toHaveBeenCalledWith({
      'url': `ws://${ipAddressLimo1}:9090`
    });
 
  });

  it('should set the topic correctly based on the environment', () => {
    spyOn(ROS3D.OccupancyGridClient.prototype, 'constructor');

    /*const isSimulation = true;
    environment.IS_SIMULATION = isSimulation;*/

    component.init();

    expect(ROS3D.OccupancyGridClient.prototype.constructor).toHaveBeenCalledWith({
      'ros': component["ros1"],
      'topic': '/map_merged/map',
      'rootObject': component["viewerMerged"].scene,
      'continuous': true
    });
  });
    
    it('should create the merged viewer', () => {
    expect(component["viewerMerged"]).toBeDefined();
    });
    
    it('should create the merged grid client', () => {
    expect(component["gridClientMerged"]).toBeDefined();
    });
    
    it('should create the ros 1', () => {
    expect(component["ros1"]).toBeDefined();
    });
    
    describe('init', () => {
    beforeEach(() => {
    spyOn(ROS3D, 'Viewer').and.callThrough();
    spyOn(ROS3D, 'OccupancyGridClient').and.callThrough();
    component.init();
    });
  });
  
  it('should create the merged viewer', () => {
    expect(ROS3D.Viewer).toHaveBeenCalledWith({
      'background': '#7e7e7e',
      'divID': 'map3',
      'width': 800,
      'height': 600,
      'antialias': true
    });
    expect(component["viewerMerged"]).toBeDefined();
  });
  
  it('should setup the merged grid client', () => {
    expect(ROS3D.OccupancyGridClient).toHaveBeenCalledWith({
      'ros': component["ros1"],
      'topic': environment.IS_SIMULATION ? '/map_merge/x_merged_map' : '/map_merged/map',
      'rootObject': component["viewerMerged"].scene,
      'continuous': true
    });
    expect(component["gridClientMerged"]).toBeDefined();
  });
  
});
