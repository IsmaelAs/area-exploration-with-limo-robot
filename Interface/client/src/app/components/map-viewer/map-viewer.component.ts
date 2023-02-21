import { Component } from '@angular/core';
const ROS3D =  require('ros3d') 
import { Ros } from 'roslib'
import { GridClient, MapViewer } from 'src/app/types/ros3d';
@Component({
  selector: 'app-map-viewer',
  templateUrl: './map-viewer.component.html',
  styleUrls: ['./map-viewer.component.scss']
})

export class MapViewerComponent {

  private limo_bridge: string = "ws://localhost:9090"
  private gridClient: GridClient = { }
  private viewer: MapViewer = { scene: undefined}
  private ros: Ros

  constructor() {
    this.ros = new Ros({
      url : this.limo_bridge
    });
  }

  init() {

        // Create the main viewer.
        this.viewer = new ROS3D.Viewer({
          divID : 'map',
          width : 800,
          height : 600,
          antialias : true
        });
    
        // Setup the map client.
        this.gridClient = new ROS3D.OccupancyGridClient({
          ros : this.ros,
          rootObject : this.viewer.scene,
          continuous: true
        });
  }


}
