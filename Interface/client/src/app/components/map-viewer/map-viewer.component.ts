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
  private gridClient1: GridClient = { }
  private gridClient2: GridClient = { }
  private gridClient3: GridClient = { }
  private viewer1: MapViewer = { scene: undefined}
  private viewer2: MapViewer = { scene: undefined}
  private viewer3: MapViewer = { scene: undefined}
  private ros: Ros

  constructor() {
    this.ros = new Ros({
      url : this.limo_bridge
    });
  }

  init() {

        // Create the main viewer 1.
        this.viewer1 = new ROS3D.Viewer({
          background: "#7e7e7e",
          divID : 'map1',
          width : 800,
          height : 600,
          antialias : true
        });
    
        // Setup the map client 1.
        this.gridClient1 = new ROS3D.OccupancyGridClient({
          ros : this.ros,
          topic: "/limo1/map",
          rootObject : this.viewer1.scene,
          continuous: true
        });

        // Create the main viewer 2.
        this.viewer2 = new ROS3D.Viewer({
          background: "#7e7e7e",
          divID : 'map2',
          width : 800,
          height : 600,
          antialias : true
        });
    
        // Setup the map client 2.
        this.gridClient2 = new ROS3D.OccupancyGridClient({
          ros : this.ros,
          topic: "/limo2/map",
          rootObject : this.viewer2.scene,
          continuous: true
        });


        // Create the Merged Map viewer 2.
        this.viewer3 = new ROS3D.Viewer({
          background: "#7e7e7e",
          divID : 'map3',
          width : 800,
          height : 600,
          antialias : true
        });
    
        // Setup the Merged map client.
        this.gridClient3 = new ROS3D.OccupancyGridClient({
          ros : this.ros,
          topic: "/map_merge/x_merged_map",
          rootObject : this.viewer3.scene,
          continuous: true
        });
  }


}
