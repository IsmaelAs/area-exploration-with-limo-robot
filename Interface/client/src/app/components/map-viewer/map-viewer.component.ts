import { Component } from '@angular/core';
const ROS3D = require('ros3d');
import ROSLIB, { Ros, TFClient } from 'roslib';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
import { GridClient, MapViewer, UrdfClient } from 'src/app/types/ros3d';
import { environment } from 'src/environments/environment';
@Component({
    'selector': 'app-map-viewer',
    'templateUrl': './map-viewer.component.html',
    'styleUrls': ['./map-viewer.component.scss']
})

export class MapViewerComponent {

    private urdfClient: UrdfClient = { };

    private tfClient: TFClient;

    private gridClient1: GridClient = { };

    private gridClient2: GridClient = { };

    private gridClientMerged: GridClient = { };


    private viewer1: MapViewer = { 
        'scene':  { 
            add: (arrowNode: any) => {
              // implementation of add method
            }, 
            remove: (arrowNode: any) => {
              // implementation of add method
            } 
          } 
      };
      

      private viewer2: MapViewer = { 
        'scene':  { 
          add: (arrowNode: any) => {
            // implementation of add method
          }, 
          remove: (arrowNode: any) => {
            // implementation of add method
          } 
        } 
      };
      

      private viewerMerged: MapViewer = { 
        'scene':  { 
            add: (arrowNode: any) => {
              // implementation of add method
            }, 
            remove: (arrowNode: any) => {
              // implementation of add method
            } 
          } 
      };
      

    private ros1: Ros;

    private ros2: Ros;

    constructor (private ipHandler: IpHandlerService) {

        this.ros1 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo1}:9090`
        });

        this.ros2 = new Ros({
            'url': `ws://${ipHandler.ipAddressLimo2}:9090`
        });

        // Create a TF client that subscribes to the fixed frame.
        this.tfClient = new ROSLIB.TFClient({
            ros          : this.ros1,
            angularThres : 0.1,
            transThres   : 0.1,
            rate         : 1.0,
            fixedFrame   : '/map'
        });


    }

    init () {

        // Create the main viewer 1.
        this.viewer1 = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map1',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the map client 1.
        this.gridClient1 = new ROS3D.OccupancyGridClient({
            'ros': this.ros1,
            'topic': environment.IS_SIMULATION ? '/limo1/map' : '/map',
            'rootObject': this.viewer1.scene,
            'continuous': true
        });

        // Create the main viewer 2.
        this.viewer2 = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map2',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the map client 2.
        this.gridClient2 = new ROS3D.OccupancyGridClient({
            'ros': this.ros2,
            'topic': environment.IS_SIMULATION ? '/limo2/map' : 'map',
            'rootObject': this.viewer2.scene,
            'continuous': true
        });


        // Create the Merged Map viewer 2.
        this.viewerMerged = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map3',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the Merged map client.
        this.gridClientMerged = new ROS3D.OccupancyGridClient({
            'ros': this.ros1,
            'topic': environment.IS_SIMULATION ? '/map_merge/x_merged_map' : '/map_merged/map',
            'rootObject': this.viewerMerged.scene,
            'continuous': true
        });


        /*var odomListener = new ROSLIB.Topic({
            ros: this.ros1,
            name: '/limo1/odom',
            messageType: 'nav_msgs/Odometry'
          });
          
          odomListener.subscribe((message: any) => {
        // Extract the x, y, and z values from the `pose` field of the `Odometry` message
        var x = message.pose.pose.position.x;
        var y = message.pose.pose.position.y;
        var z = message.pose.pose.position.z;
        
        // Update the arrow's position
        arrow.origin.set(x, y, z);
        
        // Redraw the arrow on the viewer
        this.viewer1.scene.remove(arrow); // remove the arrow from the scene
        this.viewer1.scene.add(arrow); // add the arrow back to the scene at the new position
        });*/

        setInterval(()=> {
            this.addArrow();
        }, 3000);

    }

    addArrow() {
        console.log("adding arrow func...")
        // Generate random x, y, and z values between 0 and 5
        var xx = Math.random() * 5;
        var yy = Math.random() * 5;
        var zz = Math.random() * 5;
      
        // Create a new Arrow object with random parameters
        var arrow = new ROS3D.Arrow({
            ros: this.ros1,
            length : 1.0,
            shaftRadius : 0.05,
            headRadius : 0.1,
            headLength : 0.2,
            color : "#ff0000",
            x: xx,
            y: yy,
            z: zz 
        });
      
        // Add the arrow to the viewer
        this.viewer1.scene.add(arrow);
        console.log("added!");
        //Remove the arrow after 1.5 seconds
        setTimeout(() => {
        console.log("removing arrow...")
          this.viewer1.scene.remove(arrow);
          console.log("removed!");
        }, 1500);
      }
      

}
