import { Component } from '@angular/core';
const NAV2D = require('nav2d');
const ROS3D = require('ros3d');

const ROS2D = require("node_modules/ros2d/build/ros2d.js");


import * as createjs from 'createjs-module';
import { Ros, Topic } from 'roslib';
import { IpHandlerService } from 'src/app/services/ip-handler.service';
import { GridClient, MapViewer } from 'src/app/types/ros3d';
import { environment } from 'src/environments/environment';
@Component({
    'selector': 'app-map-viewer',
    'templateUrl': './map-viewer.component.html',
    'styleUrls': ['./map-viewer.component.scss']
})

export class MapViewerComponent {


    private gridClient1: GridClient = { };

    private gridClient2: GridClient = { };

    private gridClientMerged: GridClient = { };


    private viewer1: MapViewer = {
        'scene': {
            'add': (arrowNode: any) => {
                // Implementation of add method
            },
            'remove': (arrowNode: any) => {
                // Implementation of add method
            }
        }
    };


    private viewer2: MapViewer = {
        'scene': {
            'add': (arrowNode: any) => {
            // Implementation of add method
            },
            'remove': (arrowNode: any) => {
            // Implementation of add method
            }
        }
    };


    private viewerMerged: MapViewer = {
        'scene': {
            'add': (arrowNode: any) => {
            // Implementation of add method
            },
            'remove': (arrowNode: any) => {
            // Implementation of add method
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

    }

    init () {

        // Create the main viewer 1.
        this.viewer2 = new ROS3D.Viewer({
            'background': '#7e7e7e',
            'divID': 'map2',
            'width': 800,
            'height': 600,
            'antialias': true
        });

        // Setup the map client 1.
        this.gridClient1 = new ROS3D.OccupancyGridClient({
            'ros': this.ros1,
            'topic': environment.IS_SIMULATION ? '/limo1/map' : '/map',
            'rootObject': this.viewer2.scene,
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
            'topic': environment.IS_SIMULATION ? 'laaaaaa/limo2/map' : 'laaa/map',
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
            'topic': '/map_merge/x_merged_map',
            'rootObject': this.viewerMerged.scene,
            'continuous': true
        });


        /*
         * NAV2d-----------------------------------------------------------------------------------------------------------
         *  Create the main viewer.
         */
        //this.yo();


    }

    yo() {
        console.log("yo est appeler")
            var viewerz = new ROS3D.Viewer({
                divID : 'navz',
                width : 800,
                height : 550
            });
            let gridClient = new ROS3D.OccupancyGridClient({
                topic: '/limo1/map',
                ros : this.ros1,
                rootObject : viewerz.scene,
            });

            var robotMarkers = [];
            var topics = [];
        
        var createFunc = function (handlerToCall: any, discriminator: any, robotMarker: any) {
                return discriminator.subscribe(function(pose: any){
                    robotMarker.x = pose.pose.pose.position.x;
                    robotMarker.y = -pose.pose.pose.position.y;
                    var quaZ = pose.pose.pose.orientation.z;
                    var degreeZ = 0;
                    if( quaZ >= 0 ) {
                        degreeZ = quaZ / 1 * 180
                    } else {
                        degreeZ = (-quaZ) / 1 * 180 + 180
                    };
                    robotMarker.rotation = -degreeZ + 35;
                })
            }
        
        for(let i = 0; i < 1; i++){

                // create options object
                var options = {
                    size: 0.25,
                    strokeSize: 0.05,
                    pulse: true,
                    fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.65)
                };
                
                // create NavigationArrow object
                var robotMarker = new ROS2D.NavigationArrow(options);
  
                robotMarkers.push(robotMarker)
                var poseTopic = new Topic({
                    ros: this.ros1,
                    name: '/limo1/amcl_pose',
                    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
                });
                topics.push(poseTopic);
                createFunc('subscribe', poseTopic, robotMarker);
            }

        for(let i = 0; i < robotMarkers.length; i++){
                gridClient.rootObject.addChild(robotMarkers[i]);
            }

    }

}
