
var ros;
var clockInit = false;
var centerMap = false;
var menuOpen = false;
var showArucos = false;
var showVideo = true;

var uavPath=[];
var coords;
var home;
var compass;
var pointInImage;
var currentImg;
var counter = 0;
var targetLoc;

var waypointYellowIcon;
var joePositionIcon;
var startPositionIcon;
var arucoIcon;
var knowPositionIcon;

var groundSpeedGauge;
var airSpeedGauge;

var imageTopic;

var clickPoint;
var drawing = false;
var nfzCoords = [];
var tempLayers = [];


// Connect to ROSBridge -----------------------------------------------------------------------

function connectToROS(address){
    ros = new ROSLIB.Ros({ url : address });
    ros.on('connection', function() { console.log('Connected to websocket server.');});
    ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error);});
    ros.on('close', function() { console.log('Connection to websocket server closed.');});
    setUpIcons();
    setUpGauges();
    connectToTopics();

    drawArc(2, [-35.2802273575, 149.130612897], 242.178467417, 40.107, 1, [-35.2802105398, 149.130604981], [-35.2801391502, 149.130830561]);
    drawArc(2, [-35.2806058227, 149.130369289], 225.0,  342.82, 1, [-35.2805974093, 149.130349847], [-35.2802189442,149.130593454]);

}
  


//  Define Icons ---------------------------------------------
 function setUpIcons(){ 
  waypointYellowIcon = L.icon({
      iconUrl: 'img/waypoint-yellow.png',
      iconSize: [24,24],
      popupAnchor: [0,-12],
    });

  joePositionIcon = L.icon({
      iconUrl: 'img/joesPosition.png',
      iconSize: [17,25],
      popupAnchor: [0,-12],
    });

  startPositionIcon = L.icon({
      iconUrl: 'img/whiteFlag.png',
      iconSize: [17,25],
      popupAnchor: [0,-12],
    });

  arucoIcon = L.icon({
    iconUrl: 'img/arucoIcon.png',
    iconSize: [22,22],
    popupAnchor: [0,-12],
  });

  knowPositionIcon = L.icon({
    iconUrl: 'img/icon-marker.png',
    iconSize: [20,30],
    popupAnchor: [0,-15],
    className:'knownLocation'
  });
}

//  Create Gauges ---------------------------------------------
function setUpGauges(){
  groundSpeedGauge = new Gauge(document.getElementById('groundGauge')).setOptions(opts); // create sexy gauge!
  airSpeedGauge    = new Gauge(document.getElementById('airGauge')).setOptions(opts); // create sexy gauge!

  groundSpeedGauge.maxValue = gaugeTopSpeed; // set max gauge value
  airSpeedGauge.maxValue = gaugeTopSpeed; // set max gauge value
}


// Gauge Settigns -----------------------------------------------------------------------
var opts = {
  angle: -0.26, // The span of the gauge arc
  lineWidth: 0.07, // The line thickness
  radiusScale: .60, // Relative radius
  pointer: {
    length: 0, // // Relative to gauge radius
    strokeWidth: 0, // The thickness
    color: '#000000' // Fill color
  },
  limitMax: false,     // If false, max value increases automatically if value > maxValue
  limitMin: true,     // If true, the min value of the gauge will be fixed
  colorStart: '#fff',   // Colors
  colorStop: '#fff',    // just experiment with them
  strokeColor: 'rgba(255, 255, 255, .3)',  // to see which ones work best for you
  generateGradient: false,
  highDpiSupport: true,     // High resolution support
  
};

var gaugeTopSpeed = 20;

//Set Up Topics -------------------------------------------------------------------------------



function connectToTopics() {

  //Set Up Topics -------------------------------------------------------------------------------

    var compassTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/compass_hdg',
        messageType: 'std_msgs/Float64'
      });

    var altitudeTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/local',
        messageType: 'nav_msgs/Odometry'
      });

    var navSatTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/raw/fix',
        messageType: 'sensor_msgs/NavSatFix'
      });

    var groundSpeedTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/mavros/global_position/raw/gps_vel',
        messageType: 'geometry_msgs/TwistStamped'
      });

    imageTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/gopro/aruco_marker/compressed',
        messageType : 'sensor_msgs/CompressedImage'
      });

    var clockTopic = new ROSLIB.Topic({
          ros : ros,
          name : '/mavros/time_reference',
          messageType : 'sensor_msgs/TimeReference'
      });

    var waypoints = new ROSLIB.Topic({
          ros : ros,
          name : '/mavros/mission/waypoints',
          messageType : 'mavros_msgs/WaypointList'
      });

    var targetLocationTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/gopro/target_position_local',
        messageType : 'geometry_msgs/PointStamped'
    });

    var batteryTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/mavros/battery',
        messageType : 'sensor_msgs/BatteryState'
    });

    
  //Subscribe to Topics -------------------------------------------------------------------------------

    compassTopic.subscribe(function(message) {
      compass = message.data;
      document.getElementById("compass-pointer").setAttribute('style', 'transform: rotate('+message.data+'deg'+');');
      document.getElementById("compass-direction").innerHTML = getDirection(message.data);
    });

    imageTopic.subscribe(function(message) {
      if (counter == 15){
        currentImg = "data:image/png;base64," + message.data;
        document.getElementById( 'aruco-image' ).setAttribute( 'src', currentImg );
        counter = 0;
      }else {
        counter ++;
      }
      
    });

    altitudeTopic.subscribe(function(message) {
      var altitudeMeters = message.pose.pose.position.z;
      msgStr = "";
      msgStr += `<p class='tel-detail' id='altitude-feet'>${ Math.round( altitudeMeters * 3.28084 ) }<span> FT</span></p>`
      msgStr += `<p class='tel-detail' id='altitude-meters'>${ Math.round( altitudeMeters ) }<span> M</span></p>`
      document.getElementById("tel-altitude").innerHTML= msgStr;
    });

    groundSpeedTopic.subscribe(function(message) {
      document.getElementById("tel-speed").innerHTML= getGroundVelocity(message.twist).toFixed(1) + "<span> M/S</span>";
      groundSpeedGauge.set(getGroundVelocity(message.twist));
      airSpeedGauge.set(0);
    });

    batteryTopic.subscribe(function(message) {
      //console.log( message );
    });

    clockTopic.subscribe(function(message){     
      var theDate = new Date();
      theDate.setTime(message.time_ref.secs*1000);
      //console.log(theDate.getSeconds());
      
      document.getElementById("tel-time").innerHTML = theDate.getHours()+":"+theDate.getMinutes()+":"+theDate.getSeconds();
    });

    waypoints.subscribe(function(message){
        //console.log("waypoint:" + message[0].x_lat);
    });

    navSatTopic.subscribe(function(message) {

      coords = [message.latitude, message.longitude];

      if (message != null && home == null) {
        home = coords;
        console.log("Got home coordinates: "+ home[0] + "," + home[1]);
        //L.circle(home, {radius: 4, color: '#00ff00'}).addTo(map);
        L.marker(home,{icon: startPositionIcon}).addTo(map);

        function mark(coords, label) {
          var marker = new L.marker(coords, {icon:knowPositionIcon, opacity: 1, title:label });
          marker.addTo(map);
          L.circle(coords, 10, {stroke: false, color:'#fff', fill: true, fillOpacity:.3, className: 'knownLocation'}).addTo(map);
        }

        // for bags 6,7,8 we know the marker locations
        mark([38.9778045974, -77.3378556003], "1");
        mark([38.9778552409, -77.3377138812], "2");
        mark([38.9777295212, -77.3376484097], "3");
        mark([38.9776844881, -77.3378142882], "4");
        mark([38.977631131,  -77.3376425288], "5");

      }

      uavPath.pushMaxLimit(coords, 5 );

      // If uavPath was populated before, update it.
      // Else, 
      if(uavPath.length > 1){
        updateMapPath();
      }else{
        map.setView(coords, 18);
      }

      document.getElementById("tel-lat").innerHTML= message.latitude;
      document.getElementById("tel-long").innerHTML= message.longitude;
    });

 // Marker Location in image stream ---------------------------------------------

    var markerLocation = new ROSLIB.Topic({
        ros: ros,
        name: '/gopro/landing_box_location',
        messageType: 'geometry_msgs/PointStamped'
      });
    
    markerLocation.subscribe(function(message) {
      pointInImage = [message.point.x, message.point.y];
    });
  

 // Target Location ---------------------------------------------
  
    targetLocationTopic.subscribe(function(message) {
      if (showArucos) {
        if (message.point != null) {
            dx = message.point.x;
            dy = message.point.y;
            //console.log("Got local target position: "+dx+","+dy)
            // Formula from https://stackoverflow.com/questions/2839533/adding-distance-to-a-gps-coordinate
            lat = home[0] + (180/Math.PI) * (dy/6378137);
            lon = home[1] + (180/Math.PI) * (dx/6378137) / Math.cos(home[1]);
            //console.log("    Global target position: "+lat+","+lon)
            var newAruco = L.marker([lat, lon],{icon: arucoIcon}).addTo(map);
            newAruco.bindPopup("<div class='aruco-popup-img-holder'><img src='"+currentImg+"' style='top:"+-(pointInImage[1]-50)+"px; left:"+-(pointInImage[0]-50)+"px;' class='aruco-popup-img'></div>", {className:"aruco-popup"});
            targetLoc = [lat, lon];
        }
      }
    });



   




    // Add placeholder icons to map, to be updated with real data -------------------
    //addJoe([38.977810, -77.338848]);
    //addWaypoint([38.977290, -77.338628]);
}


// Calculate ground velocity using geometry_msgs/TwistStamped.
function getGroundVelocity(data){
  var velocity = Math.sqrt(Math.abs(Math.pow(data.linear.x, 2)+Math.pow(data.linear.y, 2)));
  return velocity;
}




  

// update path on map ---------------------------------------------

function updateMapPath(){
  var newLine = [uavPath[uavPath.length-2],uavPath[uavPath.length-1]]
  var polyline = L.polyline(newLine, {color: '#4ABDE2', className: 'actualPath'}).addTo(map);
  
  if(centerMap){
    map.flyTo(uavPath[uavPath.length-1]);
  }
}

// Add Joe to map ---------------------------------------------

function addJoe(loc){
  console.log("add joe called");
  var newWaypoint = L.marker([loc[0], loc[1]],{icon: joePositionIcon}).addTo(map);
  newWaypoint.bindPopup("<p>Joe's reported location:</p><p>"+loc[0]+"<br />"+loc[1], {className:"waypointTip"});
  
  var possibleMarkerLocation = L.circle(loc, {radius: 200, color:"#ffffff", weight:3, fillColor: "#ffffff", fillOpacity: .4}).addTo(map);
    possibleMarkerLocation.bindTooltip("Marker's possible location", {sticky: true});

  var possibleJoeLocation = L.circle(loc, {radius: 100, color:"#ffffff", weight:0, fillColor: "#4ABDE2", fillOpacity: .4}).addTo(map);
    possibleJoeLocation.bindTooltip("Joe's possible location", {sticky: true});
  
  var innerMarkerLocation = L.circle(loc, {radius: 40, color:"#ffffff", weight:3, fillColor: "#ffffff", fillOpacity: 0, interactive:false}).addTo(map);

}


// Add Waypoint to Map ---------------------------------------------

function addWaypoint(loc){
  
  console.log("waypoint added");
  var newWaypoint = L.marker([loc[0], loc[1]],  {icon: waypointYellowIcon}).addTo(map);
  newWaypoint.bindPopup("<p>Altitude: 32m</p><p>Flight Speed: 35 m/s</p>", {className:"waypointTip"});
}




// Set direction ---------------------------------------------

function getDirection(rotation){
  var direction; 
  
  if(rotation < 15 || rotation > 345){
      direction="N";
  }else if(rotation>=15 && rotation < 75){
    direction="NE";
  }else if(rotation>=75 && rotation < 105){
    direction="E";
  }else if(rotation>=105 && rotation < 165){
    direction="SE";
  }else if(rotation>=165 && rotation < 195){
    direction="S";
  }else if(rotation>=195 && rotation < 255){
    direction="SW";
  }else if(rotation>=255 && rotation < 285){
    direction="W";
  }else if(rotation>=285 && rotation <= 345){
    direction="NW";
  }

  return direction;
}






// Prop def for .pushMax() to limit size of static memory.
// Avoids memory snowball if app is running for a long time.
// Source: https://codereview.stackexchange.com/a/101236
Object.defineProperty(
    Array.prototype,
    "pushMaxLimit",
    {
        configurable: false,
        enumerable: false,
        writable: false,
        value: function( value, max )
        {
            if ( this.length >= max )
            {
                this.splice( 0, this.length - max + 1 );
            }
            return this.push( value );
        }
    }
);


// Interface Interactions -----------------------------------------------------------------------

// Toggle Menu ---------------------------------------------
function toggleMenu() {
  if (!menuOpen){
    document.getElementById('mainHolder').setAttribute('class', 'container-fluid fill menuOpen');
    menuOpen = true;
  }else {
    document.getElementById('mainHolder').setAttribute('class', 'container-fluid fill menuClosed');
    menuOpen = false;
  }
} 
   
function theToggle(setTo,toToggle){
  if(setTo == true){
    document.getElementById(toToggle).setAttribute('class', 'toggleOn');
  }else{
     document.getElementById(toToggle).setAttribute('class', 'toggleOff');
  }
} 


// Toggle Centering ---------------------------------------------

function toggleMapCenter(toToggle){
  if(centerMap){
    centerMap = false;
    theToggle(false,toToggle);
  }else{
    centerMap = true;
    theToggle(true,toToggle);
  }
}

function toggleMapLayer(theLayer, toToggle){
  
  var elements = document.getElementsByClassName(theLayer);

  if(document.getElementById(toToggle).classList.contains('toggleOn')){
    theToggle(false, toToggle);
    for (var i = 0; i < elements.length; i++) { elements[i].style.display = "none"};
  }else{
    theToggle(true,toToggle);
    for (var i = 0; i < elements.length; i++) { elements[i].style.display = "block"};
  }

}

function toggleArucoMarker(toToggle){
  if(showArucos){
    showArucos = false;
    theToggle(false,toToggle);
  }else{
    showArucos = true;
    theToggle(true,toToggle);
  }
}

function toggleVideo(toToggle){
  if(showVideo){
    imageTopic.unsubscribe();
    theToggle(false,toToggle);
    showVideo = false;
  }else{
    imageTopic.subscribe(function(message) {
      currentImg = "data:image/png;base64," + message.data;
      document.getElementById( 'aruco-image' ).setAttribute( 'src', currentImg );
    });
    theToggle(true,toToggle);
    showVideo = true;
  }
}


// Create NFZ ---------------------------------------------

function createNFZ(){
  startDrawing = true;
  map.addEventListener("click", drawNFZ);
}

function drawNFZ(e){
  nfzCoords.push(e.latlng);
  var vertex;
  if (startDrawing == true){
    vertex = L.circleMarker(e.latlng, {radius: 10, color:'#DF3500', weight:2, fillOpacity:.5}).addTo(map);
    vertex.addEventListener("click", finishNFZ);
    startDrawing = false;
  }else{
    vertex = L.circleMarker(e.latlng, {radius: 5, weight:0, fillColor:'#DF3500', fillOpacity:1}).addTo(map);
    var prevlatlng = nfzCoords[nfzCoords.length-2];
    var polyline = L.polyline([prevlatlng, e.latlng], {weight:3, color: '#DF3500', dashArray:"2, 10 "}).addTo(map);
    tempLayers.push(polyline);
  }
  tempLayers.push(vertex);
}

function finishNFZ(e){
  map.removeEventListener("click", drawNFZ);
  var newNFZ = L.polygon(nfzCoords, {color:'#DF3500', weight:2, fillOpacity:.5}).addTo(map);
  for(var i=0; i<tempLayers.length; i++){
    map.removeLayer(tempLayers[i]);
  }
  tempLayers = [];
  nfzCoords = [];
}

// Draw Arc ---------------------------------------------

function drawArc(radius, latlong, startAngle, angleLength, direction, lineStart, lineEnd){
  L.arc({
    center: latlong,
    radius: radius,
    startBearing: startAngle-(angleLength)+96,
    endBearing: startAngle+96
  }).addTo(map);
  
  L.polyline([lineStart, lineEnd]).addTo(map)

  map.setView(latlong, 20);
}

