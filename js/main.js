
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



/*var nfz1 =  [[-35.2653204217,149.167854404],[-35.2680289162,149.172119218],[-35.2713521399,149.167857198],[-35.2710924879,149.158370304]];
var nfz2 = [[-35.2784356099,149.157626635],[ -35.2769489533,149.160077947],[-35.2777347419,149.163702692]];
var nfzs = [nfz1, nfz2];
var geofenceCoords = [[-35.2669809344,149.090898914],[-35.3124370914,149.090877152],[-35.3106012238,149.169581702],[-35.2566652302,149.170834472],[-35.247926401,149.161346021],[-35.2487994608,149.094637347]];
var mission_wpsCoords = [[-35.3003799576,149.123938597],[-35.2539580919,149.161561475]];

*/





// Connect to ROSBridge -----------------------------------------------------------------------

function connectToROS(address){
    ros = new ROSLIB.Ros({ url : address });
    ros.on('connection', function() { console.log('Connected to websocket server.');});
    ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error);});
    ros.on('close', function() { console.log('Connection to websocket server closed.');});
    setUpIcons();
    setUpGauges();
    connectToTopics();
   // loadMissionPlan();

    tempLayers = L.layerGroup().addTo(map);
    missionLayers = L.layerGroup().addTo(map);
    pathPlanLayer = L.layerGroup().addTo(map);

}
  


//  Define Icons ---------------------------------------------
 function setUpIcons(){ 
  createWaypointIcons();

  joePositionIcon = L.icon({
      iconUrl: 'img/joesPosition.png',
      iconSize: [17,25],
      popupAnchor: [0,-12],
    });

  startPositionIcon = L.icon({
      iconUrl: 'img/whiteFlag.png',
      iconSize: [17,25],
      popupAnchor: [0,-12],
      iconAnchor:   [0, 24]
    });

   endPositionIcon = L.icon({
      iconUrl: 'img/checkerFlag.png',
      iconSize: [17,25],
      popupAnchor: [0,-12],
      iconAnchor:   [0, 24]
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
  lineWidth: 0.05, // The line thickness
  radiusScale: .50, // Relative radius
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
  console.log("connect to topics");
  var vehicleState = new ROSLIB.Topic({
      ros: ros,
      name: '/rh/state',
      messageType: 'rh_msgs/State'
    });

  vehicleState.subscribe(function(message) {
    //console.log(message);

    //position
    var coords = [message.vehicle_state.position.lat, message.vehicle_state.position.lon];
    document.getElementById("tel-lat").innerHTML= coords[0];
    document.getElementById("tel-long").innerHTML= coords[1];
    uavPath.pushMaxLimit(coords, 5 );
    if(uavPath.length > 1){
      updateMapPath();
    }else{
      map.setView(coords, 18);
    }

    //compass heading
    document.getElementById("compass-pointer").setAttribute('style', 'transform: rotate('+message.vehicle_state.heading+'deg'+');');
    document.getElementById("compass-direction").innerHTML = getDirection(message.vehicle_state.heading);

    //altitude
    document.getElementById("tel-altitude").innerHTML= Math.round(message.vehicle_state.position.alt);

    //speed
    document.getElementById("tel-airSpeed").innerHTML= Math.round(message.vehicle_state.airspeed) + "<span> M/S</span>";

    var pathPlan = [];
    pathPlan.push(coords);
    for (var i=0; i<message.apm_wps.length; i++){
      pathPlan.push([message.apm_wps[i].x_lat, message.apm_wps[i].y_long]);
    }
    pathPlanLayer.clearLayers();
    pathPlanLayer.addLayer(new L.polyline(pathPlan, {color: '#eeeeee', opacity: .4}));

     drawMissionPlan(message.mission);

  });
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


// Toggles  ---------------------------------------------

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








