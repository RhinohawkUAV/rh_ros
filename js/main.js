
var ros;
var clockInit = false;
var centerMap = true;

var uavPath=[];
var home;
var compass;

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


// Connect to ROSBridge -----------------------------------------------------------------------

function connectToROS(address){
    ros = new ROSLIB.Ros({ url : address });
    ros.on('connection', function() { console.log('Connected to websocket server.');});
    ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error);});
    ros.on('close', function() { console.log('Connection to websocket server closed.');});
    connectToTopics();
}
  


//Set Up Topics -------------------------------------------------------------------------------



function connectToTopics() {

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

   var imageTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/gopro/aruco_marker/compressed',
      messageType : 'sensor_msgs/CompressedImage'
    });

    var clockTopic = new ROSLIB.Topic({
        ros : ros,
        name : '/mavros/time_reference',
        messageType : 'sensor_msgs/TimeReference'
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

    navSatTopic.subscribe(function(message) {

        coords = [message.latitude, message.longitude];

		if (message != null && home == null) {
		    home = coords;
            console.log("Got home coordinates: "+ home[0] + "," + home[1]);
            L.circle(home, {radius: 4, color: '#00ff00'}).addTo(map);

            function mark(coords, label) {
            	var marker = new L.marker(coords, { opacity: 0.75 });
				if (label != null) {
				    //marker.bindTooltip(label, {permanent: true, className: "label", offset: [0, 0] });
                }    
				marker.addTo(map);
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

  compassTopic.subscribe(function(message) {
    compass = message.data;
    document.getElementById("compass-pointer").setAttribute('style', 'transform: rotate('+message.data+'deg'+');');
    document.getElementById("compass-direction").innerHTML = getDirection(message.data);
  });

  imageTopic.subscribe(function(message) {
    var img = "data:image/png;base64," + message.data;
    document.getElementById( 'aruco-image' ).setAttribute( 'src', img );
  });

  altitudeTopic.subscribe(function(message) {
    var altitudeMeters = message.pose.pose.position.z;
    msgStr = ""
    msgStr += `<p class=\"tel-detail\" id=\"altitude-feet\">${ Math.round( altitudeMeters * 3.28084 ) }<span> FT</span></p>`
    msgStr += `<p class=\"tel-detail\" id=\"altitude-meters\">${ Math.round( altitudeMeters ) }<span> M</span></p>`
    document.getElementById("tel-altitude").innerHTML= msgStr;
  });

  groundSpeedTopic.subscribe(function(message) {
    document.getElementById("tel-speed").innerHTML= getGroundVelocity(message.twist).toFixed(1) + "<span> M/S</span>";
    groundSpeedGauge.set(getGroundVelocity(message.twist));
    airSpeedGauge.set(0);

    /* ********* if using VFR_HUD
    document.getElementById("tel-speed").innerHTML= message.groundSpeedGauge.toFixed(1) + "<span> M/S</span>";
    document.getElementById("tel-airSpeed").innerHTML= message.airSpeedGauge.toFixed(1) + "<span> M/S</span>";
    groundSpeedGauge.set(message.groundspeed);
    airSpeedGauge.set(message.airspeed);*/
  });

  targetLocationTopic.subscribe(function(message) {
    if (message.point != null) {
        dx = message.point.x;
        dy = message.point.y;
        //console.log("Got local target position: "+dx+","+dy)
        // Formula from https://stackoverflow.com/questions/2839533/adding-distance-to-a-gps-coordinate
        lat = home[0] + (180/Math.PI) * (dy/6378137);
        lon = home[1] + (180/Math.PI) * (dx/6378137) / Math.cos(home[1]);
        //console.log("    Global target position: "+lat+","+lon)
        L.circle([lat, lon], {radius: 1, weight: 1, opacity: 0.3, color: '#ff0000'}).addTo(map);
    }
  });

  batteryTopic.subscribe(function(message) {
    //console.log( message );
  });
/* *************************************************      SET UP CLOCK
*/
clockTopic.subscribe(function(message){
    var theDate = new Date();
    theDate.setTime(message.time_ref.secs*1000);
    //console.log(theDate.getSeconds());
    
    document.getElementById("tel-time").innerHTML = theDate.getHours()+":"+theDate.getMinutes()+":"+theDate.getSeconds();

/* Old code to calculate speed. 
    if(clockInit == false){
      startTime = theDate;
      lastTime = theDate;
      currentTime = message.time_ref.secs;
      clockInit = true;
      currentLong = uavPath[uavPath.length-1][0];
      currentLat = uavPath[uavPath.length-1][1];
    }else{
      if(message.time_ref.secs != currentTime){
        lastLong = currentLong;
        lastLat = currentLat;
        currentLong = uavPath[uavPath.length-1][0];
        currentCat = uavPath[uavPath.length-1][1];

        lastTime = currentTime;
        currentTime = message.time_ref.secs;

        var deltaTime = (currentTime - lastTime);

        document.getElementById("tel-speed").innerHTML = calcSpeed(lastLat, lastLong, currentLat, currentLong, deltaTime).toFixed(2) + "<span>M/S</span>";
      }  
    }
    */

});

var groundSpeedGauge = new Gauge(document.getElementById('groundGauge')).setOptions(opts); // create sexy gauge!
var airSpeedGauge    = new Gauge(document.getElementById('airGauge')).setOptions(opts); // create sexy gauge!

groundSpeedGauge.maxValue =gaugeTopSpeed; // set max gauge value
airSpeedGauge.maxValue =gaugeTopSpeed; // set max gauge value
}

// Calculate ground velocity using geometry_msgs/TwistStamped.
// Change to use 
function getGroundVelocity(data){
  var velocity = Math.sqrt(Math.abs(Math.pow(data.linear.x, 2)+Math.pow(data.linear.y, 2)));
  return velocity;
}


  

// update path on map---------------------------------------------

function updateMapPath(){
  var newLine = [uavPath[uavPath.length-2],uavPath[uavPath.length-1]]
  var polyline = L.polyline(newLine, {color: '#4ABDE2'}).addTo(map);
  
  if(centerMap){
    map.setView(uavPath[uavPath.length-1]);
  }
}




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

function toggleMapCenter(toToggle){
  if(centerMap){
    centerMap = false;
    document.getElementById(toToggle).setAttribute('class', 'toggleOff');
  }else{
    centerMap = true;
     document.getElementById(toToggle).setAttribute('class', 'toggleOn');
  }
}



 /* ************************************************      MARKER LOCATION
 var markerLocation = new ROSLIB.Topic({
      ros: ros,
      name: '/gopro/landing_box_location',
      messageType: 'geometry_msgs/PointStamped'
    });
 markerLocation.subscribe(function(message) {
    console.log("Landing Box Location: x: "+message.point.x +", y: "+message.point.y +", z: "+message.point.z);

    var newDot = document.createElement('div');
    newDot.setAttribute('class', 'landing-box-loc');
    newDot.setAttribute('style', 'left:'+message.point.x/2+'px; top:'+message.point.y/2+'px;');
    document.getElementById('graph-scatter-plot').appendChild(newDot);
  });
  */

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
