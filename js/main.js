
var ros;
var mapInit = false;
var clockInit = false;

var uavPath=[];

var startTime;
var lastTime;
var currentTime;
var lastLat;
var lastLong;
var currentLat;
var currentLong;

var theData = {"type": "Feature", "geometry": {"type": "LineString", "coordinates": []}};



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
      name: '/mavros/global_position/rel_alt',
      messageType: 'std_msgs/Float64'
    });

  var navSatTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/mavros/global_position/raw/fix',
      messageType: 'sensor_msgs/NavSatFix'
    });

   var imageTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/gopro/aruco_marker/compressed',
      messageType : 'sensor_msgs/CompressedImage'
    });

   var markerLocation = new ROSLIB.Topic({
      ros: ros,
      name: '/gopro/landing_box_location',
      messageType: 'geometry_msgs/PointStamped'
    });

  var clockTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/mavros/time_reference',
      messageType : 'sensor_msgs/TimeReference'
    });

  markerLocation.subscribe(function(message) {
    console.log("Landing Box Location: x: "+message.point.x +", y: "+message.point.y +", z: "+message.point.z);

    var newDot = document.createElement('div');
    newDot.setAttribute('class', 'landing-box-loc');
    newDot.setAttribute('style', 'left:'+message.point.x/2+'px; top:'+message.point.y/2+'px;');
    document.getElementById('graph-scatter-plot').appendChild(newDot);
  });

  navSatTopic.subscribe(function(message) {

    uavPath.push([message.latitude, message.longitude]);

    if(uavPath.length > 1){
      updateMapPath();
    }else{
      map.setView([ message.latitude,message.longitude], 18);
    }

    document.getElementById("tel-lat").innerHTML= message.latitude;
    document.getElementById("tel-long").innerHTML= message.longitude;

    
  });

  compassTopic.subscribe(function(message) {
    document.getElementById("compass-pointer").setAttribute('style', 'transform: rotate('+message.data+'deg'+');');
    document.getElementById("compass-direction").innerHTML = getDirection(message.data);
  });

  imageTopic.subscribe(function(message) {
    var img = "data:image/png;base64," + message.data;
    document.getElementById( 'aruco-image' ).setAttribute( 'src', img );
  });


  altitudeTopic.subscribe(function(message) {
    document.getElementById("tel-altitude").innerHTML= Math.round(message.data*3.28084) + "<span> FT</span>";
  });

  clockTopic.subscribe(function(message){
    var theDate = new Date();
    theDate.setTime(message.time_ref.secs*1000);
    //console.log(theDate.getSeconds());
    
    document.getElementById("tel-time").innerHTML = theDate.getHours()+":"+theDate.getMinutes()+":"+theDate.getSeconds();


/*
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
    }*/
  });

}

// update path on map---------------------------------------------

function updateMapPath(){
  var newLine = [uavPath[uavPath.length-2],uavPath[uavPath.length-1]]
  var polyline = L.polyline(newLine, {color: '#4ABDE2'}).addTo(map);
}



function calcSpeed (lastLat, lastLong, currentLat, currentLong, time){
  var kilometers = calcDistance(lastLat, lastLong, currentLat, currentLong, "K");
  
  var metersPerSecond = (kilometers*1000)/time;

  return metersPerSecond;
}


function calcDistance(lat1, lon1, lat2, lon2, unit) {
  var radlat1 = Math.PI * lat1/180;
  var radlat2 = Math.PI * lat2/180;
  var theta = lon1-lon2;
  var radtheta = Math.PI * theta/180;
  var dist = Math.sin(radlat1) * Math.sin(radlat2) + Math.cos(radlat1) * Math.cos(radlat2) * Math.cos(radtheta);
  dist = Math.acos(dist);
  dist = dist * 180/Math.PI;
  dist = dist * 60 * 1.1515;
  if (unit=="K") { dist = dist * 1.609344 };
  if (unit=="N") { dist = dist * 0.8684 };
  return dist;
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
