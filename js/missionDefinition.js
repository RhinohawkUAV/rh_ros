var missionPlanStep = 0;

var waypointNumber = 1;
var nfzNumber = -1;

var drawOpacity = .5;
var drawType = '';
var clickPoint;
var drawing = false;
var plannedPathColor ='#DCDCDC';
var missionBoundsColor = '#ffffff'

var waypointIcons = [];

var outlineCoords = [];


var tempLayers;
var missionLayers;
var pathPlanLayer;

var geofenceCoords = [];
var mission_wpsCoords = [];
var nfzs = [];

var activeMission;

function createWaypointIcons(){
	for (var i=0; i<10; i++){
		var newIcon = L.icon({
			iconUrl: 'img/waypointNumbers/Waypoint_'+(i+1)+".png",
			iconSize: [20,20],
			popupAnchor: [0,-12],
		});	
		waypointIcons.push(newIcon);
	}
}

function startMissionDefinition(){
	clearMission();
  toggleControls();
	createOutline('mission');
}


function clickNext(){
	tempLayers.clearLayers();
	document.getElementById("mapHolder").style.cursor = "auto";


	if(missionPlanStep == 0){
		document.getElementById('missionPlanText').innerHTML = "Click to add mission waypoints";
  	map.addEventListener("click", addWaypoints);
		deactivateNext();
		missionPlanStep ++;
	}else if(missionPlanStep == 1){
		map.removeEventListener("click", addWaypoints);
		document.getElementById('missionPlanText').innerHTML = "Click to draw no fly zones";
		missionPlanStep ++;
		createOutline('nfz');
		activateNext();
	}else if(missionPlanStep == 2){
		map.removeEventListener("click", drawOutline);
		document.getElementById('missionPlanText').innerHTML = "Ready to Fly";
    document.getElementById('button_exportMission').classList.add('visible');
    document.getElementById('button_startMission').classList.add('visible');
    toggleControls();

    createMissionObject(geofenceCoords, mission_wpsCoords, nfzs);

    setTimeout(function(){
      document.getElementById('missionPlanNext').innerHTML = 'next';
    },2000);
	}else if(missionPlanStep == 3){
		
	}
}

function deactivateNext(){
	document.getElementById('missionPlanNext').classList.add('deactive');
}

function activateNext(){
	document.getElementById('missionPlanNext').classList.remove('deactive');
}




// Create outlined shape ---------------------------------------------

function createOutline(type){
  document.getElementById("mapHolder").style.cursor = "crosshair";
 
  if (type == 'nfz'){
    drawColor = '#DF3500';
    drawOpacity = .6;
    nfzNumber++;
  }else if(type =='mission'){
    drawColor = '#efefef';
    drawOpacity = 0;
  }

  drawType = type;
  startDrawing = true;
  map.addEventListener("click", drawOutline);
}

function drawOutline(e){
  outlineCoords.push(e.latlng);
  if (startDrawing == true){
  	console.log('draw outline called');
  	var vertex;
    vertex = L.circleMarker(e.latlng, {radius: 10, color:drawColor, weight:0, fillOpacity:1});
    tempLayers.addLayer(vertex);
    vertex.addEventListener("click", finishOutline);
    startDrawing = false;
  }else{
    var prevlatlng = outlineCoords[outlineCoords.length-2];
    var polyline = L.polyline([prevlatlng, e.latlng], {weight:2, color: drawColor, dashArray:"8, 20"});
    tempLayers.addLayer(polyline);
  }
}

function finishOutline(e){
	console.log('outline finished');

	map.removeEventListener("click", drawOutline);
	document.getElementById("mapHolder").style.cursor = "auto";

	tempLayers.clearLayers();

	if(drawType=="mission"){
		var newOutline = L.polygon(outlineCoords, {color: drawColor, weight:2, fillOpacity:drawOpacity, interactive:false, dashArray:"8, 20"});

		geofenceCoords = outlineCoords;
		activateNext();
	}else if(drawType=="nfz"){
		var newOutline = L.polygon(outlineCoords, {color: drawColor, weight:0, fillOpacity:drawOpacity, interactive:false});

		nfzs[nfzNumber] = outlineCoords;
		nfzNumber++;
		startDrawing = true;
		setTimeout(function(){map.addEventListener("click", drawOutline)}, 100);
	}
	missionLayers.addLayer(newOutline);
	outlineCoords = [];
}


// Draw Mission ---------------------------------------------



function addWaypoints(e){
	if(waypointNumber < 11){
		missionLayers.addLayer(new L.marker(e.latlng, {icon:waypointIcons[waypointNumber-1], title: "Waypoint "+waypointNumber}));
	}else{
		missionLayers.addLayer(new L.circleMarker(e.latlng, {radius: 10, stroke:false, color:'#FFCB00', fillOpacity:1, title: "Waypoint "+waypointNumber}));
	}
  
  mission_wpsCoords.push(e.latlng);
  waypointNumber += 1;
  activateNext();
}


function clearMission(){
  missionLayers.clearLayers();
  tempLayers.clearLayers();
  pathPlanLayer.clearLayers();
  actualPathLayer.clearLayers();

  outlineCoords = [];
  geofenceCoords = [];
  mission_wpsCoords = [];
  nfzs = [];
  missionPlanStep = 0;
  waypointNumber = 1;

  document.getElementById('button_exportMission').classList.remove('visible');
  document.getElementById('button_startMission').classList.remove('visible');

  document.getElementById('missionPlanText').innerHTML = "Click to define mission geofence";
  document.getElementById("mapHolder").style.cursor = "auto";
  map.removeEventListener("click", drawOutline);
  map.removeEventListener("click", addWaypoints);

  deactivateNext();

  vehicleState.unsubscribe();
}

// Draw Arc ---------------------------------------------

function drawArc(radius, latlong, startAngle, angleLength, lineStart, lineEnd){
  var theStartBearing = startAngle
  var theEndBearing = startAngle + angleLength;

  if(angleLength<0){
    theStartBearing = startAngle + angleLength ;
    theEndBearing = startAngle;
  }


  L.arc({
    center: latlong,
    radius: radius,
    startBearing: theStartBearing,
    endBearing: theEndBearing,
    color: plannedPathColor,
    weight:2
  }).addTo(map);
  
  L.polyline([lineStart, lineEnd],{color: plannedPathColor,weight:2} ).addTo(map)
}

//  + Draw Mission Plan ---------------------------------------------
/*
function loadMissionPlan(){
      var theMission = jsyaml.load(mission);
      var theMissionPlan = jsyaml.load(missionPlan);
      drawMissionPlan(theMission, theMissionPlan);
}*/

function drawMissionPlan(missionObject){
  activeMission = missionObject;
  document.getElementById('button_exportMission').classList.add('visible');


  missionLayers.clearLayers();
  waypointNumber = 1;

  //Plot Mission Boundry
  var theGeofence = [];
  for (var i=0; i<missionObject.geofence.points.length; i++ ){
    theGeofence.push([missionObject.geofence.points[i].lat,missionObject.geofence.points[i].lon] )
  }
  missionLayers.addLayer(new L.polygon(theGeofence, {color: '#ffffff', weight:2, fillOpacity:0, interactive:false,dashArray:"2, 6 "}));
 // map.setView(missionObject.boundaryPoints[0], 13);

  // PLot NFZs

  for (var i=0; i < missionObject.static_nfzs.length; i++){
    var nfzCoords = [];
    for (var k=0; k<missionObject.static_nfzs[i].points.length; k++){
      nfzCoords.push([missionObject.static_nfzs[i].points[k].lat, missionObject.static_nfzs[i].points[k].lon]);
    }
    missionLayers.addLayer(new L.polygon(nfzCoords, {color: '#DF3500', weight:1, fillOpacity:.2, interactive:false}));
  }


  //Plot Wayponts

   for (var i=0; i < missionObject.mission_wps.points.length; i++){

    if(waypointNumber < 11){
      missionLayers.addLayer(new L.marker([missionObject.mission_wps.points[i].lat, missionObject.mission_wps.points[i].lon], {icon:waypointIcons[waypointNumber-1], title: "Waypoint "+waypointNumber}));
    }else{
      missionLayers.addLayer(new L.circleMarker([missionObject.mission_wps.points[i].lat, missionObject.mission_wps.points[i].lon], {radius: 10, stroke:false, color:'#FFCB00', fillOpacity:1, title: "Waypoint "+waypointNumber}));
    }
    
    mission_wpsCoords.push([missionObject.mission_wps.points[i].lat, missionObject.mission_wps.points[i].lon]);
    waypointNumber += 1;

  }

}


//Set mission --------------------------------------------------

function startMission(){
  var setTheMission = new ROSLIB.Service({
    ros : ros,
    name : 'rh/command/set_mission',
    serviceType : 'rh_msgs/SetMissionRequest'
  });

  var startTheMission = new ROSLIB.Service({
    ros : ros,
    name : 'rh/command/start_mission',
    serviceType : 'rh_msgs/StartMissionRequest'
  });

 var newRequest = new ROSLIB.ServiceRequest({
     mission:activeMission
  });

 var startRequest = new ROSLIB.ServiceRequest();

  setTheMission.callService(newRequest, function(result) {
    console.log(result);
    if(result.success == true){
      startTheMission.callService(startRequest, function(result) {
          console.log(result);
      });
    }
  });

  subscribeToState();
}


function exportMission(){
  console.log(jsyaml.dump(activeMission));
  toast('YAML logged to console');
}


function createMissionObject(geofenceCoords, missionCoords, the_nfzs){
  var theGeoFence = [];
  var theMission = [];
  var theNFZs = [];

  for(var i=0; i < geofenceCoords.length; i++){
    var coordObj = createGPSCoordMessage(geofenceCoords[i]);
    theGeoFence.push(coordObj); 
  }

  for(var i=0; i < missionCoords.length; i++){
    var coordObj = createGPSCoordMessage(missionCoords[i]);
    theMission.push(coordObj); 
  }

  for(var i=0; i < the_nfzs.length; i++){
    var aNFZ = [];
   // console.log(the_nfzs[i]);
    for(var k=0; k< the_nfzs[i].length; k++){
      var coordObj = createGPSCoordMessage(the_nfzs[i][k]);
      aNFZ.push(coordObj);
    }

     var aNewNFZ= new ROSLIB.Message ({
      points: aNFZ
    });
    theNFZs.push(aNewNFZ);
  } 

  var geoPoints = new ROSLIB.Message({
    points: theGeoFence
  });

  var missionWaypoints = new ROSLIB.Message ({
    points: theMission
  });

  var theStatic_nfzs = new ROSLIB.Message ({
    points: theNFZs
  });

  activeMission = new ROSLIB.Message({
    geofence: geoPoints,
    mission_wps: missionWaypoints,
    static_nfzs: theNFZs,
    roads: []
  });

}


function createGPSCoordMessage(latlng){
  var newCoordMessage = new ROSLIB.Message({
    lat: latlng.lat,
    lon: latlng.lng,
    alt: 0
  });

  return newCoordMessage;
}



function toggleControls(){
	document.getElementById("missionPlanner").classList.toggle('show');
	document.getElementById('timeline').classList.toggle('hidden');
	document.getElementById('telemetry').classList.toggle('hidden');
	document.getElementById('button_defineMission').classList.toggle('hidden');
  document.getElementById('button_clearMap').classList.toggle('hidden');
	document.getElementById('mapHolder').classList.toggle('hideControls');
}

function toast(message){
  document.getElementById('toast').classList.add('show');
  document.getElementById('toast').innerHTML = message;

  setTimeout(function(){
      document.getElementById('toast').classList.remove('show');
    },4000);
}

function loadYAMLFile(){
  document.getElementById("YAMLEntry").classList.add('show');
}

function submitYAML(){
  var theYAML = document.getElementById("myYAML").value;
  //console.log(jsyaml.load(theYAML));
  drawMissionPlan(jsyaml.load(theYAML));
  document.getElementById("YAMLEntry").classList.remove('show');

  map.flyTo([activeMission.mission_wps.points[0].lat,activeMission.mission_wps.points[0].lon], 12);
  document.getElementById('button_exportMission').classList.add('visible');
    document.getElementById('button_startMission').classList.add('visible');
    toggleControls();

  //hide controls
  //show start button

}




