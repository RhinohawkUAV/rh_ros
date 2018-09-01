var missionPlanStep = 0;

var waypointNumber = 1;
var nfzNumber = -1;

var drawOpacity = .5;
var drawType = '';
var clickPoint;
var drawing = false;
var plannedPathColor ='#DCDCDC';

var waypointIcons = [];

var outlineCoords = [];


var tempLayers;
var missionLayers;

var geofenceCoords = [];
var mission_wpsCoords = [];
var nfzs = [];

function createWaypointIcons(){
	for (var i=0; i<10; i++){
		var newIcon = L.icon({
			iconUrl: 'img/waypointNumbers/waypoint_'+(i+1)+".png",
			iconSize: [20,20],
			popupAnchor: [0,-12],
		});	
		waypointIcons.push(newIcon);
	}
}

function startMissionDefinition(){
	clearMission();
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
		document.getElementById('missionPlanNext').classList.add('button_green');
		document.getElementById('missionPlanNext').innerHTML = 'Start Mission';
		missionPlanStep ++;
	}else if(missionPlanStep == 3){
		createMissionObject(geofenceCoords, mission_wpsCoords, nfzs);
		setTimeout(function(){
			document.getElementById('missionPlanNext').classList.remove('button_green');
			document.getElementById('missionPlanNext').innerHTML = 'next';
		},2000);
		toggleControls();
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

  outlineCoords = [];
  geofenceCoords = [];
  mission_wpsCoords = [];
  nfzs = [];
  missionPlanStep = 0;

  document.getElementById('missionPlanText').innerHTML = "Click to define mission geofence";
  document.getElementById("mapHolder").style.cursor = "auto";
  map.removeEventListener("click", drawOutline);
  map.removeEventListener("click", addWaypoints);

  deactivateNext();
  toggleControls();
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

// Load + Draw Mission Plan ---------------------------------------------

function loadMissionPlan(){
      var theMission = jsyaml.load(mission);
      var theMissionPlan = jsyaml.load(missionPlan);
      drawMissionPlan(theMission, theMissionPlan);
}

function drawMissionPlan(missionObject, missionSolution){

  //Plot Mission Boundry
  var missionBounds =   L.polygon(missionObject.boundaryPoints, {color: missionBoundsColor, weight:2, fillOpacity:0, interactive:false,dashArray:"2, 6 "}).addTo(map);
  map.setView(missionObject.boundaryPoints[0], 13);

  // PLot NFZs
  for (var i=0; i < missionObject.noFlyZones.length; i++){
      var newNFZ =   L.polygon(missionObject.noFlyZones[i].points, {color: NFZcolor, weight:2, fillOpacity:.2, interactive:false}).addTo(map);
  }

  //Plot Wayponts
   for (var i=0; i < missionObject.wayPoints.length; i++){
       var waypointMarker = new L.marker(missionObject.wayPoints[i], {icon:endPositionIcon, opacity: 1 }).addTo(map);
  }

  // Plot Solution Path
  var pathSegs = missionSolution.solutionPathSegments;

  for (var i=0; i < pathSegs.length; i++){
      drawArc(pathSegs[i].arc.radius, pathSegs[i].arc.center, pathSegs[i].arc.start, pathSegs[i].arc.length, pathSegs[i].lineStartPoint, pathSegs[i].endPoint);

  }
}


//Set mission --------------------------------------------------


function createMissionObject(geofenceCoods, missionCoords, the_nfzs){
  var theGeoFence = [];
  var theMission = [];
  var theNFZs = [];

  for(var i=0; i < geofenceCoods.length; i++){
    var coordObj = createGPSCoordMessage(geofenceCoods[i][0], geofenceCoods[i][1], 0);
    theGeoFence.push(coordObj); 
  }

  for(var i=0; i < missionCoords.length; i++){
    var coordObj = createGPSCoordMessage(missionCoords[i][0], missionCoords[i][1], 0);
    theMission.push(coordObj); 
  }

  for(var i=0; i < the_nfzs.length; i++){
    var aNFZ = [];
   // console.log(the_nfzs[i]);
    for(var k=0; k< the_nfzs[i].length; k++){
      var coordObj = createGPSCoordMessage(the_nfzs[i][k][0], the_nfzs[i][k][1], 0);
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

  var newMissionObject = new ROSLIB.Message({
    geofence: geoPoints,
    mission_wps: missionWaypoints,
    static_nfzs: theNFZs,
    roads: []
  });

  console.log(newMissionObject);

  var setTheMission = new ROSLIB.Service({
    ros : ros,
    name : 'rh/command/set_mission',
    serviceType : 'rh_msgs/SetMissionRequest'
  });

 var newRequest = new ROSLIB.ServiceRequest({
     mission:newMissionObject
  });

  setTheMission.callService(newRequest, function(result) {
    console.log(result);
    startTheMission();
  });

}


function startTheMission(){	
	var startTheMission = new ROSLIB.Service({
	    ros : ros,
	    name : 'rh/command/start_mission',
	    serviceType : 'rh_msgs/startMissionRequest'
	 });

	var newRequest = new ROSLIB.ServiceRequest({});

	startTheMission.callService(newRequest, function(result) {
	    console.log(result);
	});
}




function createGPSCoordMessage(theLat,theLon,theAlt){
  var newCoordMessage = new ROSLIB.Message({
    lat: theLat,
    lon: theLon,
    alt: theAlt
  });

  return newCoordMessage;
}



function toggleControls(){
	document.getElementById("missionPlanner").classList.toggle('show');

	document.getElementById('timeline').classList.toggle('hidden');
	document.getElementById('telemetry').classList.toggle('hidden');
	document.getElementById('button_defineMission').classList.toggle('hidden');

	document.getElementById('mapHolder').classList.toggle('hideControls');
}
