const TILES_URL = "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}";
const TILES_LOCAL_PATH = "static/resources/tiles";

function getSearchParameters() {
  var prmstr = window.location.search.substr(1);
  return prmstr != null && prmstr != "" ? transformToAssocArray(prmstr) : {};
}

function transformToAssocArray( prmstr ) {
  var params = {};
  var prmarr = prmstr.split("&");
  for ( var i = 0; i < prmarr.length; i++) {
    var tmparr = prmarr[i].split("=");
    params[tmparr[0]] = tmparr[1];
  }
  return params;
}

const params = getSearchParameters();

document.getElementById('arm').onclick = arm;
document.getElementById('disarm').onclick = disarm;
document.getElementById("id_select").addEventListener("change", onChangeSelector, false);
document.getElementById('mission_checkbox').onclick = mission_decision;
document.getElementById('kill_switch').onclick = kill_switch;
document.getElementById('fly_accept_checkbox').onclick = fly_accept;
document.getElementById('forbidden_zones_checkbox').onclick = toggleForbiddenZones;
document.getElementById('revised-mission-accept').onclick = () => revised_mission_decision(0);
document.getElementById('revised-mission-decline').onclick = () => revised_mission_decision(1);
document.getElementById('monitoring-checkbox').onclick = () => toggle_display_mode();
document.getElementById('flight-info-checkbox').onclick = () => toggle_flight_info_response_mode();
document.getElementById('copy-id').onclick = () => copy_current_id();
document.getElementById('toggle-trajectory').onclick = toggleTrajectory;
document.getElementById('auto-mission-checkbox').onclick = () => toggle_auto_mission_approval_mode();

document.getElementById('auto-revoke-permission-checkbox').onchange = toggleAutoRevokePermission;
document.getElementById('set-revoke-coords').onclick = setRevokeCoords;
document.getElementById('auto-break-connection-checkbox').onchange = toggleAutoBreakConnection;
document.getElementById('set-break-coords').onclick = setBreakCoords;
document.getElementById('change-forbidden-zones-checkbox').onchange = toggleChangeForbiddenZones;
document.getElementById('set-change-forbidden-zones-coords').onclick = setChangeForbiddenZonesCoords;


ol.proj.useGeographic()
const place = [27.85731575, 60.0026278];

let ids = [];
let active_id = null;
let current_mission = null;
let uav = null;
let access_token = params.token;
let current_state = null;
let forbidden_zones_display = false;
let trajectoryFeature = null;

async function copyToClipboard(textToCopy) {
  if (navigator.clipboard && window.isSecureContext) {
    await navigator.clipboard.writeText(textToCopy);
  } else {
    const textArea = document.createElement("textarea");
    textArea.value = textToCopy;
    
    textArea.style.position = "absolute";
    textArea.style.left = "-999999px";
    
    document.body.prepend(textArea);
    textArea.select();
    
    try {
      document.execCommand('copy');
    } catch (error) {
      console.error(error);
    } finally {
      textArea.remove();
    }
  }
}

async function copy_current_id() {
  if (active_id !== null) {
    try {
      await copyToClipboard(active_id);
      console.log('Current ID copied to clipboard:', active_id);
      alert("ID " + active_id + " copied to clipboard!");
    } catch (err) {
      console.error('Failed to copy ID:', err);
    }
  } else {
    console.log('No active ID to copy.');
    alert("No active ID to copy.");
  }
}

let uav_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 0.5],
    src: 'static/resources/vehicle_marker.svg',
    scale: 0.25
  })
});

let inactive_uav_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 0.5],
    src: 'static/resources/vehicle_marker.svg',
    scale: 0.25,
    opacity: 0.6
  })
});

let home_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/home_marker.png'
  })
})

let regular_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/waypoint_marker.png'
  })
})

let servo_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/servo_marker.png'
  })
})

let roi_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/roi_marker.png'
  })
})

let delay_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/delay_marker.png'
  })
})

let polyline_style = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: [255, 255, 255, 255],
    width: 4
  })
});

let polyline_style_inner = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: [0, 0, 0, 255],
    width: 2
  })
});

let trajectory_style = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: 'yellow',
    width: 3
  })
});

let availableTiles = [];

await fetch('/tiles/index')
.then(response => response.json())
.then(data => {
  availableTiles = data;
});

function customTileLoadFunction(imageTile, src) {
  const urlPattern = /x=([0-9]+)&y=([0-9]+)&z=([0-9]+)/;
  const matches = src.match(urlPattern);
  const x = matches[1];
  const y = matches[2];
  const z = matches[3];
  
  const tilePath = `${z}/${x}/${y}`;
  const localUrl = `${TILES_LOCAL_PATH}/${tilePath}.png`;
  
  if (availableTiles.includes(tilePath)) {
    imageTile.getImage().src = localUrl;
  } else {
    imageTile.getImage().src = src;
  }
}

const tileLayer = new ol.layer.Tile({
  source: new ol.source.XYZ({
    url: TILES_URL,
    tileLoadFunction: customTileLoadFunction
  })
});

const map = new ol.Map({
  target: 'map',
  layers: [
    tileLayer
  ],
  view: new ol.View({
    center: place,
    zoom: 15,
  }),
});


const styles = {
  'Polygon': new ol.style.Style({
    stroke: new ol.style.Stroke({
      color: 'red',
      lineDash: [4],
      width: 3,
    }),
    fill: new ol.style.Fill({
      color: 'rgba(255, 0, 0, 0.1)',
    }),
  })
};

const styleFunction = function (feature) {
  return styles[feature.getGeometry().getType()];
};

let geoJSONLayer;

async function createGeoJSONLayer() {
  const response = await fetch(`/admin/get_forbidden_zones?token=${access_token}`);
  if (!response.ok) {
    console.error('Failed to fetch forbidden zones');
    return;
  }
  const geojsonObject = await response.json();
  
  if (geoJSONLayer) {
    map.removeLayer(geoJSONLayer);
  }
  
  geoJSONLayer = new ol.layer.Vector({
    source: new ol.source.Vector({
      features: new ol.format.GeoJSON().readFeatures(geojsonObject),
    }),
    style: styleFunction
  });
  
  geoJSONLayer.getSource().getFeatures().forEach(feature => {
    feature.set('description', `Запрещенная зона: ${feature.get('name')}`);
  });
  
  map.addLayer(geoJSONLayer);
}

const markers = new ol.layer.Vector({
  source: new ol.source.Vector(),
});

map.addLayer(markers);

const info = document.getElementById('info');

let currentFeature;
const displayFeatureInfo = function (pixel, target) {
  const feature = target.closest('.ol-control')
  ? undefined
  : map.forEachFeatureAtPixel(pixel, function (feature) {
    return feature;
  });
  if (feature) {
    info.style.left = pixel[0] + 'px';
    info.style.top = pixel[1] + 'px';
    if (feature !== currentFeature) {
      info.style.visibility = 'visible';
      info.innerText = feature.get('description') || 'Нет описания';
    }
  } else {
    info.style.visibility = 'hidden';
  }
  currentFeature = feature;
};

map.on('pointermove', function (evt) {
  if (evt.dragging) {
    info.style.visibility = 'hidden';
    currentFeature = undefined;
    return;
  }
  const pixel = map.getEventPixel(evt.originalEvent);
  displayFeatureInfo(pixel, evt.originalEvent.target);
});

map.on('click', function (evt) {
  displayFeatureInfo(evt.pixel, evt.originalEvent.target);
});

map.getTargetElement().addEventListener('pointerleave', function () {
  currentFeature = undefined;
  info.style.visibility = 'hidden';
});


function clear_markers() {
  const features = markers.getSource().getFeatures();
  features.forEach((feature) => {
    const feature_id = feature.getId();
    if(feature_id === undefined || !feature.getId().includes('uav')) {
      markers.getSource().removeFeature(feature);
    }
  });
}

function add_marker(lat, lon, alt, marker_type) {
  if (marker_type == 'uav') {
    uav = new ol.Feature(new ol.geom.Point([lon, lat]));
    uav.setId('uav')
    uav.setStyle(uav_style);
    uav.set('description', 'Высота: ' + alt);
    markers.getSource().addFeature(uav);
  } else {
    const marker = new ol.Feature(new ol.geom.Point([lon, lat]));
    if (marker_type == 'home') {
      marker.setStyle(home_marker_style);
      marker.set('description', 'Высота: ' + alt);
    } else if (marker_type == 'servo') {
      marker.setStyle(servo_marker_style);
      marker.set('description', 'Сброс груза\nВысота: ' + alt);
    } else if (marker_type === 'delay') {
      marker.setStyle(delay_marker_style);
      marker.set('description', 'Задержка\nВысота: ' + alt);
    } else if (marker_type === 'roi') {
      marker.setStyle(roi_marker_style);
      marker.set('description', 'ROI\nВысота: ' + alt);
    } else {
      marker.setStyle(regular_marker_style);
      marker.set('description', 'Высота: ' + alt);
    }
    markers.getSource().addFeature(marker);
  }
}

function add_polyline(line_path) {
  const polyline = new ol.geom.MultiLineString([line_path]);
  const polyline_feature = new ol.Feature({
    name: "Thing",
    geometry: polyline,
    description: null
  });
  polyline_feature.setStyle([polyline_style, polyline_style_inner]);
  markers.getSource().addFeature(polyline_feature);
}

function onChangeSelector() {
  let id_select = document.getElementById("id_select");
  change_active_id(id_select.options[id_select.selectedIndex].text);
}

async function arm() {
  console.log('arm')
  if (active_id != null) {
    let resp = await fetch("admin/arm_decision?id=" + active_id + "&decision=0" + "&token=" + access_token);
    console.log(await resp.text());
    getAllData();
  }
}

async function disarm() {
  if (active_id != null) {
    let resp = await fetch("admin/arm_decision?id=" + active_id + "&decision=1" + "&token=" + access_token);
    console.log(await resp.text());
    getAllData();
  }
}


async function kill_switch() {
  if (active_id != null) {
    let resp = await fetch("admin/kill_switch?id=" + active_id + "&token=" + access_token);
    console.log(await resp.text());
    getAllData();
  }
}

async function mission_decision() {
  let mission_checkbox = document.getElementById('mission_checkbox');
  if (current_mission == null || current_mission == '$-1') {
    mission_checkbox.checked = false;
  }
  else {
    let query_str = "admin/mission_decision?id=" + active_id + "&decision=";
    if (mission_checkbox.checked) {
      query_str += 0;
    }
    else {
      query_str += 1;
    }
    query_str += "&token=" + access_token
    let mission_resp = await fetch(query_str);
    let mission_text = await mission_resp.text();
    console.log(mission_text);
  }
}

async function revised_mission_decision(decision) {
  const revisedMissionBlock = document.getElementById('revised-mission-block');
  revisedMissionBlock.style.visibility = 'hidden';
  const query_str = `admin/revise_mission_decision?id=${active_id}&decision=${decision}&token=${access_token}`;
  let mission_resp = await fetch(query_str);
  let mission_text = await mission_resp.text();
  console.log(mission_text);
}

async function toggle_display_mode() {
  const query_str = `admin/toggle_display_mode?token=${access_token}`;
  await fetch(query_str);
  const $monitoringCheckbox = document.getElementById('monitoring-checkbox')
  const $mainButtons = document.getElementById('main-buttons');
  if($monitoringCheckbox.checked) {
    $mainButtons.style.visibility = 'hidden';
  } else {
    $mainButtons.style.visibility = 'visible';
  }
}


async function toggle_flight_info_response_mode() {
  const query_str = `admin/toggle_flight_info_response_mode?token=${access_token}`;
  await fetch(query_str);
}


async function toggle_auto_mission_approval_mode() {
  const query_str = `admin/toggle_auto_mission_approval_mode?token=${access_token}`;
  await fetch(query_str);
}


async function toggleAutoRevokePermission() {
  const checkbox = document.getElementById('auto-revoke-permission-checkbox');
  const coordsDiv = document.getElementById('auto-revoke-permission-coords');
  coordsDiv.style.display = checkbox.checked ? 'block' : 'none';
  if(checkbox.checked) {
    await setRevokeCoords();
  }
  const query_str = `admin/toggle_auto_revoke_permission?enabled=${checkbox.checked}&token=${access_token}`;
  await fetch(query_str);
}

async function setRevokeCoords() {
    const lat = document.getElementById('revoke-lat').value;
    const lon = document.getElementById('revoke-lon').value;
    await fetch(`admin/set_revoke_coords?lat=${lat}&lon=${lon}&token=${access_token}`);
}

async function toggleAutoBreakConnection() {
    const checkbox = document.getElementById('auto-break-connection-checkbox');
    const coordsDiv = document.getElementById('auto-break-connection-coords');
    coordsDiv.style.display = checkbox.checked ? 'block' : 'none';
    if(checkbox.checked) {
      await setBreakCoords();
    }
    const query_str = `admin/toggle_auto_break_connection?enabled=${checkbox.checked}&token=${access_token}`;
    await fetch(query_str);
}

async function setBreakCoords() {
    const lat = document.getElementById('break-lat').value;
    const lon = document.getElementById('break-lon').value;
    await fetch(`admin/set_break_coords?lat=${lat}&lon=${lon}&token=${access_token}`);
}

async function toggleChangeForbiddenZones() {
    const checkbox = document.getElementById('change-forbidden-zones-checkbox');
    const coordsDiv = document.getElementById('change-forbidden-zones-coords');
    coordsDiv.style.display = checkbox.checked ? 'block' : 'none';
    if(checkbox.checked) {
        await setChangeForbiddenZonesCoords();
    }
    const query_str = `admin/toggle_change_forbidden_zones?enabled=${checkbox.checked}&token=${access_token}`;
    await fetch(query_str);
}

async function setChangeForbiddenZonesCoords() {
    const latA = document.getElementById('change-forbidden-zones-lat-A').value;
    const lonA = document.getElementById('change-forbidden-zones-lon-A').value;
    const latB = document.getElementById('change-forbidden-zones-lat-B').value;
    const lonB = document.getElementById('change-forbidden-zones-lon-B').value;
    const latC = document.getElementById('change-forbidden-zones-lat-C').value;
    const lonC = document.getElementById('change-forbidden-zones-lon-C').value;
    await fetch(`admin/set_change_forbidden_zones_coords?lat_A=${latA}&lon_A=${lonA}&lat_B=${latB}&lon_B=${lonB}&lat_C=${latC}&lon_C=${lonC}&token=${access_token}`);
}

async function fly_accept() {
  let fly_accept_checkbox = document.getElementById('fly_accept_checkbox');
  if (active_id == null || current_state == "Kill switch ON") {
    fly_accept_checkbox.checked = false;
  } else {
    let query_str = "admin/change_fly_accept?id=" + active_id + "&decision=";
    if (fly_accept_checkbox.checked) {
      query_str += 0;
    }
    else {
      query_str += 1;
    }
    query_str += "&token=" + access_token
    let fly_accept_resp = await fetch(query_str);
    let fly_accept_text = await fly_accept_resp.text();
    console.log(fly_accept_text);
  }
}



async function get_mission(id) {
  let mission_resp = await fetch("admin/get_mission?id=" + id + "&token=" + access_token);
  let mission_text = await mission_resp.text();
  if (mission_text == '$-1') {
    current_mission = mission_text;
  }
  if (mission_text != '$-1' && current_mission != mission_text) {
    clear_markers()
    console.log(mission_text)
    current_mission = mission_text;
    let mission_path = [];
    let mission_list = mission_text.split('&');
    for (let idx = 0; idx < mission_list.length; ++idx) {
      mission_list[idx] = [Array.from(mission_list[idx])[0]].concat(mission_list[idx].slice(1).split('_'));
    }
    for (let idx = 0; idx < mission_list.length; ++idx) {
      if (mission_list[idx][0] == 'H') {
        const lat = parseFloat(mission_list[idx][1]);
        const lon = parseFloat(mission_list[idx][2]);
        const alt = mission_list[idx][3];
        add_marker(lat, lon, alt, 'home');
        mission_path.push([lon, lat]);
        // map.getView().setCenter([lon, lat]);
      }
      else if (mission_list[idx][0] == 'W') {
        const lat = parseFloat(mission_list[idx][1]);
        const lon = parseFloat(mission_list[idx][2]);
        const alt = mission_list[idx][3];
        if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'S') {
          add_marker(lat, lon, alt, 'servo');              
        } else if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'D') {
          add_marker(lat, lon, alt, 'delay');
        }
        else {
          add_marker(lat, lon, alt, 'regular');
        }
        mission_path.push([lon, lat]);
      }
      else if (mission_list[idx][0] == 'I') {
        const lat = parseFloat(mission_list[idx][1]);
        const lon = parseFloat(mission_list[idx][2]);
        const alt = mission_list[idx][3];
        add_marker(lat, lon, alt, 'roi');
      }
      else if (mission_list[idx][0] == 'L'){
        const lat = parseFloat(mission_list[idx][1]);
        const lon = parseFloat(mission_list[idx][2]);
        const alt = mission_list[idx][3];
        add_marker(lat, lon, alt, 'regular');
      }         
    }
    add_polyline(mission_path);
  }
}

let vehicles = {};

function add_or_update_vehicle_marker(id, lat, lon, alt, azimuth, speed) {
  //let rotationInRadians = azimuth * Math.PI / 180;
  const rotationInRadians = 0;
  let vehicleStyle = new ol.style.Style({
    image: new ol.style.Icon({
      anchor: [0.5, 0.5],
      src: 'static/resources/vehicle_marker.svg',
      scale: 0.25,
      rotation: rotationInRadians
    })
  });
  
  let inactiveVehicleStyle = new ol.style.Style({
    image: new ol.style.Icon({
      anchor: [0.5, 0.5],
      src: 'static/resources/vehicle_marker.svg',
      scale: 0.25,
      opacity: 0.6,
      rotation: rotationInRadians
    })
  });
  
  if (!vehicles[id]) {
    let vehicle = new ol.Feature(new ol.geom.Point([lon, lat]));
    vehicle.setId(`uav${id}`);
    vehicle.setStyle(id === active_id ? vehicleStyle : inactiveVehicleStyle);
    vehicle.set('description', `ID: ${id}\nВысота: ${alt}\nСкорость: ${speed}\nНаправление: ${azimuth}`);
    vehicles[id] = vehicle;
    markers.getSource().addFeature(vehicles[id]);
  } else {
    vehicles[id].getGeometry().setCoordinates([lon, lat]);
    vehicles[id].set('description', `ID: ${id}\nВысота: ${alt}\nСкорость: ${speed}\nНаправление: ${azimuth}`);
    vehicles[id].setStyle(id === active_id ? vehicleStyle : inactiveVehicleStyle);
  }
}



async function change_active_id(new_id) {
  const previous_active_id = active_id;
  active_id = new_id;
  current_mission = null;
  if (previous_active_id !== new_id) {
    removeTrajectory();
  }
  clear_markers()
  await get_mission(new_id);
  await getAllData();
}


async function updateForbiddenZones() {
  await createGeoJSONLayer();
}

function toggleForbiddenZones() {
  forbidden_zones_display = document.getElementById('forbidden_zones_checkbox').checked;
  if (forbidden_zones_display) {
    updateForbiddenZones();
  } else if (geoJSONLayer) {
    map.removeLayer(geoJSONLayer);
    geoJSONLayer = null;
  }
}


document.getElementById('set_delay').onclick = set_delay;

async function set_delay() {
  let delay_value = document.getElementById('delay_input').value;
  if (active_id != null && delay_value != null) {
    let resp = await fetch(`admin/set_delay?id=${active_id}&delay=${delay_value}&token=${access_token}`);
    console.log(await resp.text());
    getAllData();
  }
}


async function getAllData() {
    try {
        const response = await fetch(`/admin/get_all_data?token=${access_token}`);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();

        // Update IDs
        ids = data.ids;
        let id_select = document.getElementById("id_select");
        const selectedIndex = id_select.selectedIndex;
        id_select.innerHTML = '';
        for (let i = 0; i < ids.length; i++) {
            let opt = document.createElement('option');
            opt.value = ids[i];
            opt.innerHTML = ids[i];
            id_select.appendChild(opt);
        }
        if (selectedIndex >= 0 && selectedIndex < id_select.options.length) {
            id_select.selectedIndex = selectedIndex;
        } else if (ids.length > 0) {
            change_active_id(ids[0]);
        }

        // Update Waiters
        document.getElementById("waiters").innerHTML = "Ожидают: " + data.waiters;
        document.getElementById('arm').disabled = !(parseInt(data.waiters) > 0);
        document.getElementById('disarm').disabled = !(parseInt(data.waiters) > 0);

        // Update UAV Data
        for (const id in data.uav_data) {
            const uavData = data.uav_data[id];

            if (id === active_id) {
                document.getElementById("status").innerHTML = "Статус: " + uavData.state;
                current_state = uavData.state;

                if (uavData.state == 'В полете') {
                    document.getElementById('fly_accept_checkbox').checked = true;
                } else {
                    document.getElementById('fly_accept_checkbox').checked = false;
                }

                if (uavData.mission_state == '0') {
                    document.getElementById('mission_checkbox').checked = true;
                } else if (uavData.mission_state == '1') {
                    document.getElementById('mission_checkbox').checked = false;
                } else if (uavData.mission_state == '2') {
                    document.getElementById('mission_checkbox').checked = false;
                    const revisedMissionBlock = document.getElementById('revised-mission-block');
                    revisedMissionBlock.style.visibility = 'visible';
                } else {
                    document.getElementById('mission_checkbox').checked = false;
                }

                document.getElementById("delay").innerHTML = "Delay: " + uavData.delay;
            }

            if (uavData.telemetry) {
                const { lat, lon, alt, azimuth, dop, sats, speed } = uavData.telemetry;
                document.getElementById("dop").innerHTML = "DOP: " + dop;
                document.getElementById("sats").innerHTML = "SATS: " + sats;
                add_or_update_vehicle_marker(id, lat, lon, alt, azimuth, speed);
                if (id === active_id) {
                    map.getView().setCenter([lon, lat]);
                }
            }
        }

        // Update Forbidden Zones
        if (forbidden_zones_display) {
            await updateForbiddenZones();
        }

        const $monitoringCheckbox = document.getElementById('monitoring-checkbox')
        const $mainButtons = document.getElementById('main-buttons');
        if (data.display_mode === '0') {
            $monitoringCheckbox.checked = true;
            $mainButtons.style.visibility = 'hidden';
        } else {
            $monitoringCheckbox.checked = false;
            $mainButtons.style.visibility = 'visible';
        }

        const $flightInfoCheckbox = document.getElementById('flight-info-checkbox')
        if (data.flight_info_response_mode === '0') {
            $flightInfoCheckbox.checked = true;
        } else {
            $flightInfoCheckbox.checked = false;
        }

        const $autoMissionCheckbox = document.getElementById('auto-mission-checkbox')
        if (data.auto_mission_approval_mode === '0') {
            $autoMissionCheckbox.checked = true;
        } else {
            $autoMissionCheckbox.checked = false;
        }

        // Update Auto Revoke Permission State from all_data
        const autoRevokeData = data.auto_revoke_permission_state;
        const autoRevokeCheckbox = document.getElementById('auto-revoke-permission-checkbox');
        const autoRevokeCoordsDiv = document.getElementById('auto-revoke-permission-coords');
        const revokeLatInput = document.getElementById('revoke-lat');
        const revokeLonInput = document.getElementById('revoke-lon');
        autoRevokeCheckbox.checked = autoRevokeData.enabled;
        autoRevokeCoordsDiv.style.display = autoRevokeCheckbox.checked ? 'block' : 'none';
        if (autoRevokeData.coords) {
            if (document.activeElement !== revokeLatInput && document.activeElement !== revokeLonInput) {
                revokeLatInput.value = autoRevokeData.coords.lat || '';
                revokeLonInput.value = autoRevokeData.coords.lon || '';
            }
        }

        // Update Auto Break Connection State from all_data
        const autoBreakData = data.auto_break_connection_state;
        const autoBreakCheckbox = document.getElementById('auto-break-connection-checkbox');
        const autoBreakCoordsDiv = document.getElementById('auto-break-connection-coords');
        const breakLatInput = document.getElementById('break-lat');
        const breakLonInput = document.getElementById('break-lon');
        autoBreakCheckbox.checked = autoBreakData.enabled;
        autoBreakCoordsDiv.style.display = autoBreakCheckbox.checked ? 'block' : 'none';
        if (autoBreakData.coords) {
            if (document.activeElement !== breakLatInput && document.activeElement !== breakLonInput) {
                breakLatInput.value = autoBreakData.coords.lat || '';
                breakLonInput.value = autoBreakData.coords.lon || '';
            }
        }
        
        const changeZonesData = data.change_forbidden_zones_state;
        const changeZonesCheckbox = document.getElementById('change-forbidden-zones-checkbox');
        const changeZonesCoordsDiv = document.getElementById('change-forbidden-zones-coords');
        changeZonesCheckbox.checked = changeZonesData.enabled;
        changeZonesCoordsDiv.style.display = changeZonesCheckbox.checked ? 'block' : 'none';

        const latAInput = document.getElementById('change-forbidden-zones-lat-A');
        const lonAInput = document.getElementById('change-forbidden-zones-lon-A');
        const latBInput = document.getElementById('change-forbidden-zones-lat-B');
        const lonBInput = document.getElementById('change-forbidden-zones-lon-B');
        const latCInput = document.getElementById('change-forbidden-zones-lat-C');
        const lonCInput = document.getElementById('change-forbidden-zones-lon-C');

        const forbiddenZoneInputs = [latAInput, lonAInput, latBInput, lonBInput, latCInput, lonCInput];

        if (!forbiddenZoneInputs.includes(document.activeElement)) {
            if (changeZonesData.coords_A) {
                latAInput.value = changeZonesData.coords_A.lat || '';
                lonAInput.value = changeZonesData.coords_A.lon || '';
            }
            if (changeZonesData.coords_B) {
                latBInput.value = changeZonesData.coords_B.lat || '';
                lonBInput.value = changeZonesData.coords_B.lon || '';
            }
            if (changeZonesData.coords_C) {
                latCInput.value = changeZonesData.coords_C.lat || '';
                lonCInput.value = changeZonesData.coords_C.lon || '';
            }
        }


    } catch (error) {
        console.error("Failed to fetch all data:", error);
    }
}

setInterval(async function() {
  if(active_id) {
    await get_mission(active_id);
  }
  await getAllData();
}, 1000);

function removeTrajectory() {
  if (trajectoryFeature) {
    markers.getSource().removeFeature(trajectoryFeature);
    trajectoryFeature = null;
    console.log('Trajectory removed.');
  }
}

async function toggleTrajectory() {
  if (trajectoryFeature) {
    removeTrajectory();
  } else {
    if (!active_id) {
      alert("Please select a drone ID first.");
      return;
    }
    console.log(`Fetching trajectory for ID: ${active_id}`);
    try {
      const response = await fetch(`/logs/get_telemetry_csv?id=${active_id}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const csvData = await response.text();
      console.log('Received CSV data.');
      
      const lines = csvData.trim().split('\n');
      if (lines.length <= 1) {
        console.log('No trajectory data found.');
        alert('No trajectory data available for this drone.');
        return;
      }
      
      const coordinates = [];
      const headers = lines[0].split(',');
      const latIndex = headers.indexOf('lat');
      const lonIndex = headers.indexOf('lon');
      
      if (latIndex === -1 || lonIndex === -1) {
        throw new Error('CSV data does not contain lat or lon columns.');
      }
      
      for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',');
        const lat = parseFloat(values[latIndex]);
        const lon = parseFloat(values[lonIndex]);
        if (!isNaN(lat) && !isNaN(lon)) {
          coordinates.push([lon, lat]);
        }
      }
      
      if (coordinates.length === 0) {
        console.log('No valid coordinates found in trajectory data.');
        alert('No valid trajectory data available for this drone.');
        return;
      }
      
      const trajectoryLine = new ol.geom.LineString(coordinates);
      trajectoryFeature = new ol.Feature({
        geometry: trajectoryLine,
        name: 'Drone Trajectory',
        description: `Past trajectory for ID: ${active_id}`
      });
      trajectoryFeature.setStyle(trajectory_style);
      markers.getSource().addFeature(trajectoryFeature);
      console.log('Trajectory drawn.');
      
    } catch (error) {
      console.error("Failed to fetch or draw trajectory:", error);
      alert(`Failed to load trajectory: ${error.message}`);
    }
  }
}
