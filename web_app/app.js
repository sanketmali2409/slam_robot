// Override ROS2D Occupancy Grid Colors before anything else
ROS2D.OccupancyGrid.prototype.getColor = function(index, row, col, value) {
    if (value === 100) {
        return [34, 211, 238, 255];    // Occupied: Cyan (#22d3ee)
    } else if (value === 0) {
        return [0, 0, 0, 0];           // Free: Transparent (CSS gradient will show through)
    } else {
        return [26, 31, 46, 255];      // Unknown: Navy (#1a1f2e)
    }
};

// Application State
const PI_IP = '10.248.227.181'; // Hardcoded based on user context
let ros = null;
let viewer = null;
let gridClient = null;
let robotMarker = null;

let lastUpdateTimestamp = Date.now();

// DOM Elements
const connectionDot = document.getElementById('connection-dot');
const connectionStatus = document.getElementById('connection-status');
const lastUpdatedTxt = document.getElementById('last-updated');

const statVelX = document.getElementById('stat-vel-x');
const statVelW = document.getElementById('stat-vel-w');
const statCoverage = document.getElementById('stat-coverage');
const statPose = document.getElementById('stat-pose');
const coverageFill = document.getElementById('coverage-fill');

const btnCoverage = document.getElementById('btn-coverage');
const coverageStatus = document.getElementById('coverage-status');
const btnSave = document.getElementById('btn-save');
const mapNameInput = document.getElementById('map-name');
const saveStatus = document.getElementById('save-status');
const mapCanvas = document.getElementById('map-canvas');

const btnResetView = document.getElementById('btn-reset-view');
const btnZoomIn = document.getElementById('btn-zoom-in');
const btnZoomOut = document.getElementById('btn-zoom-out');

const goalXInput = document.getElementById('goal-x');
const goalYInput = document.getElementById('goal-y');
const goalTInput = document.getElementById('goal-t');
const btnSendGoal = document.getElementById('btn-send-goal');
const goalStatus = document.getElementById('goal-status');

// Joystick state
let joyVx = 0.0;
let joyWz = 0.0;

// Helper to trigger pulse animation
function pulseElement(el) {
    el.classList.remove('pulse');
    void el.offsetWidth; // reflow
    el.classList.add('pulse');
}

// Update "last updated" ticker
setInterval(() => {
    const diffSec = Math.floor((Date.now() - lastUpdateTimestamp) / 1000);
    if (diffSec === 0) {
        lastUpdatedTxt.textContent = 'Updated just now';
    } else if (diffSec < 60) {
        lastUpdatedTxt.textContent = `Updated ${diffSec}s ago`;
    } else {
        lastUpdatedTxt.textContent = `Updated ${Math.floor(diffSec/60)}m ago`;
    }
}, 1000);

// Initialize ROS Connection
function initROS() {
    ros = new ROSLIB.Ros({
        url: `ws://${PI_IP}:9090`
    });

    ros.on('connection', () => {
        console.log('Connected to websocket server.');
        connectionDot.className = 'dot connected';
        connectionStatus.textContent = 'Connected';
        initMap();
        initTelemetry();
        initJoystick();
    });

    ros.on('error', (error) => {
        console.log('Error connecting to websocket server: ', error);
        connectionDot.className = 'dot disconnected';
        connectionStatus.textContent = 'Error (Check if mapping.launch.py is running)';
    });

    ros.on('close', () => {
        console.log('Connection to websocket server closed.');
        connectionDot.className = 'dot disconnected';
        connectionStatus.textContent = 'Disconnected';
    });
}

// Draw faint 1m grid
function drawGrid() {
    const gridShape = new createjs.Shape();
    gridShape.graphics.setStrokeStyle(0.02).beginStroke("rgba(255,255,255,0.08)");
    
    // Draw grid lines from -50m to 50m
    for (let i = -50; i <= 50; i++) {
        // Vertical lines
        gridShape.graphics.moveTo(i, -50).lineTo(i, 50);
        // Horizontal lines
        gridShape.graphics.moveTo(-50, i).lineTo(50, i);
    }
    gridShape.graphics.endStroke();
    
    // Grid goes behind the map but above background
    viewer.scene.addChildAt(gridShape, 0); 
}

// Initialize Map Rendering & Interactivity
function initMap() {
    if (viewer) return;

    // Create the 2D viewer
    viewer = new ROS2D.Viewer({
        divID: 'map-canvas',
        width: mapCanvas.clientWidth,
        height: mapCanvas.clientHeight,
        background: 'transparent' // Required for CSS gradient to show through
    });

    drawGrid();

    // Setup the map client
    gridClient = new ROS2D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
        topic: '/map_volatile',
        continuous: true
    });

    // Scale canvas to fit the map once loaded
    gridClient.on('change', () => {
        lastUpdateTimestamp = Date.now();
        calculateCoverage(gridClient.currentGrid.data);
    });

    // Initial scale once
    let initiallyScaled = false;
    gridClient.on('change', () => {
        if (!initiallyScaled && gridClient.currentGrid.width > 0) {
            resetView();
            initiallyScaled = true;
        }
    });

    // Robot Marker
    robotMarker = new ROS2D.NavigationArrow({
        size: 0.8,
        strokeSize: 0.1,
        fillColor: createjs.Graphics.getRGB(59, 130, 246, 0.9), // Accent blue #3b82f6
        pulse: false
    });
    robotMarker.shadow = new createjs.Shadow("rgba(0,0,0,0.5)", 0, 2, 4);
    
    viewer.scene.addChild(robotMarker);
    robotMarker.visible = false;

    setupMapInteractions();
}

// Calculate Coverage %
function calculateCoverage(gridData) {
    let free = 0;
    let total = gridData.length;
    for (let i = 0; i < total; i++) {
        if (gridData[i] === 0) free++;
    }
    
    const res = gridClient.currentGrid.pose.orientation ? 0.05 : 0.05; // Fallback 0.05m
    const areaSqMeters = free * res * res;
    
    const txt = `${areaSqMeters.toFixed(1)}`;
    if (statCoverage.firstChild.textContent !== txt) {
        statCoverage.firstChild.textContent = txt;
        pulseElement(statCoverage);
        
        // Progress bar visual: assume 100 sqm is a large map for 100% scale
        let pct = (areaSqMeters / 50.0) * 100; 
        if (pct > 100) pct = 100;
        coverageFill.style.width = `${pct}%`;
    }
}

// Map Controls
function resetView() {
    if (!viewer || !gridClient.currentGrid) return;
    viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
    viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
}

btnResetView.addEventListener('click', resetView);

btnZoomIn.addEventListener('click', () => {
    if (viewer) {
        viewer.scene.scaleX *= 1.2;
        viewer.scene.scaleY *= 1.2;
    }
});

btnZoomOut.addEventListener('click', () => {
    if (viewer) {
        viewer.scene.scaleX /= 1.2;
        viewer.scene.scaleY /= 1.2;
    }
});

// Map Panning, Zooming (Mouse), and Navigation Clicks
function setupMapInteractions() {
    let isDragging = false;
    let dragStartX = 0;
    let dragStartY = 0;
    let sceneStartX = 0;
    let sceneStartY = 0;

    // Zoom (Mouse Wheel)
    mapCanvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const zoomSpeed = 1.1;
        if (e.deltaY < 0) {
            viewer.scene.scaleX *= zoomSpeed;
            viewer.scene.scaleY *= zoomSpeed;
        } else {
            viewer.scene.scaleX /= zoomSpeed;
            viewer.scene.scaleY /= zoomSpeed;
        }
    }, { passive: false });

    // Pan & Click-to-Navigate
    viewer.scene.on('stagemousedown', (event) => {
        isDragging = true;
        dragStartX = event.stageX;
        dragStartY = event.stageY;
        sceneStartX = viewer.scene.x;
        sceneStartY = viewer.scene.y;
    });

    viewer.scene.on('stagemousemove', (event) => {
        if (isDragging) {
            const dx = event.stageX - dragStartX;
            const dy = event.stageY - dragStartY;
            viewer.scene.x = sceneStartX + dx;
            viewer.scene.y = sceneStartY + dy;
        }
    });

    viewer.scene.on('stagemouseup', (event) => {
        isDragging = false;
        
        // If it was a tiny movement, treat it as a click to navigate
        const dx = event.stageX - dragStartX;
        const dy = event.stageY - dragStartY;
        if (Math.sqrt(dx*dx + dy*dy) < 5) {
            sendGoal(event.stageX, event.stageY);
        }
    });
}

function sendGoal(stageX, stageY) {
    const pos = viewer.scene.globalToRos(stageX, stageY);
    console.log(`Sending goal to X: ${pos.x.toFixed(2)}, Y: ${pos.y.toFixed(2)}`);
    
    const goalTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
    });

    const goalMsg = new ROSLIB.Message({
        header: {
            stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
            frame_id: 'map'
        },
        pose: {
            position: { x: pos.x, y: pos.y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
    });

    goalTopic.publish(goalMsg);
    
    // Visual feedback
    const circle = new createjs.Shape();
    circle.graphics.beginFill("cyan").drawCircle(0, 0, 0.2);
    circle.x = pos.x;
    circle.y = -pos.y; // ROS2D inverts Y
    viewer.scene.addChild(circle);
    
    setTimeout(() => {
        viewer.scene.removeChild(circle);
    }, 3000);
}

// Initialize Telemetry Subscriptions
function initTelemetry() {
    const odomTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    odomTopic.subscribe((message) => {
        lastUpdateTimestamp = Date.now();

        const x = message.pose.pose.position.x;
        const y = message.pose.pose.position.y;
        
        const q = message.pose.pose.orientation;
        const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        const theta = Math.atan2(siny_cosp, cosy_cosp);

        const vx = message.twist.twist.linear.x;
        const wz = message.twist.twist.angular.z;

        const vXTxt = vx.toFixed(2);
        if (statVelX.firstChild.textContent !== vXTxt) {
            statVelX.firstChild.textContent = vXTxt;
            pulseElement(statVelX);
        }

        const vWTxt = wz.toFixed(2);
        if (statVelW.firstChild.textContent !== vWTxt) {
            statVelW.firstChild.textContent = vWTxt;
            pulseElement(statVelW);
        }

        const thetaDeg = (theta * 180 / Math.PI).toFixed(1);
        statPose.textContent = `${x.toFixed(2)}, ${y.toFixed(2)}, ${thetaDeg}°`;

        // Update Robot Marker
        if (robotMarker && viewer.scene) {
            robotMarker.x = x;
            robotMarker.y = -y;
            robotMarker.rotation = viewer.scene.rosMath.quaternionToRotation(q);
            robotMarker.visible = true;
            // Ensure marker is drawn ON TOP of the map
            viewer.scene.setChildIndex(robotMarker, viewer.scene.getNumChildren() - 1);
        }
    });
}

// Button Events
btnCoverage.addEventListener('click', () => {
    if (!ros) return;
    const coverageTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/start_coverage',
        messageType: 'std_msgs/String'
    });
    coverageTopic.publish(new ROSLIB.Message({ data: 'start' }));
    coverageStatus.textContent = 'Coverage initiated!';
    coverageStatus.className = 'status-text pulse';
    coverageStatus.style.color = 'var(--secondary)';
    setTimeout(() => { coverageStatus.className = 'status-text hidden'; }, 4000);
});

btnSave.addEventListener('click', () => {
    if (!ros) return;
    const mapName = mapNameInput.value.trim() || 'my_map';
    const saveTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/save_map',
        messageType: 'std_msgs/String'
    });
    saveTopic.publish(new ROSLIB.Message({ data: mapName }));
    saveStatus.textContent = `Saving map as "${mapName}"...`;
    saveStatus.className = 'status-text pulse';
    saveStatus.style.color = 'var(--primary)';
    setTimeout(() => { saveStatus.className = 'status-text hidden'; }, 4000);
});

// Coordinate Goal Logic
btnSendGoal.addEventListener('click', () => {
    if (!ros) return;
    let gx = parseFloat(goalXInput.value);
    let gy = parseFloat(goalYInput.value);
    let gt = parseFloat(goalTInput.value);
    
    if (isNaN(gx)) gx = 0.0;
    if (isNaN(gy)) gy = 0.0;
    if (isNaN(gt)) gt = 0.0;

    // Convert degrees to radians, then to quaternion
    const thetaRad = gt * Math.PI / 180.0;
    const qz = Math.sin(thetaRad / 2.0);
    const qw = Math.cos(thetaRad / 2.0);

    const goalTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/PoseStamped'
    });

    const goalMsg = new ROSLIB.Message({
        header: {
            stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 },
            frame_id: 'map'
        },
        pose: {
            position: { x: gx, y: gy, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: qz, w: qw }
        }
    });

    goalTopic.publish(goalMsg);

    // Visual feedback
    if (viewer && viewer.scene) {
        const circle = new createjs.Shape();
        circle.graphics.beginFill("cyan").drawCircle(0, 0, 0.2);
        circle.x = gx;
        circle.y = -gy;
        viewer.scene.addChild(circle);
        setTimeout(() => { viewer.scene.removeChild(circle); }, 3000);
    }

    goalStatus.textContent = `Goal sent: (${gx.toFixed(1)}, ${gy.toFixed(1)})`;
    goalStatus.className = 'status-text pulse';
    goalStatus.style.color = 'var(--primary)';
    setTimeout(() => { goalStatus.className = 'status-text hidden'; }, 4000);
});

// Joystick Logic
function initJoystick() {
    const zone = document.getElementById('joystick-zone');
    const manager = nipplejs.create({
        zone: zone,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#3b82f6',
        size: 100
    });

    const maxLin = 0.5; // m/s
    const maxAng = 1.0; // rad/s

    manager.on('move', (evt, data) => {
        // Forward is 90 degrees in nipplejs, but we can just use vector x/y
        // data.vector.x is between -1 (left) and 1 (right)
        // data.vector.y is between -1 (down) and 1 (up)
        
        joyVx = data.vector.y * maxLin;
        joyWz = -data.vector.x * maxAng; // Left x (-1) should be positive Wz (CCW)
    });

    manager.on('end', () => {
        joyVx = 0.0;
        joyWz = 0.0;
    });

    // Publish cmd_vel at 10Hz
    const cmdTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    setInterval(() => {
        if (!ros) return;
        const twist = new ROSLIB.Message({
            linear: { x: joyVx, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: joyWz }
        });
        cmdTopic.publish(twist);
    }, 100);
}

// Start on load
window.onload = () => {
    initROS();
    
    window.addEventListener('resize', () => {
        if (viewer && gridClient && gridClient.currentGrid) {
            viewer.width = mapCanvas.clientWidth;
            viewer.height = mapCanvas.clientHeight;
        }
    });
};
