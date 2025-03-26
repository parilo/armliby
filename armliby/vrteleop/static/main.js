import * as THREE2 from 'three';
const THREE = AFRAME.THREE;
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

const INITIAL_SHIFT = new THREE.Vector3(-1, 0, 1.);

AFRAME.registerComponent('stl-model-loader', {
  schema: {
    src: {type: 'string'}
  },
  init: function () {
    const loader = new STLLoader();
    const el = this.el;

    // Load STL model from the specified URL
    loader.load(this.data.src, function (geometry) {
      const material = new THREE.MeshStandardMaterial({color: 0x5555ff});
      const mesh = new THREE.Mesh(geometry, material);
      el.setObject3D('mesh', mesh);
    });
  },
});

AFRAME.registerComponent('coordinate-frame', {
  init: function () {
    const el = this.el;
    const axisLength = 0.1;
    const axisThickness = 0.002; // Adjust thickness here

    // X-axis cylinder (red)
    const xAxis = document.createElement('a-cylinder');
    xAxis.setAttribute('position', `${axisLength / 2} 0 0`);
    xAxis.setAttribute('radius', axisThickness);
    xAxis.setAttribute('height', axisLength);
    xAxis.setAttribute('color', 'red');
    xAxis.setAttribute('rotation', '0 0 90'); // Rotate to align with the X-axis
    el.appendChild(xAxis);

    // Y-axis cylinder (green)
    const yAxis = document.createElement('a-cylinder');
    yAxis.setAttribute('position', `0 ${axisLength / 2} 0`);
    yAxis.setAttribute('radius', axisThickness);
    yAxis.setAttribute('height', axisLength);
    yAxis.setAttribute('color', 'green');
    el.appendChild(yAxis);

    // Z-axis cylinder (blue)
    const zAxis = document.createElement('a-cylinder');
    zAxis.setAttribute('position', `0 0 ${axisLength / 2}`);
    zAxis.setAttribute('radius', axisThickness);
    zAxis.setAttribute('height', axisLength);
    zAxis.setAttribute('color', 'blue');
    zAxis.setAttribute('rotation', '90 0 0'); // Rotate to align with the Z-axis
    el.appendChild(zAxis);
  }
});

// Load bodies and their STL files
function loadBodies() {
  fetch('/bodies')
    .then(response => response.json())
    .then(links => {
      links.forEach(link => {
        // Create an entity for each STL file using custom loader
        const entity = document.createElement('a-entity');
        entity.setAttribute('id', link); // Set ID to match body part name
        entity.setAttribute('stl-model-loader', { src: `/stl/${link}.stl` }); // Use custom STL loader

        // Add coordinate frame to show axes
        // entity.setAttribute('coordinate-frame', ''); // Adds the coordinate frame component
        document.querySelector('a-scene').appendChild(entity);
      });
    })
    .catch(error => console.error('Error loading bodies:', error));
}

// WebSocket connection
let socket;
const leftController = document.getElementById('leftController');
const rightController = document.getElementById('rightController');
const rightControllerFrame = document.getElementById('rightControllerFrame');

var TO_CANONICAL = new THREE2.Matrix4(); 
TO_CANONICAL.set(
  0, 1, 0, 0, 
  0, 0, 1, 0,
  1, 0, 0, 0, 
  0, 0, 0, 1
);
TO_CANONICAL = TO_CANONICAL.transpose()

function setPose(entity, matrix) {
  var position = new THREE.Vector3();
  var quaternion = new THREE.Quaternion();
  var scale = new THREE.Vector3();

  // matrix.transpose().decompose(position, quaternion, scale);
  matrix.decompose(position, quaternion, scale);

  // Update the entity’s position, rotation, and scale
  entity.object3D.position.copy(position);
  entity.object3D.quaternion.copy(quaternion);
  entity.object3D.scale.copy(scale);
}

// Initialize WebSocket connection
function initWebSocket() {
  socket = new WebSocket('wss://' + WSS_HOST + ':' + WSS_PORT); // Change to your server address

  socket.onopen = function() {
    console.log('WebSocket connection established');
    if (leftController && rightController) {
      sendControllerPose(leftController, rightController);
    }
};

  socket.onmessage = function(event) {
    // console.log('Message from server:', event.data);

    // Parse the incoming message
    const poses = JSON.parse(event.data);

    // Loop through each body in the poses
    Object.keys(poses).forEach(bodyName => {
      const poseMatrix = poses[bodyName];  // This is the 4x4 matrix for the current body

      // Find the corresponding A-Frame entity by body name
      const bodyEntity = document.querySelector(`[id="${bodyName}"]`);
      if (bodyEntity) {
        // Apply the 4x4 transformation matrix to the entity
        var position = new THREE.Vector3();
        var quaternion = new THREE.Quaternion();
        var scale = new THREE.Vector3();

        // Flatten the 4x4 matrix and use THREE.js to decompose it
        const matrix = new THREE2.Matrix4().fromArray(poseMatrix.flat());
        // const matrix = new THREE2.Matrix4().multiplyMatrices(TO_VR_FRAME, matrix_teleop);
        matrix.transpose().decompose(position, quaternion, scale);

        // Update the entity’s position, rotation, and scale
        bodyEntity.object3D.position.copy(position.add(INITIAL_SHIFT));
        bodyEntity.object3D.quaternion.copy(quaternion);
        bodyEntity.object3D.scale.copy(scale);
      }
    });

    if (leftController && rightController) {
      setPose(rightControllerFrame, rightController.object3D.matrix.clone().premultiply(TO_CANONICAL));
      sendControllerPose(leftController, rightController);
    }
  };

  socket.onclose = function() {
    console.log('WebSocket connection closed');
  };
}

function getControllerData(controller) {
  const pose = controller.object3D.matrix.clone().premultiply(TO_CANONICAL);

  // Find the corresponding gamepad
  const gamepad = controller.components['tracked-controls']?.controller.gamepad;
  var buttons = [];
  var axes = [];
  if (gamepad) {
    buttons = gamepad?.buttons.map(button => ({
      pressed: button.pressed,
      value: button.value
    })) || [];
    // const buttons = [];
    axes = gamepad?.axes || [];
  }

  var pose_data = pose.elements;
  if (pose_data instanceof Float32Array) {
      pose_data = Array.from(pose_data);
  }

  const data = {
    pose: pose_data,
    buttons,
    axes
  };
  return data;
}

// Send controller pose data
function sendControllerPose(leftController, rightController) {
  const controllersData = {
    leftController: getControllerData(leftController),
    rightController: getControllerData(rightController),
  };

  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(controllersData));
  }
}

window.onload = function () {
  initWebSocket();
  loadBodies();
};
