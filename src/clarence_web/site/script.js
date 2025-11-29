function setupRosConnection() {
  const ros = new ROSLIB.Ros({});
  //const ros = new ROSLIB.Ros({
  //  url: 'ws://pi5.local:9090' // Adjust the URL as needed
  //});
  
  ros.on('connection', function() {
    console.log('Connected to ROS');
    subscribe_to_battery();
    subscribe_to_cpu_temperature();
    subscribe_to_cpu_percent();
    subscribe_to_alsa_gain();
  });

  ros.on('error', function(error) {
    console.error('Error connecting to ROS');
  });
  
  ros.on('close', function() {
    console.log('Connection to ROS closed');
  });
  return ros
}

function setLed(light, isOn) {
  const ledPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/set_led',
    messageType: 'gpio_leds_msgs/Led'
  });
  
  const ledMessage = new ROSLIB.Message({
    number: light,
    is_on: isOn
  });
  ledPublisher.publish(ledMessage);
}

function setLed1(red, green) {
  console.log(`Setting LED1: Red=${red}, Green=${green}`);
  setLed(23, red);
  setLed(24, green);
}

function publishServoMessage() {
    const number = parseInt(document.getElementById('servoNumber').value);
    const position = parseFloat(document.getElementById('servoPercentPosition').value);
    const servoMessage = new ROSLIB.Message({number: number, position: position});
    const servoPublisher = new ROSLIB.Topic({
      ros: ros,
      name: '/set_servo',
      messageType: 'pca9685_servos_msgs/Servo'
    });
    servoPublisher.publish(servoMessage);
}

function publishAlsaGainMessage(device, control, percent) {
  const muted = false;
  
  const alsaGainMessage = new ROSLIB.Message({
    device: device,
    control: control,
    percent: [percent, percent],
    muted: muted
  });
  
  const alsaGainPublisher = new ROSLIB.Topic({
    ros: ros,
    name: '/alsa_gain_set',
    messageType: 'alsa_gain_msgs/AlsaGain'
  });
  
  alsaGainPublisher.publish(alsaGainMessage);
}
function publishTickMessage() {
    const number = parseInt(document.getElementById('servoNumber').value);
    const ticks = parseInt(document.getElementById('servoTickPosition').value);
    const servoTickMessage = new ROSLIB.Message({number: number, ticks: ticks});
    const servoTickPublisher = new ROSLIB.Topic({
      ros: ros,
      name: '/set_servo_ticks',
      messageType: 'pca9685_servos_msgs/ServoByTicks'
    });
    servoTickPublisher.publish(servoTickMessage);
}

function subscribe_to_battery() {
  // Subscribe to BatteryState topi c
  const batterySubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/ups_state',
    messageType: 'sensor_msgs/BatteryState'
  });
  batterySubscriber.subscribe(function(message) {
    document.getElementById('voltage').textContent = message.voltage.toFixed(2);
    document.getElementById('current').textContent = message.current.toFixed(2);
    document.getElementById('percentage').textContent = (message.percentage * 100).toFixed(1);
  });
}

function subscribe_to_cpu_temperature() {
  // Subscribe to CPU Temperature topic
  const tempSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/cpu_temperature',
    messageType: 'sensor_msgs/Temperature'
  });
  tempSubscriber.subscribe(function(message) {
    console.log('Received CPU Temperature:', message.temperature);
    document.getElementById('cpuTemperature').textContent = message.temperature.toFixed(2);
  });
}

function subscribe_to_cpu_percent() {
  // Subscribe to CPU Percent topic
  const cpuPercentSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/cpu_percent',
    messageType: 'std_msgs/Float32'
  });
  cpuPercentSubscriber.subscribe(function(message) {
    console.log('Received CPU Percent:', message.data);
    document.getElementById('cpuPercent').textContent = message.data.toFixed(1);
  });
}

function subscribe_to_alsa_gain() {
  // Subscribe to ALSA Volume topic
  const alsaGainSubscriber = new ROSLIB.Topic({
    ros: ros,
    name: '/alsa_gain',
    messageType: 'alsa_gain_msgs/AlsaGain'
  });
  alsaGainSubscriber.subscribe(function(message) {
    console.log('Received ALSA Gain:', message.percent);
    if (message.device == 'speaker') {
      document.getElementById('alsaSpeakerGain').textContent = message.percent[0].toFixed(0) + '%';
      document.getElementById('alsaSpeakerMuted').textContent = message.muted[0] ? 'Yes' : 'No';
    } else if (message.device == 'mike') {
      document.getElementById('alsaMikeGain').textContent = message.percent[0].toFixed(0) + '%';
      document.getElementById('alsaMikeMuted').textContent = message.muted[0] ? 'Yes' : 'No';
    } else {
      console.warn('Unknown ALSA device:', message.device);
    }
  });
}

function resetRosConnection() {
  if (!resetInProgress) {
    try {
      resetInProgress = true;
      ros.close();
      ros.connect('ws://pi5.local:9090');
    } catch (error) {
      console.error('Reconnection attempt failed:', error);
    }
    finally {
      resetInProgress = false;
    }
  }
}

// Servo Control
document.getElementById("servoPercentPosition").addEventListener("keypress", function(event) {
  if (event.key === "Enter") {
      // Action to perform when Enter is pressed
      event.preventDefault(); // Prevent default form submission if applicable
      // Call a function, submit a form, or trigger other actions
      publishServoMessage();
  }
});
document.getElementById("servoTickPosition").addEventListener("keypress", function(event) {
  if (event.key === "Enter") {
      // Action to perform when Enter is pressed
      event.preventDefault(); // Prevent default form submission if applicable
      // Call a function, submit a form, or trigger other actions
      publishTickMessage();
  }
});

// ALSA Gain Control
document.getElementById("speakerGainSlider").addEventListener("input", function(event) {
    // Action to perform when Enter is pressed
    event.preventDefault(); // Prevent default form submission if applicable
    // Call a function, submit a form, or trigger other actions
    const device ="speaker";
    const control = "Speaker";
    console.log(`Speaker Gain input, ${this.value}`);
    publishAlsaGainMessage(device, control, parseInt(this.value));
});
// ALSA Gain Control
document.getElementById("mikeGainSlider").addEventListener("input", function(event) {
    // Action to perform when Enter is pressed
    event.preventDefault(); // Prevent default form submission if applicable
    // Call a function, submit a form, or trigger other actions
    const device ="mike";
    const control = "mike_softvol";
    console.log(`Mike Gain input, ${this.value}`);
    publishAlsaGainMessage(device, control, parseInt(this.value));
});
// Initialize ROS connection
var ros = setupRosConnection();
var resetInProgress = false;
resetRosConnection();

var watchdogTimer = setInterval(function() {
  if (!ros || !ros.isConnected) {
    document.getElementById('voltage').textContent = '--';
    document.getElementById('current').textContent = '--';
    document.getElementById('percentage').textContent = '--';
    document.getElementById('cpuTemperature').textContent = '--';
    document.getElementById('cpuPercent').textContent = '--';

    console.log('ROS connection lost. Attempting to reconnect...');
    try {
      resetRosConnection();
    } 
    catch (error) {
      console.error('Reconnection attempt failed');
    }
  }
}, 5000);
