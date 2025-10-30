function setupRosConnection() {
  const ros = new ROSLIB.Ros({
    url: 'ws://pi5.local:9090' // Adjust the URL as needed
  });
  
  ros.on('connection', function() {
    console.log('Connected to ROS');
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

function resetRosConnection() {
  if (!resetInProgress) {
    try {
      resetInProgress = true;
      ros = setupRosConnection();
      subscribe_to_battery();
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

// Initialize ROS connection
var ros = null
var resetInProgress = false;
resetRosConnection();

var watchdogTimer = setInterval(function() {
  if (!ros || !ros.isConnected) {
    document.getElementById('voltage').textContent = '--';
    document.getElementById('current').textContent = '--';
    document.getElementById('percentage').textContent = '--';

    console.log('ROS connection lost. Attempting to reconnect...');
    try {
      resetRosConnection();
    } 
    catch (error) {
      console.error('Reconnection attempt failed');
    }
  }
}, 5000);
