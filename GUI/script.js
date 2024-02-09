// Create a ROS connection
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

// Create a ROS topic for publishing String messages
const motionTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/motion',
    messageType: 'std_msgs/String'
});

// Create a joystick using NippleJS
const joystick = nipplejs.create({
    zone: document.getElementById('joystick-container'),
    mode: 'static',
    position: { top: '50%', left: '50%' },
    color: 'blue',
    size: 150
});

// Track the previous command
let previousCommand = 'STOP';

// Update string message based on joystick movement
joystick.on('move', (event, data) => {
    let command = '';

    if (data.direction && data.distance > 25) {
        // Move forward, backward, left, or right based on joystick direction
        command = data.direction.angle;
    } else {
        // Stop if the joystick is near the center
        command = 'STOP';
    }

    // Only send a new command if the status of the joystick changes
    if (command !== previousCommand) {
        // Publish the command to the /motion topic
        const motionMessage = new ROSLIB.Message({
            data: command
        });

        motionTopic.publish(motionMessage);

        // Update the previous command
        previousCommand = command;
    }
});

// Stop the turtle on joystick release
joystick.on('end', () => {
    // Send a stop command when the joystick is released
    const stopMessage = new ROSLIB.Message({
        data: 'STOP'
    });

    motionTopic.publish(stopMessage);

    // Reset the previous command on joystick release
    previousCommand = 'STOP';
});

// Function to publish a command to the /motion topic
function publishCommand(command) {
    const motionMessage = new ROSLIB.Message({
        data: command
    });

    motionTopic.publish(motionMessage);
}

function sendCommand() {
    var toggleSwitch = document.getElementById('toggleSwitch');
    var command = {
        data: toggleSwitch.checked // ? 1 : 0
    };

    var topic = new ROSLIB.Topic({
        ros: ros,
        name: '/status',
        messageType: 'std_msgs/Bool'
    });

    var message = new ROSLIB.Message(command);
    topic.publish(message);
}

// Button event listeners
document.getElementById('move-up').addEventListener('click', () => {
    publishCommand('W');
});

document.getElementById('move-left').addEventListener('click', () => {
    publishCommand('A');
});

document.getElementById('move-stop').addEventListener('click', () => {
    publishCommand('STOP');
});

document.getElementById('move-right').addEventListener('click', () => {
    publishCommand('D');
});

document.getElementById('move-down').addEventListener('click', () => {
    publishCommand('S');
});

document.getElementById('greeting').addEventListener('click', () => {
    publishCommand('G');
});

document.getElementById('pushup').addEventListener('click', () => {
    publishCommand('PU');
});

document.getElementById('crawl-left').addEventListener('click', () => {
    publishCommand('CL');
});

document.getElementById('crawl-right').addEventListener('click', () => {
    publishCommand('CR');
});

// ROS connection error handling
ros.on('error', (error) => {
    console.error('ROS connection error:', error);
});

// ROS connection close handling
ros.on('close', () => {
    console.log('ROS connection closed.');
});
