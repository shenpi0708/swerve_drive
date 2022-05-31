var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

var pub_cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/test_cmd_Vel',
    messageType : 'geometry_msgs/Twist'
});

// init topic
pub_cmdVel.publish(twist)
