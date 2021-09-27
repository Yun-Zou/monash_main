$(document).ready(function () {
    
    let drone_msgs = {
        initialised: false,
        initial_time: 0,
    }
    let drone_heartbeat = 0;

    var rounding_factor = 10000;

    var ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', function () {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
    });

    $("#submit-IP").click(function() {
        let IP = 'ws://' + $("#drone-IP").val() + ':9090';
        $("#submit-IP").attr("disabled",true);
        
        let droneros = new ROSLIB.Ros({
            url: IP
        });

        droneros.on('connection', function () {
            console.log('Connected to Drone websocket server.');
            $("#drone-IP").removeClass("form-border-error").addClass("form-border-success");
            subscribeDronePositionTopics(droneros)
            subscribeDroneCameraTopics(droneros)
        });

        droneros.on('error', function (error) {
            console.log('Error connecting to Drone websocket server: ', error);
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
        });

        droneros.on('close', function () {
            console.log('Connection to Drone websocket server closed.');
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
        });

        setInterval(function () {
            $("#submit-IP").attr("disabled", false);
         }, 1000);
        
    })


    function subscribeDronePositionTopics(droneros) {
        var mavros_vision_pose = new ROSLIB.Topic({
            ros: droneros,
            name: '/mavros/vision_pose/pose',
            messageType: 'geometry_msgs/PoseStamped'
        });

        mavros_vision_pose.subscribe(function (message) {
            let q = message.pose.orientation;
            var yaw = Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
            var pitch = Math.asin(-2.0 * (q.x * q.z - q.w * q.y));
            var roll = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
            $("#drone-position-x").val(Math.round(message.pose.position.x * rounding_factor) / rounding_factor);
            $("#drone-position-y").val(Math.round(message.pose.position.y * rounding_factor) / rounding_factor);
            $("#drone-position-z").val(Math.round(message.pose.position.z * rounding_factor) / rounding_factor);
            $("#drone-orientation-yaw").val(Math.round(yaw * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#drone-orientation-pitch").val(Math.round(pitch * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#drone-orientation-roll").val(Math.round(roll * 180 / Math.PI * rounding_factor) / rounding_factor);
            
            if (drone_heartbeat.initialised == false) {
                drone_heartbeat.initial_time = message.header.stamp.secs
                drone_heartbeat.initialised = true
            }

            let time = message.header.stamp.secs - drone_msgs.initial_time;

            $("#drone-heartbeat").val(time)

        });

    }

    function subscribeDroneCameraTopics(droneros) {
        var image_topic = new ROSLIB.Topic({
            ros: droneros,
            name: '/camera/fisheye1/image_raw/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        });

        image_topic.subscribe(function (message) {
            document.getElementById('my_image').src = "data:image/jpg;base64," + message.data;
        });

    }

});