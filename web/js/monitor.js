$(document).ready(function () {
    
    let droneros;
    let local;

    let image_topic;

    let connections = {
        drone: false,
        local: false
    }

    let feeds = {
        camera_feed: false,
        maps_feed: false
    }

    let drone_msgs = {
        initialised: false,
        initial_time: 0,
    }
    let drone_heartbeat = 0;

    var rounding_factor = 10000;

    rosConnect();

    $("#submit-IP").click(function() {
        let IP = 'ws://' + $("#drone-IP").val() + ':9090';
        $("#submit-IP").attr("disabled",true);
        
        droneros = new ROSLIB.Ros({
            url: IP
        });

        droneros.on('connection', function () {
            console.log('Connected to Drone websocket server.');
            $("#drone-IP").removeClass("form-border-error").addClass("form-border-success");
            
            subscribeDronePositionTopics(droneros)
            subscribeDroneCameraTopics(droneros)
            
            enableButtons()
            
            connections.drone = true;
        });

        droneros.on('error', function (error) {
            console.log('Error connecting to Drone websocket server: ', error);
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
            
            disableButtons()
            
            connections.drone = false;
        });

        droneros.on('close', function () {
            console.log('Connection to Drone websocket server closed.');
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
            
            disableButtons()
            connections.drone = false;
        });

        setInterval(function () {
            $("#submit-IP").attr("disabled", false);
         }, 1000);
        
    })

    $("#show-camera").click(function() {
        if (connections.drone) {
            if (feeds.camera_feed) {
                $("#show-camera").addClass("btn-danger").removeClass("btn-warning").html("Close Camera Image")
                unsubscribeDroneCameraTopics(droneros);
                feeds.camera_feed = true
            } else {
                $("#show-camera").addClass("btn-warning").removeClass("btn-danger").html("Show Camera Image")
                subscribeDroneCameraTopics()
                feeds.camera_feed = false
            }
        }
    })

    $("#show-maps").click(function () {
        console.log("show maps")
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
        image_topic = new ROSLIB.Topic({
            ros: droneros,
            name: '/camera/fisheye1/image_raw/compressed',
            messageType: 'sensor_msgs/CompressedImage'
        });

        image_topic.subscribe(function (message) {
            document.getElementById('drone-camera').src = "data:image/jpg;base64," + message.data;
        });
    }

    function unsubscribeDroneCameraTopics() {
        image_topic.unsubscribe();
        document.getElementById('drone-camera').src = ""
    }

    function subscribeDroneRectTopics(ros) {
        console.log("ASDF")
        var rect_topic = new ROSLIB.Topic({
            ros: ros,
            name: '/camera/fisheye1/image_raw/rectified',
            messageType: 'sensor_msgs/CompressedImage'
        });

        rect_topic.subscribe(function (message) {
            console.log("blah")
            document.getElementById('drone-undistorted').src = "data:image/jpg;base64," + message.data;
        });

    }

    function dronerosConnect(){
        
    }

    function rosConnect() {
        ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        ros.on('connection', function () {
            console.log('Connected to websocket server.');
            connections.local = true;
        });

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
            connections.local = false;
        });

        ros.on('close', function () {
            console.log('Connection to websocket server closed.');
            connections.local = false;
        });
    }

    function enableButtons() {
        $("#show-camera").attr('disabled', false)
        $("#show-maps").attr('disabled', false)
    }

    function disableButtons() {
        $("#show-camera").attr('disabled', true)
        $("#show-maps").attr('disabled', true)
    }

});