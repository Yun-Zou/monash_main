$(document).ready(function () {
    
    let droneros;
    let command_request;

    let connections = {
        drone: false,
        local: false
    }

    let feeds = {
        camera_feed: false,
        additional_feed: false,
        additional_source: null
    }

    let drone_msgs = {
        initialised: false,
        initial_time: 0,
        current_time: 0
    }

    let target_msgs = {
        initialised: false,
        current_time: 0
    }

    let image_sources = {
        camera: {
            topic: '/camera/fisheye1/image_raw/compressed',
            msg_type: 'sensor_msgs/CompressedImage'
        },
        apriltags: {
            topic: 'tag_detections_image/compressed',
            msg_type: 'sensor_msgs/CompressedImage'
        },
        undistorted: {
            topic: '/camera/fisheye1/rect/image/compressed',
            msg_type: 'sensor_msgs/CompressedImage'
        },
        depth: {
            topic: '/camera/infra1/image_rect_raw',
            msg_type: 'sensor_msgs/CompressedImage'
        }
    }

    let image_topic
    let additional_image_topic;

    let command = ["Grounded","Hover","Flight","Search","Follow","RTL","Land","TakeOff"];    

    var rounding_factor = 100;

    $("#submit-IP").click(function() {
        let IP = 'ws://' + $("#drone-IP").val() + ':9090';
        $("#submit-IP").attr("disabled",true);
        
        droneros = new ROSLIB.Ros({
            url: IP
        });

        command_request = new ROSLIB.Service({
            ros: droneros,
            name: '/monash_motion/request_command',
            serviceType: 'monash_main/RequestAction'
        });

        droneros.on('connection', function () {
            console.log('Connected to Drone websocket server.');
            $("#drone-IP").removeClass("form-border-error").addClass("form-border-success");
            
            subscribeDronePositionTopics(droneros)
            subscribeTargetPositionTopics(droneros)
            
            enableButtons()
            enableCommands()
            connections.drone = true;
        });

        droneros.on('error', function (error) {
            console.log('Error connecting to Drone websocket server: ', error);
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
            
            disableButtons()
            disableCommands()
            connections.drone = false;
        });

        droneros.on('close', function () {
            console.log('Connection to Drone websocket server closed.');
            $("#drone-IP").removeClass("form-border-success").addClass("form-border-error");
            
            disableButtons()
            disableCommands()
            connections.drone = false;
        });

        setInterval(function () {
            $("#submit-IP").attr("disabled", false);
         }, 1000);
        
    })

    $("#show-camera").click(function() {
        if (connections.drone) {
            if (!feeds.camera_feed) {
                $("#show-camera").addClass("btn-danger").removeClass("btn-warning").html("Close Camera Image")
                $("#show-maps").addClass("btn-danger").removeClass("btn-warning")
                subscribeDroneCameraTopics(droneros)

                feeds.camera_feed = true
                feeds.additional_feed = true
                feeds.additional_source = ''
            } else {
                $("#show-camera").addClass("btn-warning").removeClass("btn-danger").html("Show Camera Image")
                $("#show-maps").addClass("btn-warning").removeClass("btn-danger")
                unsubscribeDroneCameraTopics();

                feeds.camera_feed = false
                feeds.additional_feed = false
                feeds.additional_source = ''
            }
        }
    })

    $("#show-apriltags").click(function () {
        if (connections.drone) {
            if (!feeds.additional_feed || feeds.additional_source != image_sources.apriltags) {
                unsubscribeAdditionalImages();
                $("#additional-title").html("Apriltags")
                feeds.additional_feed = true;
                feeds.additional_source = image_sources.apriltags;
                subscribeAdditionalImages(droneros,feeds.additional_source);
            }
        }
    })


    $("#show-undistorted").click(function () {
        if (connections.drone) {
            if (!feeds.additional_feed || feeds.additional_source != image_sources.undistorted) {
                unsubscribeAdditionalImages();
                $("#additional-title").html("Undistorted")
                feeds.additional_feed = true;
                feeds.additional_source = image_sources.undistorted;
                subscribeAdditionalImages(droneros, feeds.additional_source);
            }
        }
    })

    $("#show-depth").click(function () {
        if (connections.drone) {
            if (!feeds.additional_feed || feeds.additional_source != image_sources.depth) {
                unsubscribeAdditionalImages();
                $("#additional-title").html("Depth Map")
                feeds.additional_feed = true;
                feeds.additional_source = image_sources.depth;
                subscribeAdditionalImages(droneros, feeds.additional_source);
            }
        }
    })

    $("#clear-maps").click(function () {
        if (connections.drone && feeds.additional_feed) {
            unsubscribeAdditionalImages();
            $("#additional-title").html("Additional View")
            feeds.additional_feed = false
            feeds.additional_source = null
        }
    })

    $("#command-takeoff").click(function () {
        var request = getActionRequest("TakeOff");
        console.log(request);
        console.log(command_request);
        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });

    })

    $("#command-land").click(function () {
        request = getActionRequest("Land");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });

    })

    $("#command-move").click(function () {
        let x = $("#command-relative-x").val();
        let y = $("#command-relative-y").val();
        let z = $("#command-relative-z").val();
        let psi = $("#command-relative-psi").val();
        
        request = getActionRequest("Flight");
        request.param1 = x;
        request.param2 = y;
        request.param3 = z;
        request.param4 = psi;

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    $("#command-search").click(function () {

        request = getActionRequest("Search");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    $("#command-follow").click(function () {

        request = getActionRequest("Follow");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    $("#command-home").click(function () {

        request = getActionRequest("RTL");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    $("#command-clear").click(function () {

        request = getActionRequest("Hover");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    function getActionRequest(mode) {
        let commandID = command.findIndex(element => element == mode);
        var request = new ROSLIB.ServiceRequest({
            command: commandID,
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0
        });

        return request;
    }


    function subscribeDronePositionTopics(droneros) {
        var mavros_vision_pose = new ROSLIB.Topic({
            ros: droneros,
            name: '/mavros/global_position/local',
            messageType: 'nav_msgs/Odometry'
        });

        var flight_status = new ROSLIB.Topic({
            ros: droneros,
            name: '/monash_motion/flight_mode',
            messageType: 'std_msgs/String'
        });

        mavros_vision_pose.subscribe(function (msg) {
            let pose = msg.pose.pose;
            let q = pose.orientation;
            var yaw = Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
            var pitch = Math.asin(-2.0 * (q.x * q.z - q.w * q.y));
            var roll = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
            $("#drone-position-x").val(Math.round(pose.position.x * rounding_factor) / rounding_factor);
            $("#drone-position-y").val(Math.round(pose.position.y * rounding_factor) / rounding_factor);
            $("#drone-position-z").val(Math.round(pose.position.z * rounding_factor) / rounding_factor);
            $("#drone-orientation-yaw").val(Math.round(yaw * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#drone-orientation-pitch").val(Math.round(pitch * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#drone-orientation-roll").val(Math.round(roll * 180 / Math.PI * rounding_factor) / rounding_factor);
            
            if (drone_msgs.initialised == false) {
                drone_msgs.initial_time = msg.header.stamp.secs
                drone_msgs.initialised = true
            }

            let time = msg.header.stamp.secs - drone_msgs.initial_time;

            $("#drone-heartbeat").val(time)
            
        });
        flight_status.subscribe(function (msg) {

            $("#drone-flight-mode").val(msg.data)
        });
    }

    function update_target_time() {
        $("#tag-lastseen").val((Date.now() - target_msgs.current_time)/1000);
    }

    function subscribeTargetPositionTopics(droneros) {
        var target_pose = new ROSLIB.Topic({
            ros: droneros,
            name: '/monash_perception/target',
            messageType: 'geometry_msgs/PoseStamped'
        });

        target_pose.subscribe(function (msg) {
            let pose = msg.pose;
            let q = pose.orientation;
            var yaw = Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
            var pitch = Math.asin(-2.0 * (q.x * q.z - q.w * q.y));
            var roll = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
            $("#tag-position-x").val(Math.round(pose.position.x * rounding_factor) / rounding_factor);
            $("#tag-position-y").val(Math.round(pose.position.y * rounding_factor) / rounding_factor);
            $("#tag-position-z").val(Math.round(pose.position.z * rounding_factor) / rounding_factor);
            $("#tag-orientation-yaw").val(Math.round(yaw * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#tag-orientation-pitch").val(Math.round(pitch * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#tag-orientation-roll").val(Math.round(roll * 180 / Math.PI * rounding_factor) / rounding_factor);

            if (target_msgs.initialised == false) {
                target_msgs.initialised = true
                setInterval(update_target_time,100);
            }

            target_msgs.current_time = Date.now()

        });
    }

    function subscribeDroneCameraTopics(droneros) {
        image_topic = new ROSLIB.Topic({
            ros: droneros,
            name: image_sources.camera.topic,
            messageType: image_sources.camera.msg_type
        });

        image_topic.subscribe(function (message) {
            document.getElementById('drone-camera').src = "data:image/jpg;base64," + message.data;
        });
    }

    function unsubscribeDroneCameraTopics() {
        if (image_topic != null) {
            image_topic.unsubscribe();
            document.getElementById('drone-camera').src = ""
        }
    }

    function subscribeAdditionalImages(droneros, source) {
        additional_image_topic = new ROSLIB.Topic({
            ros: droneros,
            name: source.topic,
            messageType: source.msg_type
        });

        additional_image_topic.subscribe(function (message) {
            document.getElementById('drone-additional').src = "data:image/jpg;base64," + message.data;
        });

    }

    function unsubscribeAdditionalImages() {
        if (additional_image_topic != null) {
            additional_image_topic.unsubscribe();
            document.getElementById('drone-additional').src = ""
        }
    }
    
    function enableButtons() {
        $("#show-camera").attr('disabled', false)
        $("#show-maps").attr('disabled', false)
    }

    function disableButtons() {
        $("#show-camera").attr('disabled', true)
        $("#show-maps").attr('disabled', true)
    }

    function enableCommands() {
        $("#command-takeoff").attr('disabled', false)
        $("#command-land").attr('disabled', false)
        $("#command-move").attr('disabled', false)
        $("#command-search").attr('disabled', false)
        $("#command-follow").attr('disabled', false)
        $("#command-clear").attr('disabled', false)
        $("#command-home").attr('disabled', false)
        $("#command-relative-x").attr('disabled', false)
        $("#command-relative-y").attr('disabled', false)
        $("#command-relative-z").attr('disabled', false)
        $("#command-relative-psi").attr('disabled', false)
        
    }

    function disableCommands() {
        $("#command-takeoff").attr('disabled', true)
        $("#command-land").attr('disabled', true)
        $("#command-move").attr('disabled', true)
        $("#command-search").attr('disabled', true)
        $("#command-follow").attr('disabled', true)
        $("#command-clear").attr('disabled', true)
        $("#command-home").attr('disabled', true)
        $("#command-relative-x").attr('disabled', true)
        $("#command-relative-y").attr('disabled', true)
        $("#command-relative-z").attr('disabled', true)
        $("#command-relative-psi").attr('disabled', true)
    }

});