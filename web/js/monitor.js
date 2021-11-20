// JS file which controls monitor.js client. Uses JQuery for controlling/manipulating HTML front-end content
// Uses ROSLibJS to communicate with ROS

$(document).ready(function () {
    
    // Holds IP value once you type it in
    let IP;
    
    // Holds the ROS URL object
    let droneros;
    
    // Holds the ROS Service object for making command requests
    let command_request;
    
    // Whether a connection is made
    let connections = {
        drone: false,
    }

    // Factor used to round values. 100 => 2 decimal points
    var rounding_factor = 100;

    // Tracks drone pose time
    let drone_msgs = {
        initialised: false,
        initial_time: 0,
        current_time: 0
    }

    // Tracks the last received target time
    let target_msgs = {
        initialised: false,
        current_time: 0
    }

    // List of commands to request. Value must be exact as values in enum state in FlightController.cpp from monash_motion
    let command = ["Grounded", "Hover", "Flight", "Circle", "Search", "Follow", "RTL", "Land", "TakeOff"];


    // Image stream ROS topics
    let image_sources = {
        camera: {
            topic: '/camera/fisheye1/image_raw'
        },
        apriltags: {
            topic: '/tag_detections_image'
        },
        undistorted: {
            topic: '/camera/fisheye1/rect/image'
        },
        depth: {
            topic: '/disparity'
        }
    }

    // Tracks image feed status and source
    let feeds = {
        camera_feed: false,
        additional_feed: false,
        additional_source: null
    }

    // Set the quality of the stream. Helps reducing lag and dropped frames
    let quality = 80;


    /**
     * 
     * Called: When submit button is cliked in IP form 
     * Description: Initialises monitoring, ROSLib objects and subcribes to ROS Topics
     */
    $("#submit-IP").click(function() {

        IP = $("#drone-IP").val();
        let link = 'ws://' + IP + ':9090'; // IP form as required by ROSBridge 
        $("#submit-IP").attr("disabled",true);
        
        droneros = new ROSLIB.Ros({
            url: link
        });

        // ROS Service RequestAction defined in monash_main
        command_request = new ROSLIB.Service({
            ros: droneros,
            name: '/monash_motion/request_command',
            serviceType: 'monash_main/RequestAction'
        });

        // Subscribe/unsubscribe and enable/disable buttons depending on connection state
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

        // Disable the submit IP button for a second to prevent spam
        setInterval(function () {
            $("#submit-IP").attr("disabled", false);
         }, 1000);
        
    })

    /**
     * 
     * @param {Object} droneros ROSLib Object with the Drone IP
     * Description: Subscribe to the local pose published by mavros on the drone. Turn angle from quaternion into yaw,pitch, roll
     * Display the information on the frontend
     */
    function subscribeDronePositionTopics(droneros) {
        
        // Subscribe to local position published by mavros
        var mavros_vision_pose = new ROSLIB.Topic({
            ros: droneros,
            name: '/mavros/global_position/local',
            messageType: 'nav_msgs/Odometry'
        });

        // Convert quaternion into yaw, pitch and roll. Display pose on monitor
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

            // If first message, set initial time
            if (drone_msgs.initialised == false) {
                drone_msgs.initial_time = msg.header.stamp.secs
                drone_msgs.initialised = true
            }

            // Set time of last message since first received
            let time = msg.header.stamp.secs - drone_msgs.initial_time;
            $("#drone-heartbeat").val(time)

        });

        // Subscribe to flight mode status published by monash_motion
        var flight_status = new ROSLIB.Topic({
            ros: droneros,
            name: '/monash_motion/flight_mode',
            messageType: 'std_msgs/String'
        });

        // Display the flight mode data
        flight_status.subscribe(function (msg) {
            $("#drone-flight-mode").val(msg.data)
        });
    }

    /**
     * Description: Update last seen time for the target on the monitor
     */
    function update_target_time() {
        $("#tag-lastseen").val((Date.now() - target_msgs.current_time) / 1000);
    }

    /**
     * 
     * @param {Object} droneros ROSLib Object with the Drone IP
     * Description: Subscribe to the monash_perception published target coordinates. Turn angle from quaternion into yaw,pitch, roll
     * Display the information on the frontend
     */
    function subscribeTargetPositionTopics(droneros) {
        
        // Subscribe to target pose published to monash_perception
        var target_pose = new ROSLIB.Topic({
            ros: droneros,
            name: '/monash_perception/target',
            messageType: 'geometry_msgs/PoseStamped'
        });

        // Convert target quaternion into yaw, pitch and roll. Display pose on monitor
        target_pose.subscribe(function (msg) {

            let pose = msg.pose;
            let q = pose.orientation;

            // Transform quaternion into yaw, pitch, roll
            var yaw = Math.atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
            var pitch = Math.asin(-2.0 * (q.x * q.z - q.w * q.y));
            var roll = Math.atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

            // Display the pose of the target. Round the numbers
            $("#tag-position-x").val(Math.round(pose.position.x * rounding_factor) / rounding_factor);
            $("#tag-position-y").val(Math.round(pose.position.y * rounding_factor) / rounding_factor);
            $("#tag-position-z").val(Math.round(pose.position.z * rounding_factor) / rounding_factor);
            $("#tag-orientation-yaw").val(Math.round(yaw * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#tag-orientation-pitch").val(Math.round(pitch * 180 / Math.PI * rounding_factor) / rounding_factor);
            $("#tag-orientation-roll").val(Math.round(roll * 180 / Math.PI * rounding_factor) / rounding_factor);

            // If first message, start target timer
            if (target_msgs.initialised == false) {
                target_msgs.initialised = true
                setInterval(update_target_time, 100);
            }

            target_msgs.current_time = Date.now()

        });
    }

    /**
     * 
     * Called: When Camera button is click
     * Description: Show the raw image stream
     */
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

    /**
     * 
     * Called: When AprilTag image stream button is clicked
     * Description: Show AprilTags image stream in the additional image spot
     */
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

    /**
     *
     * Called: When Undistorted image stream button is clicked
     * Description: Show Undistorted image stream in the additional image spot
     */
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

    /**
     *
     * Called: When Depth image stream button is clicked
     * Description: Show Depth image stream in the additional image spot
     */
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

    /**
     *
     * Called: When Clear Additional Image button is clicked
     * Description: Stop showing additional image
     */
    $("#clear-maps").click(function () {
        if (connections.drone && feeds.additional_feed) {
            unsubscribeAdditionalImages();
            $("#additional-title").html("Additional View")
            feeds.additional_feed = false
            feeds.additional_source = null
        }
    })

    /**
     *
     * Called: When TakeOff command button is clicked
     * Description: Send TakeOff request to monash_motion. Try to take off if ArduPilot conditions are met. Will need to be in Guided mode
     */
    $("#command-takeoff").click(function () {
        request = getActionRequest("TakeOff");
        
        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });

    })

    /**
     *
     * Called: When Land command button is clicked
     * Description: Send Land request to monash_motion. Request drone to land.
     */
    $("#command-land").click(function () {
        request = getActionRequest("Land");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });

    })

    /**
     *
     * Called: When Move command button is clicked
     * Description: Send Move request to monash_motion. Request drone to move to absolute coordinates
     */
    $("#command-move").click(function () {
        let x = $("#command-aboslute-x").val();
        let y = $("#command-aboslute-y").val();
        let z = $("#command-aboslute-z").val();
        let psi = $("#command-aboslute-psi").val();
        
        request = getActionRequest("Flight");
        
        // These parameters are read by monash_motion. Refer to set_flight_mode function in FlightController.cpp
        request.param1 = Number.parseFloat(x);
        request.param2 = Number.parseFloat(y);
        request.param3 = Number.parseFloat(z);
        request.param4 = Number.parseFloat(psi);

        console.log(request);
        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     *
     * Called: When Circle command button is clicked
     * Description: Send Circle request to monash_motion. Request drone to move to absolute coordinates.
     * Dir refers to direction the drone faces. 0 = Forward. 1 = Inward towards center. 2 = Outwards away from center
     */
    $("#command-circle").click(function () {
        let x = $("#command-circle-x").val();
        let y = $("#command-circle-y").val();
        let r = $("#command-circle-radius").val();
        let dir = $("#command-circle-direction").val();

        request = getActionRequest("Circle");

        // These parameters are read by monash_motion. Refer to set_flight_mode function in FlightController.cpp
        request.param1 = Number.parseFloat(x);
        request.param2 = Number.parseFloat(y);
        request.param3 = Number.parseFloat(r);
        request.param4 = Number.parseFloat(dir);

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     *
     * Called: When Search command button is clicked
     * Description: Send Search request to monash_motion. Perform search grid looking for target
     */
    $("#command-search").click(function () {

        request = getActionRequest("Search");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     *
     * Called: When Follow command button is clicked
     * Description: Send Follow request to monash_motion. Try follow target if available.
     */
    $("#command-follow").click(function () {

        request = getActionRequest("Follow");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     *
     * Called: When Home command button is clicked
     * Description: Send Return to Home request to monash_motion
     */
    $("#command-home").click(function () {

        request = getActionRequest("RTL");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     *
     * Called: When Clear command button is clicked
     * Description: Send Clear request to monash_motion. Stop drone and clear future waypoints
     */
    $("#command-clear").click(function () {

        request = getActionRequest("Hover");

        command_request.callService(request, function (result) {
            console.log('Result for service call on ' + command_request.name + ': '
                + result.success);
        });
    })

    /**
     * 
     * @param {string} mode Exact string of mode that is to be requested. Must match a string in `command` array
     * @returns Return generic form of flight mode service request and fill in the requested mode
     */
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

    /**
     * 
     * Description: Show the image of the raw image stream received. Implemented using web_video_server package.
     */
    function subscribeDroneCameraTopics() {
        document.getElementById('drone-camera').src = "http://" + IP + ":8080/stream?topic=" + image_sources.camera.topic + "&quality=" + quality;
    }

    /**
     * 
     * @param {string} source additional image topic string
     * Description: Show the image of the additional image stream received. Implemented using web_video_server package.
     */
    function subscribeAdditionalImages(source) {
        document.getElementById('drone-additional').src = "http://" + IP + ":8080/stream?topic=" + source.topic + "&quality=" + quality;
    }

    /**
     * Description: Stop displaying raw image stream
     */
    function unsubscribeAdditionalImages() {
        document.getElementById('drone-additional').src = "";
    }

    /**
     * Description: Stop displaying additional image stream
     */
    function unsubscribeDroneCameraTopics() {
        document.getElementById('drone-camera').src = ""
    }
    
    /**
     * Description: Enable the image stream buttons
     */
    function enableButtons() {
        $("#show-camera").attr('disabled', false)
        $("#show-maps").attr('disabled', false)
    }

    /**
     * Description: Disable the image stream buttons
     */
    function disableButtons() {
        $("#show-camera").attr('disabled', true)
        $("#show-maps").attr('disabled', true)
    }

    /**
     * Description: Enable the command inputs
     */
    function enableCommands() {
        $("#command-takeoff").attr('disabled', false)
        $("#command-land").attr('disabled', false)
        $("#command-move").attr('disabled', false)
        $("#command-circle").attr('disabled', false)
        $("#command-search").attr('disabled', false)
        $("#command-follow").attr('disabled', false)
        $("#command-clear").attr('disabled', false)
        $("#command-home").attr('disabled', false)
        $("#command-aboslute-x").attr('disabled', false)
        $("#command-aboslute-y").attr('disabled', false)
        $("#command-aboslute-z").attr('disabled', false)
        $("#command-aboslute-psi").attr('disabled', false)
        $("#command-circle-x").attr('disabled', false)
        $("#command-circle-y").attr('disabled', false)
        $("#command-circle-radius").attr('disabled', false)
        $("#command-circle-direction").attr('disabled', false)
        
    }

    /**
     * Description: Disable the command inputs
     */
    function disableCommands() {
        $("#command-takeoff").attr('disabled', true)
        $("#command-land").attr('disabled', true)
        $("#command-move").attr('disabled', true)
        $("#command-circle").attr('disabled', true)
        $("#command-search").attr('disabled', true)
        $("#command-follow").attr('disabled', true)
        $("#command-clear").attr('disabled', true)
        $("#command-home").attr('disabled', true)
        $("#command-aboslute-x").attr('disabled', true)
        $("#command-aboslute-y").attr('disabled', true)
        $("#command-aboslute-z").attr('disabled', true)
        $("#command-aboslute-psi").attr('disabled', true)
        $("#command-circle-x").attr('disabled', true)
        $("#command-circle-y").attr('disabled', true)
        $("#command-circle-radius").attr('disabled', true)
        $("#command-circle-direction").attr('disabled', true)
    }

});