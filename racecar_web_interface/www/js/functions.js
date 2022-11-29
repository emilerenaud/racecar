// Functions for APP3
var is_connected = false;

// ----- ROS functions -----

function connectROS(ip_addr) {
    // This function connects to the rosbridge server
    rbServer = new ROSLIB.Ros({
        url : "ws://" + ip_addr + ":9090"
    });

    rbServer.on('connection', function(){
        console.log('Connected to websocket server.');
        add_text_status_box("Connected to Racecar!");
        enable_buttons(true);
        update_video(ip_addr);
        is_connected = true;

        // These lines create a topic object as defined by roslibjs
        cmdVelTopic = new ROSLIB.Topic({
            ros : rbServer,
            name : '/racecar/cmd_vel_abtr_2',
            messageType : 'geometry_msgs/Twist'
        });
    });

    rbServer.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        add_text_status_box("Connection error with the racecar.")
        is_connected = false;
    });

    rbServer.on('close', function() {
        console.log('Connection to websocket server closed.');
        is_connected = false;
        empty_status_box();
        add_text_status_box("Connection closed.");
        enable_buttons(false);
        update_video("");
        consigne_buttons_speed = 0.0;
        consigne_buttons_steering = 0.0;
        consigne_joy_speed = 0.0;
        consigne_joy_steering = 0.0;
    });
}

function test_connect_ROS(ip_addr) {
    // This function connects to the rosbridge server
    rb_test = new ROSLIB.Ros({
        url : "ws://" + ip_addr + ":9090"
    });

    rb_test.on('connection', function(){
        is_connected = true;
    });

    rb_test.on('error', function(error) {
        is_connected = false;
    });

    rb_test.on('close', function() {
        is_connected = false;
    });
}


// ----- Connection functions -----

function connect(){
    // Other way to get the connection values on the dashboard
    window.sessionStorage.setItem("username", document.getElementById("username").value);
    window.sessionStorage.setItem("ip_addr", document.getElementById("ip_addr").value);

    document.getElementById("error_msg").innerHTML = "";

    test_connect_ROS(document.getElementById("ip_addr").value);
    setTimeout(() => {
        if (is_connected) {
            rb_test.close();
            document.getElementById("form_credentials").submit();
        }
        else {
            document.getElementById("error_msg").innerHTML = "Invalid username or IP address.";
        }
    }, 50);
    // document.getElementById("form_credentials").submit();
}

function disconnect(){
    rbServer.close();
    document.getElementById("form_disconnect").submit();
}


// ----- Status functions -----

function empty_status_box(){
    let state = is_connected ? "Connected" : "Disconnected";
    document.getElementById("status_box").innerHTML = header + "Status : " + state + "\n\n";
}

function add_text_status_box(text){
    document.getElementById("status_box").innerHTML += text + "\n";
}

function change_text_status_box(text){
    document.getElementById("status_box").innerHTML = text + "\n";
}


// ----- Video functions -----

function update_video(ip_addr){
    document.getElementById("video_frame").src = "http://" + ip_addr + ":8080/stream?topic=/racecar/raspicam_node/image&type=ros_compressed"
}


// ----- Joystick functions -----

function create_joystick(){
    return new VirtualJoystick({
        container : joystick_container,
        mouseSupport : true,
        stationaryBase : true,
        baseX : 0.5 * joystick_container.clientWidth,
        baseY : 0.5 * joystick_container.clientHeight,
        limitStickTravel : true,
        stickRadius : joystick_radius,
    });
}

function reset_joystick(){
    joystick.destroy();
    joystick_radius = 0.4 * joystick_container.clientHeight;
    joystick = create_joystick();
}


// ----- Button functions -----

function enable_buttons(value){
    document.getElementById("button_avancer").disabled = !value;
    document.getElementById("button_arreter").disabled = !value;
    document.getElementById("button_reculer").disabled = !value;
    document.getElementById("button_gauche").disabled = !value;
    document.getElementById("button_devant").disabled = !value;
    document.getElementById("button_droite").disabled = !value;
}

function deplacer(text, speed){
    add_text_status_box(text);
    consigne_buttons_speed += speed;
}

function arreter(){
    add_text_status_box('Stop!');
    consigne_buttons_speed = 0.0;
}

function tourner(text, angle){
    add_text_status_box(text);
    consigne_buttons_steering = angle;
}
