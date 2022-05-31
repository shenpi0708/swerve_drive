
var speed = 0.5;
var web_control = true;
var twist = new ROSLIB.Message({
    linear : {
    x : 0.0,
    y : 0.0,
    z : 0.0
    },
    angular : {
    x : 0.0,
    y : 0.0,
    z : 0.0
    }
});

function range_change(data){
    // change speed`
    document.getElementById("speed_enter").value = data;
    speed = data
}

function enter_change(data){
    // change speed
    document.getElementById("speed_range").value = data;
    speed = data
}

function control_type(data){
    // button or keybord
    if(data==0){
        document.getElementById("button_enable").style = "width:120px;background-color: #d1d1d1";
        document.getElementById("keybord_enable").style = "width:120px;background-color: #ffffff";
        KeyboardState(false);
        web_control = true;
        console.log("button control");
    }else if(data==1){
        document.getElementById("button_enable").style = "width:120px;background-color: #ffffff";
        document.getElementById("keybord_enable").style = "width:120px;background-color: #d1d1d1";
        KeyboardState(true);
        web_control = false;
        console.log("keyboard control");
    }
}

function movement(data){
    if (web_control == true){
        var x_speed = 0;
        var y_speed = 0;
        var z_speed = 0;
        if(data==1){
            x_speed = 1;
            y_speed = 1;
            z_speed = 0;
        }else if(data==2){
            x_speed = 1;
            y_speed = 0;
            z_speed = 0;
        }else if(data==3){
            x_speed = 1;
            y_speed = -1;
            z_speed = 0;
        }else if(data==4){
            x_speed = 0;
            y_speed = 0;
            z_speed = 1;
        }else if(data==5){
            x_speed = 0;
            y_speed = 1;
            z_speed = 0;
        }else if(data==6){
            x_speed = 0;
            y_speed = 0;
            z_speed = 0;
        }else if(data==7){
            x_speed = 0;
            y_speed = -1;
            z_speed = 0;
        }else if(data==8){
            x_speed = 0;
            y_speed = 0;
            z_speed = -1;
        }else if(data==9){
            x_speed = -1;
            y_speed = 1;
            z_speed = 0;
        }else if(data==10){
            x_speed = -1;
            y_speed = 0;
            z_speed = 0;
        }else if(data==11){
            x_speed = -1;
            y_speed = -1;
            z_speed = 0;
        }else{
            x_speed = 0;
            y_speed = 0;
            z_speed = 0;
        }

        twist = new ROSLIB.Message({
            linear : {
            x : 1*x_speed*speed,
            y : 1*y_speed*speed,
            z : 0.0
            },
            angular : {
            x : 0.0,
            y : 0.0,
            z : 1*z_speed*speed
            }
        });
        pub_cmdVel.publish(twist)
    }
}
