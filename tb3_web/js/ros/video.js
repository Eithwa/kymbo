//===========================================================================
//new

function Camera_Switch(value) {
    var view = document.getElementById("Camera-View");

    if (view.checked) {
        viewflag = value;
    } else {
        viewflag = 'off';
    }
}

function Open_Camera(value) {

    var video = document.getElementById("player");
    var view = document.getElementById("Camera-View");
    var mode = parseInt(document.getElementById('HSVSelect').value);
    Camera_Switch(value);
    if (view.checked) {
        video.src = "img/black.png";
        if (viewflag == 'src') {
            video.style.height = "100%";
            video.style.top="0%"
            setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/src"},100);
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/src";
        } else if (viewflag == 'mask') {
            //video.position="relative";
            video.style.top="40%"
            console.log("fuck")
            video.style.height = "60%";
            switch (mode) {
                case 0:
                    Pub_VideoMode(0);
                    break;
                case 1:
                    Pub_VideoMode(1);
                    break;
                case 2:
                    Pub_VideoMode(2);
                    break;
                case 3:
                    Pub_VideoMode(3);
                    break;
                case 4:
                    Pub_VideoMode(4);
                    break;
            }
            console.log(mode);
            setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/mask"},100);
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/mask";
        } else if (viewflag == 'monitor') {
            video.style.top="40%"
            video.style.height = "60%";
            setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/monitor"},100);
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/tb3/monitor";
        } else if (viewflag == 'off') {
            video.src = "img/offline.png";
        }
    } else {
        video.src = "img/offline.png";
    }
    if (Topic_VideoMode_Flag) {
        if (viewflag == 'mask') {
            console.log(viewflag + ' mode : ' + mode);
        } else {
            console.log(viewflag);
        }
    }
}
