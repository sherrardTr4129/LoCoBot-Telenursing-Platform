function homeArmFunction(){
        homeArmURL = "http://robotcontrol.live:5000/homeArm"
	homeArmData = {"homeArmData": 1};
	$.ajax({type: 'GET',
		url: homeArmURL,
		success: function(res) { console.log(res); }
	});
}

function homeCameraFunction(){
	homeCameraURL = "http://robotcontrol.live:5000/homeCamera"
        homeCameraData = {"homeCameraData": 1};
        $.ajax({type: 'GET',
                url: homeCameraURL,
                success: function(res) { console.log(res); }
        });
}

