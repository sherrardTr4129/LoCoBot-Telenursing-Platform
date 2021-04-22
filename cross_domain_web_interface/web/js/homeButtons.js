function homeArmFunction(){
        homeArmURL = "http://robotcontrol.live:5000/homeArm"
	homeArmData = {"homeArmData": 1};
	$.ajax({type: 'POST',
		url: homeArmURL,
		data: JSON.stringify (homeArmData),
		success: function(data) {  },
		contentType: "application/json",
		dataType: 'json'
	});
}

function homeCameraFunction(){
	homeCameraURL = "http://robotcontrol.live:5000/homeCamera"
        homeCameraData = {"homeCameraData": 1};
        $.ajax({type: 'POST',
                url: homeCameraURL,
                data: JSON.stringify (homeCameraURL),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

