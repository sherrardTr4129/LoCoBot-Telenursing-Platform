function zeroPanFunction()
{
	panURL = "http://robotcontrol.live:5000/setPanOffset"
        posPanData = {"panOffsetData": 0};
        $.ajax({type: 'POST',
                url: panURL,
                data: JSON.stringify (posPanData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

function zeroTiltFunction()
{
	tiltURL = "http://robotcontrol.live:5000/setTiltOffset"
        posTiltData = {"tiltOffsetData": 0};
        $.ajax({type: 'POST',
                url: tiltURL,
                data: JSON.stringify (posTiltData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });
}

function posPanFunction()
{
        panURL = "http://robotcontrol.live:5000/setPanOffset"
        posPanData = {"panOffsetData": 0.1};
        $.ajax({type: 'POST',
                url: panURL,
                data: JSON.stringify (posPanData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });

	setTimeout(zeroPanFunction, 500);
}

function negPanFunction()
{
        panURL = "http://robotcontrol.live:5000/setPanOffset"
        negPanData = {"panOffsetData": -0.1};
        $.ajax({type: 'POST',
                url: panURL,
                data: JSON.stringify (negPanData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });

	setTimeout(zeroPanFunction, 500);
}

function posTiltFunction()
{
        tiltURL = "http://robotcontrol.live:5000/setTiltOffset"
        posTiltData = {"tiltOffsetData": 0.1};
        $.ajax({type: 'POST',
                url: tiltURL,
                data: JSON.stringify (posTiltData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });

	setTimeout(zeroTiltFunction, 500);
}

function negTiltFunction()
{
        tiltURL = "http://robotcontrol.live:5000/setTiltOffset"
        negTiltData = {"tiltOffsetData": -0.1};
        $.ajax({type: 'POST',
                url: tiltURL,
                data: JSON.stringify (negTiltData),
                success: function(data) {  },
                contentType: "application/json",
                dataType: 'json'
        });

	setTimeout(zeroTiltFunction, 500);
}

