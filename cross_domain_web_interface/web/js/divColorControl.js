/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 15th, 2021
 */

function set_div_color(div_id, color)
{
	var elem_id = "d_" + div_id;
        var element_ref = document.getElementById(elem_id);
        element_ref.style.backgroundColor = color;
}

function set_colors()
{
	const url = "http://robotcontrol.live:5000/getLiDARdata";
	$.ajax({type: 'GET',
                url: url,
                success: function(data) {
			for(var i = 0; i < data.length; i++)
			{
				var color;
				if(data[i] == 0){color = "white";}
				else if(data[i] == 1){color = "yellow";}
				else if(data[i] == 2){color = "red";}
				set_div_color(i + 1, color);
			}
		},
                contentType: "application/json",
                dataType: 'json'
        });
}

setInterval(function(){ set_colors() }, 250);
