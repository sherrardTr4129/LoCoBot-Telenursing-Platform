/* Author: Trevor Sherrard
 * Course: Directed Research
 * Project: Socially Distanced Telenursing
 * Since: June 15th, 2021
 */

/* set color of div based on div_id
   params:
      div_id (int) digit between 1-40.
      color (string) color value to set the div to.

   returns:
      None
*/
function set_div_color(div_id, color)
{
	var elem_id = "d_" + div_id;
        var element_ref = document.getElementById(elem_id);
        element_ref.style.backgroundColor = color;
}

/* update the color of each of the individual scan 'bins'
   based on down-sampled data from the lidar. This function will
   run on a fixed interval of 250 ms. 

   params:
      None
   returns:
      None
*/
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
