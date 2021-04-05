// Update attitude and heading indicator
$(document).ready(function () {
// Save origin url
	var start = false
	var indicatorInterval;
	var attitude = $.flightIndicator('#attitude', 'attitude', {roll:0, pitch:0, size:200, showBox: true, img_directory: "static/img/"});
	var heading = $.flightIndicator('#heading', 'heading', {heading:0, showBox:true, img_directory: "static/img/"});
	$('#FlightIndicator').hide()
	$('#StartStopIndicator').click(function () {
		if (start == false) {
			indicatorInterval = setInterval(function() {
				$.ajax({
					url: location.origin + "/attitude"
				}).then(function(result) {
					// Attitude update
				attitude.setRoll(result.attitude[0]*180/Math.PI);
				attitude.setPitch(result.attitude[1]*180/Math.PI);
				
				// Heading update
				heading.setHeading(result.attitude[2]*180/Math.PI);
				});
			}, 300);
			start = true;
			$('#StartStopIndicator').prop('value', "Hide Flight Indicator");
			$('#FlightIndicator').show()
		} else {
			clearInterval(indicatorInterval);
			start = false;
			$('#StartStopIndicator').prop('value', "Show Flight Indicator");
			$('#FlightIndicator').hide()
		}
	});
});
