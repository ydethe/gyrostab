// Handle calibration input and sent it to the server
$(document).ready(function () {	
  $("#Calibrate").click(function () {
	var pinNumber = $("#pinNumber").val();
	var valueMinPulse = $("#valueMinPulse").val();
	if (valueMinPulse < 500) { valueMinPulse=500}
	if (valueMinPulse > 2500) { valueMinPulse=2500}
	var valueMaxPulse = $("#valueMaxPulse").val();
	if (valueMaxPulse < 500) { valueMaxPulse=500}
	if (valueMaxPulse > 2500) { valueMaxPulse=2500}
	var valueMinAngle = $("#valueMinAngle").val();
	if (valueMinAngle < -180) { valueMinAngle=-180}
	if (valueMinAngle > 180) { valueMinAngle=180}
	var valueMaxAngle = $("#valueMaxAngle").val();
	if (valueMaxAngle < -180) { valueMaxAngle=-180}
	if (valueMaxAngle > 180) { valueMaxAngle=180}
	
	var url = location.origin + "/servo/calibrate/" + $("#axis").val();
	$.ajax({
	  url: url,
	  type: 'POST',
	  dataType: 'json',
	  data: {servoPin: pinNumber,
			 minPulse: valueMinPulse, maxPulse: valueMaxPulse,
			 minAngle: valueMinAngle, maxAngle: valueMaxAngle},
	  success: function(response){}
	}).done(function(result) {
		$('#recalibration').text(result['status'])});
  });
});
