// Get displayable data from the server and present it in the selector
$(document).ready(function () {	
	$.ajax({
		url: location.origin + "/display"
	}).then(function(result) {
		$('#AddChartEndPointSelect').children().remove();
		$.each(result, function (i, routes) {
			$('#AddChartEndPointSelect').append($("<option>", { 
				value: routes.route,
				text : routes.title 
			}));
		$('#AddChartEndPointSelect option:first').attr('selected','selected');
		});
	})
});

// Add the selected chart to the ChartTable
$(document).ready(function () {
	var chartId = 0;
	$('#AddChartButton').click(function () {
		// Get the displayable value selected
		var chartEndPoint = $('#AddChartEndPointSelect').val();
		
		// Append new chart row to the chart table
		var chartRow = $("<tr>", {
			id: "chartRow" + chartId
		});
		$('#ChartTable').append(chartRow);
		
		// Append chart to the chart row 
		var epochChartId = "chart" + chartId
		var chartCell = $("<td>", {
			 id: epochChartId,
			 class: "epoch",
			 style: "width:75%; height:300px;"});
		var chartValueText = $("<p>", {class: "centerCellP"}).text("");
		chartCell.append(chartValueText);
		chartRow.append(chartCell)
		
		// Get Caption information from server and display it
		// <input type="button" id="AddChartButton" value="Add" />
		var captionCell = $("<td>");
		var chartValuesText = []
		$.ajax({
			url: location.origin + chartEndPoint + "/caption",
			dataType: 'json',
			type:  'get',
			success:  function (result) {
				captionCell.append($("<p>", {class: "centerCellP"}).text(result.chartTitle))
				// "<p>" + result.chartTitle + "</p>");
				$.each(result.curvesTitle, function (i, curveTitle) {
					captionCell.append("<p class=\"caption_" + i + "\">" + curveTitle + "</p>");
					chartValuesText[i] = $("<span>", { class:"caption_" + i});
					chartValueText.append(chartValuesText);
				});
				
				// Append caption to the chart row
				chartRow.append(captionCell)
				$('#AddChartButtonDone').text(result.chartTitle + " chart added")
			}
		});
		
		// Create the chart
		var chartData = [
			{
			label: "Curve 1",
			values: []
			},
			{
			label: "Curve 2",
			values: []
			},
			{
			label: "Curve 3",
			values: []
			}];
		var chart = chartCell.epoch({ type: 'time.line', data: chartData, axes: ['left', 'bottom', 'right'], ticks: {time:5, left: 10, right: 10}, queueSize: 1, historySize: 1});
		var chartUrl = location.origin + chartEndPoint
		setInterval(function() {
			$.ajax({
				url: chartUrl
			}).then(function(result) {
				chart.push(result);
				$.each(chartValuesText, function (i, valueText) {
					valueText.text(" | " + result[i].y.toFixed(6) + " | ");});
			});
		  }, 1000);
		
		  
		chartId++;
	})
});

// Display calibration status
$(document).ready(function () {
	$('#CalibrateGyro').click(function () {
		$.ajax({
			url: location.origin + "/calibration/gyrometer/calibrate",
			type:  'put',
			data: {},
			success: function (result) {
				$("#GyroCalibrationStatus").text("Gyrometer calibrating...")
				var GyroCalRequestInterval = setInterval(function() {
					$.ajax({
						url: location.origin + "/calibration/gyrometer/status"
					}).then(function(result) {
						if (result.status) {
							$("#GyroCalibrationStatus").text("Gyrometer calibration done")
							clearInterval(GyroCalRequestInterval)
						} else {
							$("#GyroCalibrationStatus").text("Gyrometer calibrating...")
						}
					});
				}, 1000);	
			}
		});
	});
	
	$('#CalibrateAccel').click(function () {
		$.ajax({
			url: location.origin + "/calibration/accelerometer/calibrate",
			type:  'put',
			data: {},
			success: function (result) {
				$("#AccelCalibrationStatus").text("Accelerometer calibrating...")
				var AccelCalRequestInterval = setInterval(function() {
					$.ajax({
						url: location.origin + "/calibration/accelerometer/status"
					}).then(function(result) {
						if (result.status) {
							$("#AccelCalibrationStatus").text("Accelerometer calibration done")
							clearInterval(AccelCalRequestInterval)
						} else {
							$("#AccelCalibrationStatus").text("Accelerometer calibrating...")
						}
					});
				}, 1000);	
			}
		});
	});
});
