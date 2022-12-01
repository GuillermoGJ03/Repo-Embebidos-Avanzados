window.addEventListener("load", onload);

function onload(event) {
  chartT = createTemperatureChart();
  chartF = createFftChart();
}

// Create Temperature Chart
function createTemperatureChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: "chart-temperature",
      type: "spline",
      backgroundColor: "#1c1c1e",
    },
    series: [
      {
        name: "Pressure",
        color: "#50cd89",
        dashStyle: "longdash",
      },
      {
        name: "Reference",
        color: "#FF0000",
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: "datetime",
      dateTimeLabelFormats: { second: "%H:%M:%S" },
    },
    yAxis: {
      title: {
        text: "PSI",
      },
      min: 11,
      max: 13,
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}

// Create Humidity Chart
function createFftChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: "chart-fft",
      type: "spline",
      backgroundColor: "#1c1c1e",
    },
    series: [
      {
        name: "FFT",
        color: "#50cd",
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: "datetime",
      dateTimeLabelFormats: { second: "%H:%M:%S" },
    },
    yAxis: {
      title: {
        text: "PSI",
      },
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}
