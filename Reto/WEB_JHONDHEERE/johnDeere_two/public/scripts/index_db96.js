const loginElement = document.querySelector("#login-form");
const contentElement = document.querySelector("#content-sign-in");
const userDetailsElement = document.querySelector("#user-details");
const authBarElement = document.querySelector("#authentication-bar");

// Elements for sensor readings
const tempElement = document.getElementById("temp");
const humElement = document.getElementById("hum");
const presElement = document.getElementById("pres");
const ledElement = document.getElementById("led");

var valueMax = document.getElementById("kt_slider_basic_porc");

var dbPathLed;
let num;
var num2 = 12;

// MANAGE LOGIN/LOGOUT UI
const setupUI = (user) => {
  if (user) {
    //toggle UI elements
    loginElement.style.display = "none";
    contentElement.style.display = "block";
    authBarElement.style.display = "block";
    userDetailsElement.style.display = "block";
    userDetailsElement.innerHTML = user.email;

    // get user UID to get data from database
    var uid = user.uid;
    console.log(uid);
    console.log("Hola");

    // Database paths (with user UID)
    var dbPathTemp = "UsersData/" + uid.toString() + "/temperature";
    var dbPathHum = "UsersData/" + uid.toString() + "/humidity";
    var dbPathPres = "UsersData/" + uid.toString() + "/pressure";
    dbPathLed = "UsersData/" + uid.toString() + "/led";

    // Database references
    var dbRefTemp = firebase.database().ref().child(dbPathTemp);
    var dbRefHum = firebase.database().ref().child(dbPathHum);
    var dbRefPres = firebase.database().ref().child(dbPathPres);
    //var dbPathLed = firebase.database().ref().child(dbPathLed);

    // Update page with new readings
    dbRefPres.on("value", (snap) => {
      presElement.innerText = snap.val().toFixed(3);
      var x = new Date().getTime(),
        y = parseFloat(snap.val().toFixed(3)),
        y2 = parseFloat(num2);

      // y = parseFloat(this.responseText);
      //console.log(this.responseText);
      if (chartT.series[0].data.length > 40) {
        chartT.series[0].addPoint([x, y], true, true, true);
        chartT.series[1].addPoint([x, y2], true, true, true);
      } else {
        chartT.series[0].addPoint([x, y], true, false, true);
        chartT.series[1].addPoint([x, y2], true, false, true);
      }
    });

    dbRefHum.on("value", (snap) => {
      humElement.innerText = snap.val().toFixed(1);
    });

    dbRefTemp.on("value", (snap) => {
      tempElement.innerText = snap.val().toFixed(3);
    });

    dbRefPres.on("value", (snap) => {
      presElement.innerText = snap.val().toFixed(3);
    });

    // if user is logged out
  } else {
    // toggle UI elements
    loginElement.style.display = "block";
    authBarElement.style.display = "none";
    userDetailsElement.style.display = "none";
    contentElement.style.display = "none";
  }
};

document.addEventListener("DOMContentLoaded", () => {
  // ======== Slider with one handle

  const priceSlider = document.getElementById("r-slider");

  noUiSlider.create(priceSlider, {
    start: 12,
    connect: true,
    tooltips: true,
    range: {
      min: 11,
      max: 13,
    },
    tooltips: true,
  });

  priceSlider.noUiSlider.on("change", function (values, handle) {
    if (handle) {
      console.log(values[handle]);
      console.log("led ON");
    } else {
      console.log("Pot_to_control: ");
      console.log(values[handle]);
      //((input - min) * 100) / (max - min)

      valueMax.innerHTML = (((values[handle] - 11) * 100) / (13 - 11)).toFixed(
        0
      );
      num = values[handle];
      num2 = num;
      firebase.database().ref(dbPathLed).set(num);
    }
  });
});
