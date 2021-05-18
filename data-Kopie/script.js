var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var typeSlider = 'A';
var typeSwitch = 'B';
var typeAnalog = 'C';
var typeGraph  = 'D';
var typePic    = 'E';

window.addEventListener('load', onload);

function onload(event)
  {
    initWebSocket();
/*
    getCurrentValue1();
    getCurrentValue2();
    getCurrentValue3();
*/
  }

function initWebSocket()
  {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    console.log(' gateway', gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
  }

// event handles
  function onOpen(event)
    {
      console.log('Connection opened');
    }

  function onClose(event)
    {
      console.log('Connection closed');
      setTimeout(initWebSocket, 2000);
    }

  function onMessage(event)
    {
      console.log('Message received');
      console.log(event.data);
      var data = event.data;
      //document.getElementById('state').innerHTML = event.data;
    }
// sliders callback
  function updateSliderPWM_a(element)
    {
      var value = document.getElementById("pwmSlider_a").value;
      var sliderValue = typeSlider + 'a' + value;
      document.getElementById("textSliderValue_a").innerHTML = value;
      console.log("PWM_a ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

  function updateSliderPWM_b(element)
    {
      var value = document.getElementById("pwmSlider_b").value;
      var sliderValue = typeSlider + 'b' + value;
      document.getElementById("textSliderValue_b").innerHTML = value;
      console.log("PWM_b ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

  function updateSliderPWM_c(element)
    {
      var value = document.getElementById("pwmSlider_c").value;
      var sliderValue = typeSlider + 'c' + value;
      document.getElementById("textSliderValue_c").innerHTML = value;
      console.log("PWM_c ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

// Templates
  /*
    function getCurrentValue1()
      {
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function()
          {
            if (this.readyState == 4 && this.status == 200)
              {
                document.getElementById("pwmSlider1").value = this.responseText;
                document.getElementById("textSliderValue1").innerHTML = this.responseText;
              }
          };
          //xhr.open("GET", "/currentValue1", true);
        xhr.send();
      }

    function getCurrentValue2()
      {
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function()
          {
            if (this.readyState == 4 && this.status == 200)
              {
                document.getElementById("pwmSlider2").value = this.responseText;
                document.getElementById("textSliderValue2").innerHTML = this.responseText;
              }
          };
        //xhr.open("GET", "/currentValue2", true);
        xhr.send();
      }

    function getCurrentValue3()
      {
        var xhr = new XMLHttpRequest();
        xhr.onreadystatechange = function()
          {
            if (this.readyState == 4 && this.status == 200)
              {
                var val = this.responseText;
                document.getElementById("pwmSlider3").value = val;
                //document.getElementById("textSliderValue3").writeValue(val);
              }
          };
        //xhr.open("GET", "/currentValue3", true);
        xhr.send();
      }

function initButton() {
  document.getElementById('bON').addEventListener('click', toggleON);
  document.getElementById('bOFF').addEventListener('click', toggleOFF);
}

function toggleON(event) {
  websocket.send('bON');
}

function toggleOFF(event) {
  websocket.send('bOFF');
}



.state {
  font-size: 1.2rem;
  color:#1282A2;
}
button {
  border: none;
  color: #FEFCFB;
  padding: 15px 32px;
  text-align: center;
  text-decoration: none;
  font-size: 16px;
  width: 100px;
  border-radius: 4px;
  transition-duration: 0.4s;
}
.button-on {
  background-color:#034078;
}
.button-on:hover {
  background-color: #1282A2;
}
.button-off {
  background-color:#858585;
}
.button-off:hover {
  background-color: #252524;
}

      <div class="card-grid">
        <div class="card">
          <p class="card-title"> switch 1 </p>
          <p>
            <button class="button-on"  id="bON">ON</button>
            <button class="button-off" id="bOFF">OFF</button>
          </p>
          <p class="state">State: <span id="state">%STATE%</span></p>
        </div>
        <div class="card">
          <p class="card-title"> switch 2 </p>
          <p>
            <button class="button-on"  id="bON">ON</button>
            <button class="button-off" id="bOFF">OFF</button>
          </p>
          <p class="state">State: <span id="state">%STATE%</span></p>
        </div>
      </div>

    <div class="content">
      <div class="card-grid">
        <div class="card">
          <p class="card-title"><i class="fas fa-lightbulb"></i> GPIO 2</p>
          <label class="switch">
            <input type="checkbox" onchange="toggleCheckbox(this)" id="2">
            <span class="slider"></span>
          </label>
          <p class="state">State: <span id="2s"></span></p>
        </div>
        <div class="card">
          <p class="card-title"><i class="fas fa-lightbulb"></i> GPIO 4</p>
          <label class="switch">
            <input type="checkbox" onchange="toggleCheckbox(this)" id="4">
            <span class="slider"></span>
          </label>
          <p class="state">State: <span id="4s"></span></p>
        </div>
        <div class="card">
          <p class="card-title"><i class="fas fa-lightbulb"></i> GPIO 14</p>
          <label class="switch">
            <input type="checkbox" onchange="toggleCheckbox(this)" id="14">
            <span class="slider"></span>
          </label>
          <p class="state">State: <span id="14s"></span></p>
        </div>
        <div class="card">
          <p class="card-title"><i class="fas fa-lightbulb"></i> GPIO 12</p>
          <label class="switch">
            <input type="checkbox" onchange="toggleCheckbox(this)" id="12">
            <span class="slider"></span>
          </label>
          <p class="state">State: <span id="12s"></span></p>
        </div>
      </div>
    </div>

*/
