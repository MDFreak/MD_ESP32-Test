var gateway = `ws://${window.location.hostname}/ws`;
var websocket;

window.addEventListener('load', onload);

function onload(event)
  {
    initWebSocket();
    getCurrentValue1();
    getCurrentValue2();
    getCurrentValue3();
  }

function initWebSocket()
  {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage;
  }

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
    console.log(event.data);
  }

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
    xhr.open("GET", "/currentValue1", true);
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
    xhr.open("GET", "/currentValue2", true);
    xhr.send();
  }

function getCurrentValue3()
  {
    var xhr = new XMLHttpRequest();
    xhr.onreadystatechange = function()
      {
        if (this.readyState == 4 && this.status == 200)
          {
            document.getElementById("pwmSlider3").value = this.responseText;
            document.getElementById("textSliderValue3").innerHTML = this.responseText;
          }
      };
    xhr.open("GET", "/currentValue3", true);
    xhr.send();
  }

function updateSliderPWM1(element)
  {
    var sliderValue = '1 ' + document.getElementById("pwmSlider1").value;
    document.getElementById("textSliderValue1").innerHTML = sliderValue;
    console.log(sliderValue);
    websocket.send(sliderValue);
  }

function updateSliderPWM2(element)
  {
    var sliderValue = '2 ' + document.getElementById("pwmSlider2").value;
    document.getElementById("textSliderValue2").innerHTML = sliderValue;
    console.log(sliderValue);
    websocket.send(sliderValue);
  }

function updateSliderPWM3(element)
  {
    var sliderValue = '3 ' + document.getElementById("pwmSlider3").value;
    document.getElementById("textSliderValue3").innerHTML = sliderValue;
    console.log(sliderValue);
    websocket.send(sliderValue);
  }
