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

  */
