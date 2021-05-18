var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
var typeSlider = 'A';
var typeSwitch = 'B';
var typeAnalog = 'C';
var typeGraph  = 'D';
var typePic    = 'E';
var labelDef   = 'def'
var labelInput = 'inp'


window.addEventListener('load', onload);
//document.addEventListener('DOMContentLoaded', DOMContentLoaded);


function onload(event)
  {
    initWebSocket();
    initSliders();
    updateAnalog_a();
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
  function sendSlider_1()
    {
      var value = updateSlider_1();
      var sliderValue = typeSlider + 'a' + value;
      console.log(" send Slider1 ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

  function sendSlider_2()
    {
      var value = updateSlider_2();
      var sliderValue = typeSlider + 'b' + value;
      console.log(" send Slider2 ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

  function sendSlider_3()
    {
      var value = updateSlider_3();
      var sliderValue = typeSlider + 'c' + value;
      console.log(" send Slider3 ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

  function sendSlider_4()
    {
      var value = updateSlider_4();
      var sliderValue = typeSlider + 'd' + value;
      console.log(" Slider4 ",value," ",sliderValue);
      websocket.send(sliderValue);
    }

// switces callback
  function sendSwitch_a()
    {
      var value = 0; if (document.getElementById("Sw1").checked) { value = 1; }
      var switchValue = typeSwitch + 'a' + value;
      setSwitch_a(value);
      console.log("send Switch1 ",value," ",switchValue);
      websocket.send(switchValue);
    }

  function sendSwitch_b()
    {
      var value = 0; if (document.getElementById("Sw2").checked) { value = 1; }
      var switchValue = typeSwitch + 'b' + value;
      setSwitch_b(value);
      console.log("send Switch2 ",value," ",switchValue);
      websocket.send(switchValue);
    }

  function sendSwitch_c()
    {
      var value = 0; if (document.getElementById("Sw3").checked) { value = 1; }
      var switchValue = typeSwitch + 'c' + value;
      setSwitch_c(value);
      console.log("send Switch3 ",value," ",switchValue);
      websocket.send(switchValue);
    }

  function sendAnalog_a()
    {
      //var value = document.getElementById("Ana1").value;
      var value = updateAnalog_a();
      var anaValue = typeAnalog + 'a' + value;
      console.log("send Analog1 ",value," ",anaValue);
      websocket.send(anaValue);
    }


// -----------------------------------
// sliders functions
  function initSliders()
    {
      labelSlider_1(labelDef);
      labelSlider_2(labelDef);
      labelSlider_3(labelDef);
      labelSlider_4(labelDef);
      updateSlider_1();
      updateSlider_2();
      updateSlider_3();
      updateSlider_4();
    }
  function updateSlider_1()
    {
      var value = document.getElementById("Slider1").value;
      document.getElementById("valSlider1").innerHTML = value;
      console.log("update Slider1 ",value);
      return value;
    }

  function updateSlider_2()
    {
      var value = document.getElementById("Slider2").value;
      document.getElementById("valSlider2").innerHTML = value;
      console.log("update Slider2 ",value);
      return value;
    }

  function updateSlider_3()
    {
      var value = document.getElementById("Slider3").value;
      document.getElementById("valSlider3").innerHTML = value;
      console.log("update Slider3 ",value);
      return value;
    }

  function updateSlider_4()
    {
      var value = document.getElementById("Slider4").value;
      document.getElementById("valSlider4").innerHTML = value;
      console.log("update Slider4 ",value);
      return value;
    }

  function labelSlider_1(mode, name, unit)
    {
      if (mode != labelInput)
        {
          name = 'Slider 1';
          unit = '%';
        }
      document.getElementById("nameSlider1").innerHTML = name;
      document.getElementById("unitSlider1").innerHTML = unit;
      console.log("label Slider1 ", name, unit);
    }

  function labelSlider_2(mode, name, unit)
    {
      if (mode != labelInput)
        {
          name = 'Slider 2';
          unit = '%';
        }
      document.getElementById("nameSlider2").innerHTML = name;
      document.getElementById("unitSlider2").innerHTML = unit;
      console.log("label Slider2 ", name, unit);
    }

  function labelSlider_3(mode, name, unit)
    {
      if (mode != labelInput)
        {
          name = 'Slider 3';
          unit = '%';
        }
      document.getElementById("nameSlider3").innerHTML = name;
      document.getElementById("unitSlider3").innerHTML = unit;
      console.log("label Slider3 ", name, unit);
    }

  function labelSlider_4(mode, name, unit)
    {
      if (mode != labelInput)
        {
          name = 'Slider 4';
          unit = '%';
        }
      document.getElementById("nameSlider4").innerHTML = name;
      document.getElementById("unitSlider4").innerHTML = unit;
      console.log("label Slider4 ", name, unit);
    }

// switches functions
  function setSwitch_a(inval)
    {
      if (inval) { document.getElementById("Sw1i").checked = true;  }
      else       { document.getElementById("Sw1i").checked = false; }
    }

  function setSwitch_b(inval)
    {
      if (inval) { document.getElementById("Sw2i").checked = true;  }
      else       { document.getElementById("Sw2i").checked = false; }
    }

  function setSwitch_c(inval)
    {
      if (inval) { document.getElementById("Sw3i").checked = true;  }
      else       { document.getElementById("Sw3i").checked = false; }
    }

// values functions
  function updateAnalog_a()
    {
      var value = document.getElementById("Ana1").value;
      if (value == 0) { value = 12345; }
      document.getElementById("Ana1").value = value;
      document.getElementById("valAna1").innerHTML = value;
      console.log("update Ana1 ",value);
      return value;
    }

  function updateAnalog_b()
    {
      var value = document.getElementById("Ana2").value;
      if (value == 0) { value = 12345; }
      document.getElementById("Ana2").value = value;
      document.getElementById("valAna2").innerHTML = value;
      console.log("update Ana2 ",value);
      return value;
    }

  function updateAnalog_c()
    {
      var value = document.getElementById("Ana3").value;
      if (value == 0) { value = 12345; }
      document.getElementById("Ana3").value = value;
      document.getElementById("valAna3").innerHTML = value;
      console.log("update Ana3 ",value);
      return value;
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
  function DOMContentLoaded()
    {
      function b(B)
        {
          let C;switch(B.type)
          {
            case'checkbox':C=B.checked?1:0;break;case'range':case'select-one':C=B.value;break;case'button':case'submit':C='1';break;default:return;      }
          const D=`${c}/control?var=${B.id}&val=${C}`;fetch(D).then(E=>
            {console.log(`request to ${D} finished, status: ${E.status}`)
            })
        }
      var c=document.location.origin;
      const e=B=>
              {
                B.classList.add('hidden')
              },
            f=B=>
              {
                B.classList.remove('hidden')
              },
            g=B=>
              {
                B.classList.add('disabled'),
                B.disabled=!0
              },
            h=B=>
              {
                B.classList.remove('disabled'),
                B.disabled=!0
              },
            i=(B,C,D)=>
              {
                D=!(null!=D)||D;
                let E;
                'checkbox'===B.type?(E=B.checked,C=!!C,B.checked=C)
                                   :(E=B.value,B.value=C),D&&E!==C?b(B)
                                   :!D&&('aec'===B.id?C?e(v)
                                   :f(v):'agc'===B.id?C?(f(t),e(s)):(e(t),f(s))
                                   :'awb_gain'===B.id?C?f(x):e(x)
                                   :'face_recognize'===B.id&&(C?h(n):g(n)))
              };
      document.querySelectorAll('.close')
                .forEach(B=>{B.onclick=()=>{e(B.parentNode)}}),
                 fetch(`${c}/status`)
                .then(function(B)
                      {
                        return B.json()
                      }
                     )
                .then(function(B)
                      {
                        document.querySelectorAll('.default-action').forEach(C=>{i(C,B[C.id],!1)})
                      });
      const j=document.getElementById('stream'),
            k=document.getElementById('stream-container'),
            l=document.getElementById('get-still'),
            m=document.getElementById('toggle-stream'),
            n=document.getElementById('face_enroll'),
            o=document.getElementById('close-stream'),
            p=()=>
              {
                window.stop(),m.innerHTML='Start Stream'
              },
            q=()=>
              {
                j.src=`${c+':81'}/stream`,
                f(k),
                m.innerHTML='Stop Stream'
              };
      l.onclick=()=>
        {
          p(),
          j.src=`${c}/capture?_cb=${Date.now()}`,
          f(k)
        },
      o.onclick=()=>
        {
          p(),e(k)
        },
      m.onclick=()=>
        {
          const B='Stop Stream'===m.innerHTML;
          B?p():q()
        },
      n.onclick=()=>
        {
          b(n)
        },
      document.querySelectorAll('.default-action').forEach(B=>
        {
          B.onchange=()=>b(B)
        });
      const r=document.getElementById('agc'),
            s=document.getElementById('agc_gain-group'),
            t=document.getElementById('gainceiling-group');
      r.onchange=()=>
        {
          b(r),r.checked?(f(t),e(s)):(e(t),f(s))
        };
      const u=document.getElementById('aec'),
            v=document.getElementById('aec_value-group');
            u.onchange=()=>
              {
                b(u),u.checked?e(v):f(v)
              };
      const w=document.getElementById('awb_gain'),
            x=document.getElementById('wb_mode-group');
      w.onchange=()=>
        {
          b(w),w.checked?f(x):e(x)
        };
      const y=document.getElementById('face_detect'),
            z=document.getElementById('face_recognize'),
            A=document.getElementById('framesize');
      A.onchange=()=>
        {
          b(A),5<A.value&&(i(y,!1),i(z,!1))
        },
      y.onchange=()=>
        {
          return 5<A.value?(alert('Please select CIF or lower resolution before enabling this feature!'),
                            void i(y,!1)
                           ):
                           void(b(y),!y.checked&&(g(n),i(z,!1)))
        },
      z.onchange=()=>
        {
          return 5<A.value?(alert('Please select CIF or lower resolution before enabling this feature!'),
                            void i(z,!1)
                           ):
                           void(b(z),z.checked?(h(n),i(y,!0)):g(n))
        }
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
