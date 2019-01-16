// ----------------------------------------------------------------------------- Shortcuts
var $ = function(id) { return document.getElementById(id); };
var C = function(tag) { return document.createElement(tag); };
// ----------------------------------------------------------------------------- Settings
var currSet;
var currCat;
var currOpt;
var rStatus = {
  "FC":"",
  "Name":"",
  "Arm state":"",
  "GPS":"",
  "Battery":""
}
var rSettings = [{
  "cat": "General",
    "sub": [{
      "name": "UAV timeout",
      "cmd": "config uavtimeout ",
      "clitext": "UAV timeout",
      "value": "",
      "options": [
        { "name": "10 seconds", "value": 10 },
        { "name": "5 minutes", "value": 300 },
        { "name": "1 hour", "value": 3600 }]
    }]
  },{
  "cat": "Lora",
  "sub": [{
    "name": "Frequency",
    "cmd": "config loraFreq ",
    "clitext": "Lora frequency",
    "value": "",
    "options": [
      { "name": "433 MHz", "value": 433000000 },
      { "name": "868 MHz", "value": 868000000 },
      { "name": "915 MHz", "value": 915000000 }]
    }, {
    "name": "Bandwidth",
    "cmd": "config loraBandwidth ",
    "clitext": "Lora bandwidth",
    "value": "",
    "options": [
      { "name": "250 kHz", "value": 250000 },
      { "name": "62.8 kHz", "value": 62800 },
      { "name": "7.8 kHz", "value": 7800 }]
    }, {
    "name": "Spreading factor",
    "cmd": "config loraSpread ",
    "clitext": "Lora spreading factor",
    "value": "",
    "options": [
      { "name": "7", "value": 7 },
      { "name": "8", "value": 8 },
      { "name": "9", "value": 9 },
      { "name": "10", "value": 10 },
      { "name": "11", "value": 11 },
      { "name": "12", "value": 12 }]
  }]
},{
  "cat": "Debugging",
  "sub": [{
    "name": "Debug output",
    "cmd": "debug",
    "clitext": "Debug output",
    "value": "",
    "options": [{ "name": "onoff" }]
  }, {
    "name": "Local fake UAVs",
    "cmd": "localfakeplanes",
    "clitext": "Local fake planes",
    "value": "",
    "options": [{ "name": "onoff" }]
  },{
    "name": "Radio fake UAVs",
    "cmd": "radiofakeplanes",
    "clitext": "Radio fake planes",
    "value": "",
    "options": [{ "name": "onoff" }]
  },{
    "name": "Move fake UAVs",
    "cmd": "movefakeplanes",
    "clitext": "Move fake planes",
    "value": "",
    "options": [{ "name": "onoff" }]
  }]
},{
  "cat": "App",
  "sub": [{
    "name": "About this app",
    "value": "Version 0.1",
    "options": [
      { "name": "Developed by" },
      { "name": "Daniel Heymann" },
      { "name": "dh@iumt.de" }]
  },{
    "name": "About INAV-Radar",
    "value": "Version 0.1",
    "options": [
      { "name": "Developed by" },
      { "name": "Camille Maria and Daniel Heymann" },
      { "name": "dh@iumt.de" }]
  }]
}];
// ----------------------------------------------------------------------------- Phonon UI
phonon.options({
  navigator: {
    defaultPage: 'home',
    animatePages: true
  },
  i18n: null
});

var navi = phonon.navigator();
navi.on({
  page: 'home',
  preventClose: true,
  content: null
});

navi.on({
  page: 'suboptions',
  preventClose: false,
  content: null
}, function(activity) {
  activity.onReady(function() {
    $("subtitle").textContent = currSet.name;
    var list = C('ul');
    list.className = 'list';
    currSet.options.forEach(function(option,i) {
      var opli = C("li");
      var a = C("a");
      a.className = "padded-list";
      a.textContent = option.name;
      opli.on('click', function () {
        var o = option;
        var out = currSet.cmd + "\n";
        if (rSettings[currCat].sub[currOpt].cmd.slice(-1) == ' ') out = rSettings[currCat].sub[currOpt].cmd + o.value + "\n";
        ws.send(out);
        rSettings[currCat].sub[currOpt].value = o.value;
        navi.changePage('radiosettings');
      })
      opli.appendChild(a);
      list.appendChild(opli);
    })
    $('subcontent').innerHTML = ''
    $('subcontent').appendChild(list);
  });
});

navi.on({
  page: 'radiosettings',
  preventClose: false,
  content: null
}, function(activity) {
  activity.onReady(function() {
    $('slist').innerHTML = '';
    rSettings.forEach(function (cat,ci) {
      var li = C("li");
      li.className = "divider";
      li.textContent = cat.cat;
      slist.appendChild(li);
      cat.sub.forEach(function (sub,i) {
        if (sub.options[0].name =='onoff') {
          var input = C("input");
          input.type = "checkbox";
          if (sub.value == 1) input.checked = true;
          else input.checked = false;
          var span = C("span");
          span.className = "text";
          span.textContent = sub.name;
          var span2 = C("span");
          span2.style.marginRight = "16px";
          var opli = C("li");
          opli.on('click', function (ev)  {
            var cset = sub;
            var ccat = ci;
            var copt = i;
            var inp = input;
            currSet = cset;
            currCat = ccat;
            currOpt = copt;
            var out = currSet.cmd + "\n";
            ws.send(out);
            rSettings[currCat].sub[currOpt].value = !inp.checked;
            inp.checked = !inp.checked
          });
          opli.className = "checkbox";
          opli.style.paddingLeft = "16px";
          opli.appendChild(input);
          opli.appendChild(span2);
          opli.appendChild(span);
          $('slist').appendChild(opli);
        } else {
          var span = C("span");
          span.className = "pull-right";
          span.style.width = "100px";
          span.textContent = sub.value;
          var a = C("a");
          a.className = "padded-list";
          a.textContent = sub.name;
          a.on('click', function (ev)  {
            var cset = sub;
            var ccat = ci;
            var copt = i;
            currSet = cset;
            currCat = ccat;
            currOpt = copt;
            navi.changePage('suboptions');

          });
          var opli = C("li");
          opli.appendChild(span);
          opli.appendChild(a);
          $('slist').appendChild(opli);
        }
      });
    });
  });
  activity.onClose(function(self) {

  });
});
var setStatus = function (items) {
  $('connectedto').textContent = 'Connected to ' + items[1] + '(' + items[0] + ')';
  if (items[3] == 1) $('armed').textContent = 'Armed';
  else $('armed').textContent = 'Disarmed';
  $('gpsstatus').textContent = items[4] + ' Sats';
  $('battery').textContent = items[2] + ' V';
}
var setConfig = function (items) {
  rSettings.forEach(function (cat,ci) {
    cat.sub.forEach(function (sub,si) {
      cfgs = items.split('<');
      cfgs.forEach(function (cfg,i) {
        keyval = cfg.split('>');
        if (keyval[0] == rSettings[ci].sub[si].clitext) rSettings[ci].sub[si].value = keyval[1];
      })
    })
  })
}
// ----------------------------------------------------------------------------- ws com
var ws = null;
function sendBlob(str){
  var buf = new Uint8Array(str.length);
  for (var i = 0; i < str.length; ++i) buf[i] = str.charCodeAt(i);
  ws.send(buf);
}
function addMessage(m){
  console.log(m);
}
function startSocket(){
  ws = new WebSocket('ws://'+document.location.host+'/ws',['arduino']);
  ws.binaryType = "arraybuffer";
  ws.onopen = function(e){
    addMessage("Connected");
  };
  ws.onclose = function(e){
    addMessage("Disconnected");
  };
  ws.onerror = function(e){
    console.log("ws error", e);
    addMessage("Error");
  };
  ws.onmessage = function(e){
    addMessage(e.data);
  };
  //ws.send(ge("input_el").value);

}
function startEvents(){
  var es = new EventSource('/events');
  es.onopen = function(e) {
    addMessage("Events Opened");
  };
  es.onerror = function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      addMessage("Events Closed");
    }
  };
  es.onmessage = function(e) {
    addMessage("Event: " + e.data);
    msg = e.data.split(": ");
    if (msg[0] == "Status") setStatus(msg[1].split(", "));
    if (msg[0] == "Config") setConfig(msg[1]);
  };
  es.addEventListener('ota', function(e) {
    addMessage("Event[ota]: " + e.data);
  }, false);
}


// ----------------------------------------------------------------------------- starting point
(function() {
  navi.start()
  startSocket();
  startEvents();

})();
