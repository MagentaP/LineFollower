#include "web.hpp"

WebServer server(80);
Chassis* car = nullptr;

String  web_mode = "STOP";
String  web_cmd = "STOP";
float   web_debug_angle = 0.0f;
float   web_debug_speed = 1.0f;
unsigned long web_cmd_time = 0;

const char HTML_PAGE[] = R"HTMLEND(<!DOCTYPE html><html lang="zh"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no"><title>ESP32 Car</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#111;color:#eee;padding:6px;max-width:480px;margin:auto}
h1{text-align:center;font-size:15px;margin:4px 0}
#bar{background:#1a1a2e;border-radius:6px;padding:6px 8px;font-size:10px;line-height:1.5;margin-bottom:6px}
#bar .row{display:flex;justify-content:space-between;font-size:11px;padding:1px 0}
.v{color:#0f0}
#gp{display:flex;height:180px;margin:6px 0;gap:4px}
.hf{flex:1;display:flex;flex-direction:column;gap:4px}
.gb{flex:1;border:none;border-radius:10px;font-size:24px;font-weight:bold;touch-action:manipulation;user-select:none;-webkit-user-select:none}
.bl{background:#1a3a5c;color:#4af}.br{background:#1a3a5c;color:#4af}
.bf{background:#1a4a1a;color:#4f4}.bb{background:#1a4a1a;color:#4f4}
.bl:active,.br:active{background:#08f;color:#fff}
.bf:active,.bb:active{background:#0f0;color:#000}
.mr{display:flex;gap:4px;margin:6px 0}
.mb{flex:1;border:none;border-radius:6px;padding:8px;font-size:13px;font-weight:bold;touch-action:manipulation}
.ma{background:#0a0;color:#fff}.mm{background:#c60;color:#fff}.ms{background:#c00;color:#fff}
.mon{box-shadow:0 0 0 3px #fff}
#tune{display:grid;grid-template-columns:1fr 1fr 1fr;gap:4px;margin:6px 0}
.pb{background:#1a1a2e;border-radius:6px;padding:6px 4px;text-align:center;cursor:ew-resize;user-select:none;-webkit-user-select:none;touch-action:none}
.pb .pn{font-size:9px;color:#888}
.pb .pv{font-size:13px;color:#0f0;font-weight:bold;margin-top:2px}
#raws{display:flex;gap:3px;margin-top:3px;flex-wrap:wrap}
#raws span{font-size:9px;padding:1px 3px;background:#222;border-radius:3px;min-width:36px;text-align:center}
canvas{display:block}
</style></head><body>
<h1>ESP32 Car</h1>
<div id="bar"><div class="row"><span>State</span><span class="v" id="st">-</span><span>Mode</span><span class="v" id="md">-</span></div>
<div class="row"><span>Pos</span><span class="v" id="ps">-</span><span>Conf</span><span class="v" id="cf">-</span><span>w</span><span class="v" id="om">-</span><span>V</span><span class="v" id="lv">-</span></div>
<div class="row"><span>Roll</span><span class="v" id="rl">-</span><span>Pitch</span><span class="v" id="ph">-</span><span>Yaw</span><span class="v" id="yw">-</span><span>GZ</span><span class="v" id="gz">-</span></div>
<div class="row"><span>YawRef</span><span class="v" id="yrv">-</span><span>YawAct</span><span class="v" id="yav">-</span></div>
<div><span id="imustat" style="color:#888;font-size:10px">IMU?</span></div>
<div style="font-size:9px;color:#888"><span>Bias: X</span><span class="v" id="bxv">-</span><span> Y</span><span class="v" id="byv">-</span><span> Z</span><span class="v" id="bzv">-</span></div>
<div id="raws"></div></div>
<div id="gp">
<div class="hf"><button class="gb bl" id="btnL">L</button><button class="gb br" id="btnR">R</button></div>
<div class="hf"><button class="gb bf" id="btnF">F</button><button class="gb bb" id="btnB">B</button></div>
</div>
<div style="background:#1a1a2e;border-radius:6px;padding:4px;margin-bottom:6px;text-align:center">
<canvas id="axes" width="140" height="140" style="width:140px;height:140px"></canvas>
<canvas id="chart" width="480" height="220" style="width:100%;height:220px"></canvas>
</div>
<div class="mr">
<button class="mb ms mon" id="bs" onclick="setM('STOP')">STOP</button>
<button class="mb ma" id="ba" onclick="setM('AUTO')">AUTO</button>
<button class="mb mm" id="bm" onclick="setM('MANUAL')">MANUAL</button>
<button class="mb" id="bd" style="background:#48f;color:#fff" onclick="setM('DEBUG')">DEBUG</button>
</div>
<div style="text-align:center;margin:4px 0"><button style="font-size:11px;padding:4px 12px;background:#c60;color:#fff;border:none;border-radius:4px" onclick="fetch('/cal')">Calib IMU</button></div>
<div id="tune"></div>
<script>
var P={FWD:0,BACK:0,LEFT:0,RIGHT:0};
['btnF','btnL','btnB','btnR'].forEach(function(id){
  var el=document.getElementById(id);
  var c={'btnF':'FWD','btnL':'LEFT','btnB':'BACK','btnR':'RIGHT'}[id];
  el.addEventListener('pointerdown',function(e){e.preventDefault();P[c]=1});
  el.addEventListener('pointerup',function(e){e.preventDefault();P[c]=0});
  el.addEventListener('pointercancel',function(e){P[c]=0});
});
var km={w:'FWD',a:'LEFT',s:'BACK',d:'RIGHT'};
document.addEventListener('keydown',function(e){var c=km[e.key];if(c){e.preventDefault();P[c]=1}});
document.addEventListener('keyup',function(e){var c=km[e.key];if(c){e.preventDefault();P[c]=0}});
setInterval(function(){
  var c='STOP';if(P.FWD)c='FWD';else if(P.BACK)c='BACK';else if(P.LEFT)c='LEFT';else if(P.RIGHT)c='RIGHT';
  fetch('/man?cmd='+c);
},80);
var pm=[
  ['Speed','speed',0.2,0.02,0.05,10,'1.0'],
  ['Max w','maxw',1,0.1,0.5,40,'6.0'],
  ['Max%','maxduty',10,2,10,100,'80'],
  ['Bias','bias',1,0.1,0,50,'20.0'],
  ['T-Slo','turnslow',0.05,0.01,0,1,'0.6'],
  ['Gap','gap',100,20,0,10000,'500'],
  ['Lost','ltime',200,50,0,10000,'1000'],
  ['Conf','conf',0.1,0.01,0.01,1,'0.15'],
  ['Pos-KP','poskp',0.5,0.05,0,20,'4.0'],
  ['Pos-KI','poski',0.2,0.02,0,10,'2.0'],
  ['Pos-KD','poskd',0.1,0.02,0,5,'0.0'],
  ['Pos-IL','posilim',0.5,0.1,0.5,20,'4.0'],
  ['YA-KP','yakp',1,0.1,0,50,'10.0'],
  ['YA-KI','yaki',0.5,0.05,0,30,'1.0'],
  ['YA-KD','yakd',0.5,0.05,0,30,'0.8'],
  ['YA-IL','yailim',1,0.1,0.5,50,'10.0'],
  ['D-Ang','dang',5,0.5,-180,180,'0'],
  ['D-Spd','dspd',0.1,0.01,0.05,5,'1.0'],
];
var tc=document.getElementById('tune'), decs={};
pm.forEach(function(p){
  var d=document.createElement('div');d.className='pb';
  d.innerHTML='<div class="pn">'+p[0]+'</div><div class="pv">'+p[6]+'</div>';
  d.dataset.k=p[1];d.dataset.c=p[2];d.dataset.f=p[3];d.dataset.mn=p[4];d.dataset.mx=p[5];
  decs[p[1]]=p[3].toString().split('.')[1]?p[3].toString().split('.')[1].length:0;
  tc.appendChild(d);
});
tc.addEventListener('pointerdown',function(e){
  var el=e.target.closest('.pb');if(!el)return;
  e.preventDefault();el.setPointerCapture(e.pointerId);
  el.style.background='#336';
  var sx=e.clientX,sy=e.clientY,sv=parseFloat(el.querySelector('.pv').textContent);
  var c=parseFloat(el.dataset.c),f=parseFloat(el.dataset.f);
  var mn=parseFloat(el.dataset.mn),mx=parseFloat(el.dataset.mx),dec=decs[el.dataset.k];
  function mv(e){
    var v=sv+(e.clientX-sx)*c-(e.clientY-sy)*f;
    if(v<mn)v=mn;if(v>mx)v=mx;
    el.querySelector('.pv').textContent=v.toFixed(dec);
  }
  function up(e){
    el.releasePointerCapture(e.pointerId);
    el.style.background='#1a1a2e';
    var v=parseFloat(el.querySelector('.pv').textContent);
    fetch('/set?'+el.dataset.k+'='+v);
    el.removeEventListener('pointermove',mv);el.removeEventListener('pointerup',up);
  }
  el.addEventListener('pointermove',mv);el.addEventListener('pointerup',up);
});
function setM(m){fetch('/mode?m='+m);
  document.getElementById('bs').className='mb ms'+(m=='STOP'?' mon':'');
  document.getElementById('ba').className='mb ma'+(m=='AUTO'?' mon':'');
  document.getElementById('bm').className='mb mm'+(m=='MANUAL'?' mon':'');
  document.getElementById('bd').className='mb'+(m=='DEBUG'?' mon':'');
  document.getElementById('bd').style.background='#48f';document.getElementById('bd').style.color='#fff';
}
var cvA=document.getElementById('axes'),ctxA=cvA.getContext('2d');
function q2m(q){var w=q[0],x=q[1],y=q[2],z=q[3];
  return[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
         2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
         2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];}
function proj(v,m){var x=v[0]*m[0]+v[1]*m[1]+v[2]*m[2],
  y=v[0]*m[3]+v[1]*m[4]+v[2]*m[5],z=v[0]*m[6]+v[1]*m[7]+v[2]*m[8];
  var s=1/(1+z*0.002);return[cvA.width/2+x*s*60,cvA.height/2-y*s*60];}
function drawAxes(q){
  var m=q2m(q);ctxA.fillStyle='#1a1a2e';ctxA.fillRect(0,0,cvA.width,cvA.height);
  var cx=cvA.width/2,cy=cvA.height/2;
  ctxA.strokeStyle='#333';ctxA.beginPath();ctxA.moveTo(cx,0);ctxA.lineTo(cx,cvA.height);ctxA.stroke();
  ctxA.beginPath();ctxA.moveTo(0,cy);ctxA.lineTo(cvA.width,cy);ctxA.stroke();
  var o=proj([0,0,0],m),cols=['#f44','#4f4','#48f'],lbls=['X','Y','Z'];
  for(var i=0;i<3;i++){var v=[0,0,0];v[i]=2.5;var p=proj(v,m);
    ctxA.strokeStyle=cols[i];ctxA.lineWidth=2;
    ctxA.beginPath();ctxA.moveTo(o[0],o[1]);ctxA.lineTo(p[0],p[1]);ctxA.stroke();
    ctxA.fillStyle=cols[i];ctxA.font='10px monospace';ctxA.fillText(lbls[i],p[0]+3,p[1]-3);}
}
var chCv=document.getElementById('chart'),chCtx=chCv.getContext('2d');
var chData=[],CHM=100;
function drawChart(){
  var W=chCv.width,H=chCv.height,n=chData.length;
  chCtx.fillStyle='#1a1a2e';chCtx.fillRect(0,0,W,H);
  if(n<2)return;
  chCtx.strokeStyle='#444';chCtx.beginPath();chCtx.moveTo(0,H/2);chCtx.lineTo(W,H/2);chCtx.stroke();
  function sy(v){return H-((v+185)/370)*H;}
  function sx(i){return(i/(CHM-1))*W;}
  chCtx.strokeStyle='rgba(255,200,50,0.7)';chCtx.setLineDash([4,2]);chCtx.lineWidth=2;
  chCtx.beginPath();
  for(var i=0;i<n;i++){var x=sx(i),y=sy(chData[i].yr||0);i==0?chCtx.moveTo(x,y):chCtx.lineTo(x,y);}
  chCtx.stroke();
  chCtx.strokeStyle='#0f0';chCtx.setLineDash([]);chCtx.lineWidth=2.5;
  chCtx.beginPath();
  for(var i=0;i<n;i++){var x=sx(i),y=sy(chData[i].ya||0);i==0?chCtx.moveTo(x,y):chCtx.lineTo(x,y);}
  chCtx.stroke();chCtx.setLineDash([]);
  chCtx.fillStyle='#fa0';chCtx.font='11px monospace';chCtx.fillText('ref',4,14);
  chCtx.fillStyle='#0f0';chCtx.fillText('act',4,28);
  chCtx.fillStyle='#888';chCtx.font='10px monospace';chCtx.fillText('+-180d',W-45,14);
}

function poll(){
  fetch('/status').then(function(r){return r.json()}).then(function(d){
    document.getElementById('st').textContent=d.s;document.getElementById('md').textContent=d.m;
    document.getElementById('ps').textContent=d.p.toFixed(2);document.getElementById('cf').textContent=d.c.toFixed(2);
    document.getElementById('om').textContent=d.w.toFixed(3);document.getElementById('lv').textContent=d.v.toFixed(3);
    document.getElementById('rl').textContent=d.roll.toFixed(1)+'d';
    document.getElementById('ph').textContent=d.pitch.toFixed(1)+'d';
    document.getElementById('yw').textContent=d.yaw.toFixed(1)+'d';
    document.getElementById('gz').textContent=d.gz.toFixed(3);
    document.getElementById('yrv').textContent=(d.yr||0).toFixed(1)+'d';
    document.getElementById('yav').textContent=(d.yaw||0).toFixed(1)+'d';
    var im=document.getElementById('imustat');
    im.textContent=d.imu?'IMU OK':(d.fall?'FALL':'NO IMU');
    im.style.color=d.imu?'#0f0':(d.fall?'#f00':'#888');
    document.getElementById('bxv').textContent=d.bx.toFixed(6);
    document.getElementById('byv').textContent=d.by.toFixed(6);
    document.getElementById('bzv').textContent=d.bz.toFixed(6);
    if(d.imu&&d.q0)drawAxes([d.q0,d.q1,d.q2,d.q3]);
    chData.push({yr:d.yr||0,ya:d.yaw||0});while(chData.length>CHM)chData.shift();drawChart();
    if(d.r){var h='';for(var i=0;i<d.r.length;i++)h+='<span>S'+i+':'+d.r[i]+'</span>';document.getElementById('raws').innerHTML=h}
  });
}
setInterval(poll,200);poll();
</script></body></html>)HTMLEND";

void httpHandleRoot()
{
    server.send(200, "text/html; charset=utf-8", HTML_PAGE);
}

void httpHandleStatus()
{
    if (!car)
    {
        server.send(200, "application/json", "{}");
        return;
    }
    const ImuState& s = car->imuState();
    char buf[640];
    snprintf(buf, sizeof(buf),
        "{\"s\":\"%s\",\"m\":\"%s\",\"p\":%.3f,\"c\":%.3f,\"w\":%.4f,\"v\":%.4f,"
        "\"r\":[%d,%d,%d,%d],"
        "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"gz\":%.3f,\"yr\":%.3f,"
        "\"q0\":%.4f,\"q1\":%.4f,\"q2\":%.4f,\"q3\":%.4f,"
        "\"imu\":%d,\"fall\":%d,"
        "\"bx\":%.6f,\"by\":%.6f,\"bz\":%.6f}",
        car->state_str_.c_str(), web_mode.c_str(),
        car->pos_, car->conf_, car->omega_, car->vel_,
        car->raw_buf_[0], car->raw_buf_[1], car->raw_buf_[2], car->raw_buf_[3],
        s.roll, s.pitch, s.yaw, s.gyro_z, car->yaw_ref_,
        s.q0, s.q1, s.q2, s.q3,
        s.ok ? 1 : 0, car->fallen_ ? 1 : 0,
        s.bias_x, s.bias_y, s.bias_z);
    server.send(200, "application/json", buf);
}

void httpHandleSet()
{
    for (int i = 0; i < server.args(); i++)
    {
        String k = server.argName(i);
        float v = server.arg(i).toFloat();
        if (k == "speed")
        {
            target_speed = v;
        }
        else if (k == "maxw")
        {
            max_angular_velocity = v;
        }
        else if (k == "maxduty")
        {
            max_duty = v;
        }
        else if (k == "bias")
        {
            duty_bias = v;
        }
        else if (k == "turnslow")
        {
            turn_slow = v;
        }
        else if (k == "gap")
        {
            gap_grace_ms = (int)v;
        }
        else if (k == "ltime")
        {
            lost_recovery_timeout_ms = (int)v;
        }
        else if (k == "conf")
        {
            line_confidence_threshold = v;
        }
        else if (k == "yakp")
        {
            yaw_kp = v;
        }
        else if (k == "yaki")
        {
            yaw_ki = v;
        }
        else if (k == "yakd")
        {
            yaw_kd = v;
        }
        else if (k == "yailim")
        {
            yaw_i_limit = v;
        }
        else if (k == "poskp")
        {
            pos_kp = v;
        }
        else if (k == "poski")
        {
            pos_ki = v;
        }
        else if (k == "poskd")
        {
            pos_kd = v;
        }
        else if (k == "posilim")
        {
            pos_i_limit = v;
        }
        else if (k == "dang")
        {
            web_debug_angle = v;
        }
        else if (k == "dspd")
        {
            web_debug_speed = v;
        }
    }
    server.send(200, "text/plain", "ok");
}

void httpHandleMode()
{
    for (int i = 0; i < server.args(); i++)
    {
        if (server.argName(i) == "m")
        {
            web_mode = server.arg(i);
            if (web_mode != "MANUAL")
            {
                web_cmd = "STOP";
            }
        }
    }
    server.send(200, "text/plain", "ok");
}

void httpHandleManual()
{
    for (int i = 0; i < server.args(); i++)
    {
        if (server.argName(i) == "cmd")
        {
            web_cmd = server.arg(i);
            web_cmd_time = millis();
        }
    }
    server.send(200, "text/plain", "ok");
}

void httpHandleCalibrate()
{
    if (car)
    {
        car->calibrateImu();
    }
    server.send(200, "text/plain", "ok");
}
