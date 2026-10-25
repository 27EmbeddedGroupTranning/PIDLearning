"use strict";
const $ = id => document.getElementById(id);
const colors = {cyan: "#55dfcb", gold: "#ffc76a", purple: "#b5a2ff", white: "#e6edf3", grid: "#273747", muted: "#92a7b8"};
let rows = [], previous = [], time = 0, playing = true, last = performance.now();
function canvas(id) {
  const c = $(id), w = c.clientWidth, h = c.clientHeight, ratio = window.devicePixelRatio || 1;
  if (c.width !== Math.round(w * ratio) || c.height !== Math.round(h * ratio)) {
    c.width = Math.round(w * ratio); c.height = Math.round(h * ratio);
  }
  const ctx = c.getContext("2d"); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, w, h);
  ctx.font = '11px system-ui'; return {ctx, w, h};
}
function line(ctx, x1, y1, x2, y2, color, width = 1) {
  ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
}
function circle(ctx, x, y, r, fill, stroke) {
  ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fillStyle = fill; ctx.fill();
  if (stroke) {ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.stroke();}
}
function drawGimbal(r) {
  const {ctx, w, h} = canvas("gimbal");
  // 正交投影：世界坐标 x 为前向，y 为左向，z 为向上。
  const scale = Math.min(w / 5.3, 80), cx = w * 0.5, cy = h * 0.38;
  const project = (x, y, z) => [cx + scale * (x * .82 - y * .82), cy + scale * (x * .35 + y * .35 - z)];
  const segment = (a,b,color,width=1) => line(ctx,...project(...a),...project(...b),color,width);
  for (let n=-2;n<=2;n++) {segment([n,-2,-.6],[n,2,-.6],colors.grid);segment([-2,n,-.6],[2,n,-.6],colors.grid);}
  segment([0,0,-.6],[0,0,.3],"#4d6477",18);
  const origin = project(0,0,.3); circle(ctx,...origin,25,"#1e3343","#61788c");
  function barrel(yaw,pitch,color,dashed) {
    const a=yaw*Math.PI/180,b=pitch*Math.PI/180;
    const end=project(1.65*Math.cos(b)*Math.cos(a),1.65*Math.cos(b)*Math.sin(a),.3+1.65*Math.sin(b));
    ctx.setLineDash(dashed?[6,5]:[]);line(ctx,...origin,...end,color,dashed?3:12);ctx.setLineDash([]);
    if(!dashed) circle(ctx,...end,7,"#0b1219",color);
  }
  barrel(r.yaw_target,r.pitch_target,colors.gold,true);barrel(r.yaw,r.pitch,colors.cyan,false);
  ctx.fillStyle=colors.muted;ctx.fillText("双轴云台 · 等轴测投影",12,22);
  const smallR = Math.min(40, w/9);
  function dial(x,y,value,target,label) {
    circle(ctx,x,y,smallR,"#0b1721",colors.grid);
    const pointer=(angle,color,dash)=>{const a=angle*Math.PI/180;ctx.setLineDash(dash?[4,3]:[]);line(ctx,x,y,x+smallR*.85*Math.cos(a),y-smallR*.85*Math.sin(a),color,2);ctx.setLineDash([]);};
    pointer(target,colors.gold,true);pointer(value,colors.cyan,false);
    ctx.fillStyle=colors.muted;ctx.textAlign="center";ctx.fillText(label+"  "+value.toFixed(1)+"°",x,y+smallR+20);ctx.textAlign="left";
  }
  dial(w*.27,h-77,r.yaw,r.yaw_target,"Yaw / 俯视");dial(w*.73,h-77,r.pitch,r.pitch_target,"Pitch / 侧视");
  if(r.load) {ctx.fillStyle=colors.gold;ctx.textAlign="right";ctx.fillText("外力矩 "+r.load.toFixed(2)+" N·m / 轴",w-8,22);ctx.textAlign="left";}
}
function plot(id, series, unit) {
  const {ctx,w,h}=canvas(id), left=50,right=w-12,top=16,bottom=h-28;
  const values=series.flatMap(s=>s.data.map(r=>r[s.key]));
  let min=Math.min(0,...values),max=Math.max(0,...values),pad=Math.max((max-min)*.12,.05);min-=pad;max+=pad;
  const x=t=>left+t/15*(right-left), y=v=>bottom-(v-min)/(max-min)*(bottom-top);
  ctx.fillStyle=colors.muted;ctx.fillText(unit,3,11);
  for(let n=0;n<=4;n++) {const v=min+(max-min)*n/4;line(ctx,left,y(v),right,y(v),colors.grid);ctx.fillStyle=colors.muted;ctx.fillText(v.toFixed(max-min>10?0:2),2,y(v)+4);}
  for(let t=0;t<=15;t+=3) {line(ctx,x(t),top,x(t),bottom,colors.grid);ctx.fillStyle=colors.muted;ctx.fillText(t+"s",x(t)-7,h-7);}
  for(const s of series) {ctx.beginPath();ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1.6;ctx.setLineDash(s.dash?[5,4]:[]);s.data.forEach((r,i)=>i?ctx.lineTo(x(r.time),y(r[s.key])):ctx.moveTo(x(r.time),y(r[s.key])));ctx.stroke();}
  ctx.setLineDash([]);line(ctx,x(time),top,x(time),bottom,"#ffffff66");
}
function metrics() {
  const axis=$("axis").value, target=rows.find(r=>r.time>=1)[axis+"_target"];
  const step=rows.filter(r=>r.time>=1&&r.time<6), sign=Math.sign(target);
  const overshoot=target===0?"—":(Math.max(0,...step.map(r=>(r[axis]-target)*sign))/Math.abs(target)*100).toFixed(1)+"%";
  const tail=rows.filter(r=>r.time>=14), mae=tail.reduce((sum,r)=>sum+Math.abs(r[axis+"_target"]-r[axis]),0)/tail.length;
  $("metrics").textContent=`初次阶跃超调（1–6 s）：${overshoot}　末秒平均绝对误差：${mae.toFixed(2)}°`;
}
function draw() {
  if(!rows.length)return;
  const r=rows[Math.min(rows.length-1,Math.round(time/.02))],axis=$("axis").value;
  drawGimbal(r);
  const series=[];if(previous.length)series.push({data:previous,key:axis,color:"#7d899c",dash:true});
  series.push({data:rows,key:axis+"_target",color:colors.gold,dash:true},{data:rows,key:axis,color:colors.cyan});
  plot("response",series,"角度 / °");
  plot("terms",[{data:rows,key:axis+"_p",color:colors.cyan},{data:rows,key:axis+"_i",color:colors.gold},{data:rows,key:axis+"_d",color:colors.purple},{data:rows,key:axis+"_u",color:colors.white,width:2}],"力矩 / N·m");
  $("yawError").textContent=(r.yaw_target-r.yaw).toFixed(2)+"°";
  $("pitchError").textContent=(r.pitch_target-r.pitch).toFixed(2)+"°";
  $("torque").textContent=r.yaw_u.toFixed(3)+" N·m";
  $("clock").textContent=time.toFixed(2)+" s";$("scrub").value=time;
  $("play").textContent=playing?"暂停":"播放";
}
async function run() {
  for(const id of ["yaw","pitch","load"])if(!$(id).reportValidity() || $(id).value==="")return;
  $("run").disabled=true;playing=false;$("status").className="";$("status").textContent="正在编译 C++ 并计算 15 秒响应…";
  try {
    const response=await fetch("/api/run",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({yaw:Number($("yaw").value),pitch:Number($("pitch").value),load:Number($("load").value),reverse:$("reverse").checked})});
    const data=await response.json();if(!response.ok)throw new Error(data.error||"仿真失败");
    previous=rows;rows=data.rows;time=0;playing=true;metrics();
    $("status").textContent="编译成功 · 已读取最新 C++ 参数 · 15 s 仿真 / 500 Hz 控制 / 50 Hz 记录";
  } catch(error) {$("status").className="error";$("status").textContent="编译 / 运行失败，本次结果未更新：\n"+error.message;}
  finally {$("run").disabled=false;draw();}
}
$("run").onclick=run;
$("play").onclick=()=>{if(time>=15)time=0;playing=!playing;draw();};
$("scrub").oninput=()=>{time=Number($("scrub").value);playing=false;draw();};
$("axis").onchange=()=>{if(rows.length)metrics();draw();};
window.addEventListener("resize",draw);
function frame(now){const delta=Math.min((now-last)/1000,.1);last=now;if(playing&&rows.length){time=Math.min(15,time+delta*Number($("speed").value));if(time>=15)playing=false;draw();}requestAnimationFrame(frame);}
requestAnimationFrame(frame);run();
