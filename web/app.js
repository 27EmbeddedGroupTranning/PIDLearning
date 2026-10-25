"use strict";
const $ = id => document.getElementById(id);
const colors = {cyan: "#55dfcb", gold: "#ffc76a", purple: "#b5a2ff", white: "#e6edf3", grid: "#273747", muted: "#92a7b8"};
let rows = [], time = 0, paused = false, after = 0, generation = -1;
let sending = false, pending = null, compiling = false, connected = false, message = "";
let session = null;
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
  const start=Math.max(0,time-15), end=Math.max(15,time);
  const x=t=>left+(t-start)/(end-start)*(right-left), y=v=>bottom-(v-min)/(max-min)*(bottom-top);
  ctx.fillStyle=colors.muted;ctx.fillText(unit,3,11);
  for(let n=0;n<=4;n++) {const v=min+(max-min)*n/4;line(ctx,left,y(v),right,y(v),colors.grid);ctx.fillStyle=colors.muted;ctx.fillText(v.toFixed(max-min>10?0:2),2,y(v)+4);}
  for(let n=0;n<=5;n++) {const t=start+n*3;line(ctx,x(t),top,x(t),bottom,colors.grid);ctx.fillStyle=colors.muted;ctx.fillText(t.toFixed(0)+"s",x(t)-7,h-7);}
  for(const s of series) {ctx.beginPath();ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1.6;ctx.setLineDash(s.dash?[5,4]:[]);s.data.forEach((r,i)=>i?ctx.lineTo(x(r.time),y(r[s.key])):ctx.moveTo(x(r.time),y(r[s.key])));ctx.stroke();}
  ctx.setLineDash([]);line(ctx,x(time),top,x(time),bottom,"#ffffff66");
}
function draw() {
  if(!rows.length)return;
  const r=rows[rows.length-1],axis=$("axis").value;
  time=r.time;
  drawGimbal(r);
  plot("response",[{data:rows,key:axis+"_target",color:colors.gold,dash:true},{data:rows,key:axis,color:colors.cyan}],"角度 / °");
  plot("terms",[{data:rows,key:axis+"_p",color:colors.cyan},{data:rows,key:axis+"_i",color:colors.gold},{data:rows,key:axis+"_d",color:colors.purple},{data:rows,key:axis+"_u",color:colors.white,width:2}],"力矩 / N·m");
  $("yawError").textContent=(r.yaw_target-r.yaw).toFixed(2)+"°";
  $("pitchError").textContent=(r.pitch_target-r.pitch).toFixed(2)+"°";
  $("torque").textContent=r.yaw_u.toFixed(3)+" N·m";
  $("clock").textContent=time.toFixed(2)+" s";
  const tail=rows.filter(sample=>sample.time>=time-1);
  const mae=tail.reduce((sum,sample)=>sum+Math.abs(sample[axis+"_target"]-sample[axis]),0)/tail.length;
  $("metrics").textContent=`${axis.toUpperCase()} 已应用目标：${r[axis+"_target"].toFixed(0)}°　最近 1 秒平均绝对误差：${mae.toFixed(2)}°`;
}
function showStatus() {
  $("status").className=message?"error":"";
  $("status").textContent=message || (compiling?"正在编译 C++，当前控制器继续运行…":!connected?"正在连接 C++ 控制器…":paused?"仿真已暂停 · 修改的目标将在继续后生效":"实时运行 · 拖动滑块即时下发目标 · C++ 固定步长 2 ms");
  $("play").textContent=paused?"继续仿真":"暂停仿真";
}
async function post(path, data={}) {
  const response=await fetch(path,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(data),signal:AbortSignal.timeout(70000)});
  const result=await response.json();
  if(!response.ok)throw new Error(result.error||"请求失败");
  return result;
}
function labels() {
  $("yawValue").textContent=$("yaw").value+"°";
  $("pitchValue").textContent=$("pitch").value+"°";
}
// 合并尚未发出的滑块指令；最多一个请求在途，避免旧目标覆盖新目标。
async function sendControl(update) {
  pending={...(pending||{}),...update};
  if(sending)return;
  sending=true;
  while(pending) {
    const data=pending; pending=null;
    try {await post("/api/control",data);message="";}
    catch(error) {message="指令下发失败："+error.message;pending=null;}
    showStatus();
  }
  sending=false;
}
async function poll() {
  try {
    const response=await fetch("/api/state?after="+after,{cache:"no-store",signal:AbortSignal.timeout(3000)});
    if(!response.ok)throw new Error("HTTP "+response.status);
    const data=await response.json();
    if(session!==data.session) {
      const hadCursor=after!==0;
      session=data.session;after=0;generation=-1;rows=[];
      if(hadCursor)return;
    }
    if(data.generation!==generation) {rows=[];generation=data.generation;}
    for(const r of data.rows) {rows.push(r);after=r.seq;}
    if(rows.length)rows=rows.filter(r=>r.time>=rows[rows.length-1].time-15).slice(-751);
    paused=data.controls.paused;
    // 初始化或重连时恢复服务端目标；避免覆盖用户正在拖动的滑块。
    if(!connected && !sending && !pending) {
      for(const key of ["yaw","pitch","load"])$(key).value=data.controls[key];
      labels();
    }
    connected=true;
    if(data.error)message="C++ 运行异常："+data.error;
    else if(!data.running)message="C++ 控制器未运行，请重新编译";
    else if(message.startsWith("连接中断") || message.startsWith("C++ 运行异常") || message.startsWith("C++ 控制器"))message="";
    showStatus();draw();
  } catch(error) {
    connected=false;after=0;generation=-1;
    message="连接中断，正在重连："+error.message;showStatus();
  } finally {setTimeout(poll,50);}
}
$("run").onclick=async()=>{
  compiling=true;message="";$("run").disabled=true;showStatus();
  try {await post("/api/run");message="";}
  catch(error) {message="重新编译失败，原控制器保持运行：\n"+error.message;}
  finally {compiling=false;$("run").disabled=false;showStatus();}
};
for(const key of ["yaw","pitch"])$(key).oninput=()=>{labels();sendControl({[key]:Number($(key).value)});};
$("load").oninput=()=>{if($("load").value!=="" && $("load").validity.valid)sendControl({load:Number($("load").value)});};
$("zero").onclick=()=>{$("yaw").value=0;$("pitch").value=0;labels();sendControl({yaw:0,pitch:0});};
$("play").onclick=()=>sendControl({paused:!paused});
$("reset").onclick=async()=>{try {await post("/api/reset");message="";}catch(error){message=error.message;}showStatus();};
$("axis").onchange=draw;
window.addEventListener("resize",draw);
labels();poll();
