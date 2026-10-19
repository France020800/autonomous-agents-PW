"""Esportazione di una simulazione in un documento HTML autonomo."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .basic_formation import (
    DynamicNavigationExperimentResult,
    ExperimentResult,
    FormationGoalExperimentResult,
    MovingObstacleExperimentResult,
    StaticObstacleExperimentResult,
)


def export_interactive_html(result: ExperimentResult, path: str | Path) -> Path:
    """Salva tutti i timestep con slider e riproduzione, senza risorse esterne.

    Il documento può essere aperto offline in un browser o pubblicato come
    pagina statica. GitHub non esegue JavaScript all'interno dei README.
    """
    dynamic = isinstance(result, (MovingObstacleExperimentResult, DynamicNavigationExperimentResult))
    has_obstacles = dynamic or isinstance(result, StaticObstacleExperimentResult)
    has_goal = isinstance(result, FormationGoalExperimentResult)
    obstacle_positions = (
        result.obstacle_positions if dynamic
        else np.array([[obstacle.position for obstacle in result.obstacles]]) if has_obstacles
        else np.empty((1, 0, 2))
    )
    points = [result.positions.reshape(-1, 2)]
    if has_obstacles:
        points.append(obstacle_positions.reshape(-1, 2))
    if has_goal:
        points.append(result.goal.reshape(1, 2))
    bounds = np.vstack(points)

    def values(array: np.ndarray) -> list[float | None]:
        return [float(value) if np.isfinite(value) else None for value in array]

    data = {
        "times": result.times.tolist(),
        "positions": result.positions.tolist(),
        "agentIds": list(result.agent_ids),
        "edges": [
            [result.agent_ids.index(edge.first_agent), result.agent_ids.index(edge.second_agent)]
            for edge in result.formation.edges
        ],
        "obstaclePositions": obstacle_positions.tolist(),
        "obstacleRadii": [float(obstacle.radius) for obstacle in result.obstacles] if has_obstacles else [],
        "dynamic": dynamic,
        "goal": result.goal.tolist() if has_goal else None,
        "navigationStart": float(result.navigation_start_time) if has_goal else None,
        "bounds": [bounds.min(axis=0).tolist(), bounds.max(axis=0).tolist()],
        "formationPotential": values(result.potentials),
        "goalPotential": values(result.goal_potentials) if has_goal else None,
        "obstaclePotential": values(result.obstacle_potentials) if has_obstacles else None,
    }
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(data, allow_nan=False).replace("<", "\\u003c")
    output.write_text(_HTML.replace("__SIMULATION_DATA__", serialized), encoding="utf-8")
    return output


_HTML = r'''<!doctype html>
<html lang="it">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MAS — Evoluzione temporale della simulazione</title>
<style>
body{font:16px system-ui,sans-serif;background:#f3f5f8;color:#172033;margin:0}
main{max-width:1100px;margin:auto;padding:24px}h1{font-size:1.5rem}
canvas{display:block;width:100%;height:560px;background:white;border:1px solid #bbc3ce;border-radius:8px}
.controls{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin:18px 0}
button,select{padding:8px;font:inherit}input[type=range]{flex:1;min-width:180px}
.legend{display:flex;gap:18px;flex-wrap:wrap;margin:12px 0}.dot{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:5px}
output{font-variant-numeric:tabular-nums}.metrics{padding:12px;background:white;border-radius:6px}
.note{color:#475569;font-size:.9rem}@media(max-width:650px){canvas{height:430px}main{padding:12px}}
</style>
</head>
<body><main>
<h1>Controllo di formazione: evoluzione temporale</h1>
<p>Selezionare un istante con lo slider oppure avviare la riproduzione. I dati sono calcolati dal simulatore Python; il browser ne visualizza gli stati registrati.</p>
<canvas id="scene" aria-label="Posizioni e traiettorie degli agenti e degli ostacoli"></canvas>
<div class="legend" id="legend"></div>
<div class="controls">
<button id="play" type="button">Riproduci</button>
<label for="time">Tempo</label><input id="time" type="range" min="0" value="0" step="1">
<output id="clock" for="time"></output>
<label for="speed">Velocità</label><select id="speed"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
</div>
<div class="metrics" id="metrics"></div>
<p class="note">Assi in metri, scala spaziale uniforme. I tratti continui rappresentano la storia degli agenti; i tratti grigi tratteggiati la storia degli ostacoli. I dischi grigi hanno il raggio fisico dell'ostacolo; i marcatori degli agenti sono simbolici, non in scala. Le linee nere descrivono i vincoli di formazione. Il goal, quando previsto, compare solo all'attivazione della navigazione. L'incrocio di due traiettorie non implica una collisione: occorre confrontare posizioni allo stesso istante.</p>
<noscript>Abilitare JavaScript per utilizzare lo slider.</noscript>
</main>
<script id="simulation-data" type="application/json">__SIMULATION_DATA__</script>
<script>
"use strict";
const data=JSON.parse(document.getElementById("simulation-data").textContent);
const canvas=document.getElementById("scene"),ctx=canvas.getContext("2d");
const slider=document.getElementById("time"),play=document.getElementById("play"),speed=document.getElementById("speed");
const colors=["#0072b2","#d55e00","#009e73","#cc79a7"];
slider.max=data.times.length-1;
const legend=document.getElementById("legend");
data.agentIds.forEach((id,i)=>{const span=document.createElement("span");span.innerHTML='<i class="dot" style="background:'+colors[i%colors.length]+'"></i>Agente '+id;legend.appendChild(span);});
if(data.obstacleRadii.length)legend.insertAdjacentHTML("beforeend",'<span><i class="dot" style="background:#64748b"></i>Ostacoli</span>');
if(data.goal)legend.insertAdjacentHTML("beforeend",'<span><i class="dot" style="background:#dc2626"></i>Goal</span>');
let running=false,baseTime=0,baseWall=0;
function draw(){
 const frame=Number(slider.value),time=data.times[frame];
 const w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;
 if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
 ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
 const low=data.bounds[0].map(x=>x-1),high=data.bounds[1].map(x=>x+1);
 const scale=Math.min((w-90)/(high[0]-low[0]),(h-80)/(high[1]-low[1]));
 const left=(w-scale*(high[0]-low[0]))/2,top=(h-scale*(high[1]-low[1]))/2;
 const point=p=>[left+(p[0]-low[0])*scale,top+(high[1]-p[1])*scale];
 function line(points,color,dash=[]){ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=1.7;ctx.setLineDash(dash);points.forEach((p,i)=>{const [x,y]=point(p);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();ctx.setLineDash([]);}
 function disk(p,r,color){const [x,y]=point(p);ctx.beginPath();ctx.fillStyle=color;ctx.arc(x,y,r,0,2*Math.PI);ctx.fill();}
 const tick=Math.pow(10,Math.floor(Math.log10(Math.max(high[0]-low[0],high[1]-low[1]))))/2;
 ctx.font="12px system-ui";ctx.fillStyle="#475569";
 for(let x=Math.ceil(low[0]/tick)*tick;x<=high[0];x+=tick){line([[x,low[1]],[x,high[1]]],"#e2e8f0");const p=point([x,low[1]]);ctx.fillText(x.toFixed(1),p[0]-10,p[1]+18);}
 for(let y=Math.ceil(low[1]/tick)*tick;y<=high[1];y+=tick){line([[low[0],y],[high[0],y]],"#e2e8f0");const p=point([low[0],y]);ctx.fillText(y.toFixed(1),p[0]-35,p[1]+4);}
 ctx.fillText("x [m]",w/2,h-5);ctx.fillText("y [m]",8,20);
 data.agentIds.forEach((id,i)=>{const history=data.positions.slice(0,frame+1).map(p=>p[i]);line(history,colors[i%colors.length]);disk(data.positions[frame][i],6,colors[i%colors.length]);});
 data.edges.forEach(([i,j])=>line([data.positions[frame][i],data.positions[frame][j]],"#334155",[5,4]));
 data.obstacleRadii.forEach((radius,j)=>{
   const current=data.obstaclePositions[data.dynamic?frame:0][j];
   if(data.dynamic)line(data.obstaclePositions.slice(0,frame+1).map(p=>p[j]),"#64748b",[5,4]);
   disk(current,radius*scale,"#64748baa");
 });
 if(data.goal&&time>=data.navigationStart){const [x,y]=point(data.goal);ctx.fillStyle="#dc2626";ctx.font="28px system-ui";ctx.fillText("★",x-12,y+10);}
 document.getElementById("clock").textContent=time.toFixed(2)+" s";
 const format=v=>v===null?"non attivo":v.toExponential(3);
 let text="Vf = "+format(data.formationPotential[frame]);
 if(data.goalPotential)text+=" · Vg = "+format(data.goalPotential[frame]);
 if(data.obstaclePotential)text+=" · Vo = "+format(data.obstaclePotential[frame]);
 document.getElementById("metrics").textContent=text;
}
function pause(){running=false;play.textContent="Riproduci";}
function animate(now){
 if(!running)return;
 const time=baseTime+(now-baseWall)/1000*Number(speed.value);
 let frame=Number(slider.value);while(frame<data.times.length-1&&data.times[frame+1]<=time)frame++;
 slider.value=frame;draw();if(frame===data.times.length-1)pause();else requestAnimationFrame(animate);
}
play.addEventListener("click",()=>{if(running){pause();return;}if(Number(slider.value)===data.times.length-1)slider.value=0;baseTime=data.times[Number(slider.value)];baseWall=performance.now();running=true;play.textContent="Pausa";requestAnimationFrame(animate);});
slider.addEventListener("input",()=>{pause();draw();});
speed.addEventListener("change",()=>{baseTime=data.times[Number(slider.value)];baseWall=performance.now();});
window.addEventListener("resize",draw);draw();
</script></body></html>
'''
