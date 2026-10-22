import { parseCsv, analyzeRows } from './assets/power-core.mjs';
const $ = selector => document.querySelector(selector);
const $$ = selector => [...document.querySelectorAll(selector)];
const escape = value => String(value).replace(/[&<>"']/g, char => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[char]));
let coefficients;
const evaluate = (c, d, r) => c[0] + c[1]*d + c[2]*r + c[3]*d*d + c[4]*d*r + c[5]*r*r;
function renderMap() {
  if (!coefficients) return;
  const d = Number($('#map-distance').value), r = Number($('#map-velocity').value);
  $('#map-distance-value').textContent = `${d.toFixed(2)} m`;
  $('#map-velocity-value').textContent = `${r.toFixed(2)} m/s`;
  const hood = evaluate(coefficients.hood,d,r), roller = evaluate(coefficients.roller,d,r);
  $('#map-hood').textContent = `${hood.toFixed(3)} rot`;
  $('#map-angle').textContent = `${((hood+7.49442)/0.0857913).toFixed(1)}°`;
  $('#map-roller').textContent = `${Math.abs(roller).toFixed(1)} RPS`;
  const x = distance => 65+(distance-1.5)/6.5*575;
  const charts = [ {c:coefficients.hood,min:-3,max:0,top:43,bottom:160,label:'Hood position · motor rotations',color:'#1755ed',sign:1,ticks:[-3,-2,-1,0]}, {c:coefficients.roller,min:0,max:125,top:226,bottom:343,label:'Flywheel speed · RPS magnitude',color:'#365475',sign:-1,ticks:[0,50,100,125]} ];
  let svg = '';
  for (const chart of charts) {
    const y = value => chart.bottom - (value-chart.min)/(chart.max-chart.min)*(chart.bottom-chart.top);
    svg += `<text x="65" y="${chart.top-17}" fill="#42516c" font-size="12">${chart.label}</text>`;
    for (const tick of chart.ticks) svg += `<line x1="65" y1="${y(tick)}" x2="640" y2="${y(tick)}" stroke="#dce3ef"/><text x="53" y="${y(tick)+4}" text-anchor="end" fill="#62738c" font-size="11">${tick}</text>`;
    const points = Array.from({length:131},(_,i)=>{const distance=1.5+i*.05; return `${x(distance)},${y(chart.sign*evaluate(chart.c,distance,r))}`;}).join(' ');
    svg += `<polyline points="${points}" fill="none" stroke="${chart.color}" stroke-width="3"/><line x1="${x(d)}" x2="${x(d)}" y1="${chart.top}" y2="${chart.bottom}" stroke="#8aa9dc" stroke-dasharray="4 4"/><circle cx="${x(d)}" cy="${y(chart.sign*evaluate(chart.c,d,r))}" r="5" fill="${chart.color}" stroke="white" stroke-width="2"/>`;
    for (const tick of [1.5,3,4.5,6,8]) svg+=`<text x="${x(tick)}" y="${chart.bottom+20}" text-anchor="middle" fill="#62738c" font-size="11">${tick}</text>`;
  }
  svg+='<text x="640" y="387" text-anchor="end" fill="#42516c" font-size="11">Target distance (m)</text>';
  $('#map-svg').innerHTML=svg;
}
fetch('./assets/shotmap.json').then(r=>{if(!r.ok)throw new Error('ShotMap coefficients unavailable');return r.json();}).then(data=>{coefficients=data;renderMap();}).catch(()=>{$('#map-hood').textContent='Reload to load coefficients';});
['map-distance','map-velocity'].forEach(id=>$('#'+id).addEventListener('input',renderMap));
function renderTurret() {
  const current = Number($('#turret-current').value), desired=Number($('#turret-desired').value);
  const low=-4.235/4*360, high=1.90/4*360;
  const equivalents=[];
  for(let k=Math.ceil((low-desired)/360-1e-9);k<=Math.floor((high-desired)/360+1e-9);k++)equivalents.push(desired+360*k);
  let goal=equivalents.sort((a,b)=>Math.abs(a-current)-Math.abs(b-current))[0];
  if(goal===undefined)goal=Math.max(low,Math.min(high,desired+Math.round((current-desired)/360)*360));
  $('#turret-current-value').textContent=`${current}°`;
  $('#turret-desired-value').textContent=`${desired}°`;
  $('#turret-goal').textContent=`${goal.toFixed(1)}°`;
  $('#turret-travel').textContent=`${goal-current>=0?'+':''}${(goal-current).toFixed(1)}°`;
  const x=angle=>35+(angle-low)/(high-low)*550;
  $('#turret-svg').innerHTML=`<rect x="35" y="54" width="550" height="12" rx="6" fill="#d5e1f6"/><line x1="${x(current)}" y1="60" x2="${x(goal)}" y2="60" stroke="#1755ed" stroke-width="7"/>${[low,-180,0,high].map(a=>`<line x1="${x(a)}" y1="49" x2="${x(a)}" y2="74" stroke="#92a8c9"/><text x="${x(a)}" y="96" font-size="11" fill="#566278" text-anchor="middle">${a.toFixed(a===low?1:0)}°</text>`).join('')}<circle cx="${x(current)}" cy="40" r="6" fill="#14213b"/><line x1="${x(current)}" y1="40" x2="${x(current)}" y2="60" stroke="#14213b"/><circle cx="${x(goal)}" cy="60" r="7" fill="#1755ed" stroke="white" stroke-width="2"/><text x="35" y="20" fill="#14213b" font-size="11">● Current angle</text><text x="180" y="20" fill="#1755ed" font-size="11">● Selected legal goal</text><text x="585" y="122" fill="#566278" text-anchor="end" font-size="10">Configured logical turret travel</text>`;
}
$('#turret-current').value=-340;$('#turret-desired').value=30;
['turret-current','turret-desired'].forEach(id=>$('#'+id).addEventListener('input',renderTurret));renderTurret();

const frame=$('#calculator-frame');
let pendingReplay=null;
function sendReplay(){if(pendingReplay&&frame.dataset.ready){frame.contentWindow.postMessage({type:'ronin-replay',path:pendingReplay},location.origin);pendingReplay=null;}}
window.addEventListener('message',event=>{
  if(event.origin!==location.origin||event.source!==frame.contentWindow)return;
  if(event.data?.type==='ronin-calculator-height'&&Number.isFinite(event.data.height))frame.style.height=`${Math.min(16000,Math.max(500,event.data.height))}px`;
  if(event.data?.type==='ronin-calculator-ready'){frame.dataset.ready='true';sendReplay();}
});
$$('[data-replay]').forEach(link=>link.addEventListener('click',()=>{pendingReplay=link.dataset.replay;frame.loading='eager';sendReplay();}));

let parsed=null, report=null, powerSource='', currentFixture='mechanism', activeSeries=new Set();
const seriesColors=['#14213b','#1755ed','#a86818','#7b4db1','#207872'];
function seriesList(){
  if(!parsed)return[];
  const mechanisms=[...new Set(parsed.rows.flatMap(row=>Object.keys(row.mechanismSupplyCurrentA)))].sort();
  return[{id:'pdh',label:'PDH total',read:row=>row.totalCurrentA},...mechanisms.map(id=>({id,label:id.replaceAll('_',' '),read:row=>row.mechanismSupplyCurrentA[id]}))];
}
function showPower(text, label, fixture=null) {
  const candidate=parseCsv(text);
  if(candidate.rows.length>50000)throw new Error('Choose a CSV with 50,000 samples or fewer for this browser explorer.');
  const analysis=analyzeRows(candidate.rows,{joinMaxAgeSec:fixture==='voltage'?0.06:0.05});
  parsed=candidate;report=analysis;powerSource=label;currentFixture=fixture;
  $('#power-error').hidden=true;
  $('#power-source-label').textContent=label;
  activeSeries=new Set(seriesList().map(s=>s.id));
  $$('[data-power]').forEach(button=>{const selected=button.dataset.power===fixture;button.classList.toggle('selected',selected);button.setAttribute('aria-pressed',String(selected));});
  $('#power-detail').textContent=fixture?'Example inputs from tools/power-analysis/test/analyze.test.mjs. The displayed values describe that repository fixture.':`${candidate.rows.length.toLocaleString()} timestamped samples from your selected CSV. Each plotted series retains its original units; gaps are left open.`;
  renderPower();renderSeriesButtons();
}
function renderSeriesButtons(){
  const root=$('#power-series');root.replaceChildren();
  seriesList().forEach((series,i)=>{
    const button=document.createElement('button');button.type='button';button.setAttribute('aria-pressed',String(activeSeries.has(series.id)));
    const mark=document.createElement('i');mark.style.setProperty('--series-color',seriesColors[i%seriesColors.length]);button.append(mark,document.createTextNode(series.label));
    button.addEventListener('click',()=>{if(activeSeries.has(series.id))activeSeries.delete(series.id);else activeSeries.add(series.id);button.setAttribute('aria-pressed',String(activeSeries.has(series.id)));renderPower();});root.append(button);
  });
}
function renderPower(){
  if(!report)return;
  $('#power-summary').innerHTML=[['Samples',report.sampleCount.toLocaleString()],['Minimum voltage',report.minimumVoltage?`${report.minimumVoltage.voltageV.toFixed(2)} V`:'Not present'],['Voltage-dip events',report.voltageDips.length],['Time span',`${(report.timeRangeSec[1]-report.timeRangeSec[0]).toFixed(3)} s`]].map(([label,value])=>`<div><span>${label}</span><strong>${value}</strong></div>`).join('');
  const rows=parsed.rows,minT=report.timeRangeSec[0],maxT=report.timeRangeSec[1],duration=Math.max(maxT-minT,0.001);
  const x=t=>80+(t-minT)/duration*845;
  const series=seriesList();
  const finiteCurrents=rows.flatMap(row=>series.filter(s=>activeSeries.has(s.id)).map(s=>s.read(row))).filter(Number.isFinite);
  const maxCurrent=finiteCurrents.reduce((maximum,value)=>Math.max(maximum,value),10)*1.1;
  const panels=[{title:'PDH voltage (V)',min:0,max:16,top:44,bottom:190,series:[{read:row=>row.voltageV,color:'#1755ed'}]}, {title:'Supply current (A)',min:0,max:maxCurrent,top:284,bottom:431,series:series.map((s,i)=>({...s,color:seriesColors[i%seriesColors.length]})).filter(s=>activeSeries.has(s.id))}];
  let svg='';
  for(const panel of panels){
    const y=v=>panel.bottom-(v-panel.min)/(panel.max-panel.min)*(panel.bottom-panel.top);
    svg+=`<text x="80" y="${panel.top-18}" fill="#314762" font-size="15">${panel.title}</text>`;
    for(let i=0;i<=4;i++){const val=panel.min+(panel.max-panel.min)*i/4;svg+=`<line x1="80" y1="${y(val)}" x2="925" y2="${y(val)}" stroke="#dce3ef"/><text x="65" y="${y(val)+5}" text-anchor="end" fill="#5f7189" font-size="13">${val.toFixed(1)}</text>`;}
    for(const line of panel.series){
      let d='',previous=null;
      for(const row of rows){const value=line.read(row);if(!Number.isFinite(value)){previous=null;continue;}const gap=previous===null||row.timestampSec-previous>report.settings.joinMaxAgeSec+1e-9;d+=`${gap?'M':'L'}${x(row.timestampSec).toFixed(2)},${y(value).toFixed(2)} `;previous=row.timestampSec;}
      svg+=`<path d="${d}" fill="none" stroke="${line.color}" stroke-width="3"/>`;
      if(rows.length<100)for(const row of rows){const value=line.read(row);if(Number.isFinite(value))svg+=`<circle cx="${x(row.timestampSec)}" cy="${y(value)}" r="4" fill="${line.color}"><title>${escape(row.timestampSec)} s: ${escape(value)}</title></circle>`;}
    }
    for(let i=0;i<=4;i++){const t=minT+duration*i/4;svg+=`<text x="${x(t)}" y="${panel.bottom+25}" text-anchor="middle" fill="#5f7189" font-size="12">${t.toFixed(duration<1?3:2)}</text>`;}
  }
  svg+='<text x="925" y="484" text-anchor="end" fill="#314762" font-size="13">Time from log timestamp (s)</text>';
  $('#power-svg').innerHTML=svg;
}
let powerLoad=0;
async function loadFixture(type){const request=++powerLoad;try{const response=await fetch(`./assets/power-${type}-fixture.csv`);if(!response.ok)throw new Error('Example input could not be loaded.');const text=await response.text();if(request!==powerLoad)return;showPower(text,`Repository example · ${type==='voltage'?'voltage-event':'mechanism-signal'} fixture`,type);}catch(error){showPowerError(error);}}
function showPowerError(error){$('#power-error').textContent=error.message||'Unable to analyze this CSV.';$('#power-error').hidden=false;}
$$('[data-power]').forEach(button=>button.addEventListener('click',()=>loadFixture(button.dataset.power)));
$('#power-file').addEventListener('change',async event=>{const file=event.target.files[0];if(!file)return;const request=++powerLoad;try{if(file.size>10*1024*1024)throw new Error('Choose a CSV smaller than 10 MB for this browser explorer.');const text=await file.text();if(request!==powerLoad)return;showPower(text,`Your CSV · ${file.name}`);}catch(error){showPowerError(error);}event.target.value='';});
$('#power-export').addEventListener('click',()=>{if(!report)return;const blob=new Blob([JSON.stringify({source:powerSource,sourceKind:currentFixture?'repository-example':'user-csv',...report},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob);const anchor=document.createElement('a');anchor.href=url;anchor.download='ronin-power-analysis.json';anchor.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
loadFixture('mechanism');
