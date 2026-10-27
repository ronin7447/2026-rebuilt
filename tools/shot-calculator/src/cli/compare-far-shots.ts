import { readFileSync, writeFileSync } from 'node:fs';
import { defaultConfig as c, findBestCandidate } from '../math/trajectory';
import { evaluatePolynomial as poly } from '../math/polynomial';
import { generatedCoefficients as base } from '../math/generatedCoefficients';
import { evaluateFinalTrajectory } from '../math/evaluator';
import type { Polynomial2 } from '../math/types';
const text = readFileSync('hood-edge-free-candidate/generatedCoefficients.ts.candidate','utf8');
const coeff = (key:string) => text.match(new RegExp(key+': \\[([^\\]]+)'))![1].split(',').map(Number).filter(Number.isFinite) as Polynomial2;
const next = {hood:coeff('hood'),roller:coeff('roller')};
const java=readFileSync('../../src/main/java/frc/robot/constants/GeneratedFarShotTrials.java','utf8');
const flat=java.match(/RPS = \{\s*\{([^}]+)}/)![1].split(',').map(Number) as Polynomial2;
function evaluate(model:string,d:number,r:number,s:number,trim:number) {
 const input={distanceMeters:d,radialVelocityMps:r};
 let h=poly(base.hood,input),rpm=poly(base.roller,input);
 if(model==='no-edge-fit'){h=poly(next.hood,input);rpm=poly(next.roller,input);}
 if(model==='no-edge-search'){const b=findBestCandidate(c,input);if(!b)return null;h=b.hoodRot;rpm=b.rollerRps;}
 if(model==='fixed63'||model==='active-blend'){
  let blend=model==='fixed63'?1:Math.min(1,Math.max(0,(d-4.9)/.6));blend=blend*blend*(3-2*blend);
  h+=blend*(c.hoodSlope*63+c.hoodIntercept-h);rpm+=blend*(poly(flat,input)-rpm);
 }
 const angle=(h-c.hoodIntercept)/c.hoodSlope,pitch=angle*Math.PI/180;
 const v=(Math.abs(rpm)-c.rollerIntercept)/c.rollerSlope,hor=v*Math.cos(pitch);
 const output=Math.min(100,Math.abs(rpm)*trim),ratio=((output-c.rollerIntercept)/c.rollerSlope)/v;
 const vx=Math.sqrt(Math.max(0,hor*hor-s*s))*ratio+r,vy=s*(1-ratio),vz=v*Math.sin(pitch)*ratio;
 const legal=h>=c.minHoodRot&&h<=c.maxHoodRot&&v>0&&v<=c.maxExitSpeedMps&&Math.abs(rpm)<=100&&hor>Math.abs(s)&&vx>0;
 const ev=evaluateFinalTrajectory(c,d,vx,vy,vz);
 const z=c.hubFrontLipHeightMeters+c.fuelRadiusMeters,disc=vz*vz-2*c.gravityMps2*(z-c.shooterExitHeightMeters);
 const t=disc>=0?(vz+Math.sqrt(disc))/c.gravityMps2:NaN,offset=vx*t-d;
 const plane=legal&&ev.frontLipCleared&&Math.abs(offset)<=c.hubOpeningDepthMeters/2-c.fuelRadiusMeters&&Math.abs(vy*t)<=c.goalRadiusMeters;
 const tf=(d-c.hubOpeningDepthMeters/2)/vx;
 const lip=c.shooterExitHeightMeters+vz*tf-.5*c.gravityMps2*tf*tf-z-c.hubLipClearanceMeters;
 return {model,distance:d,radial:r,sideways:s,trim,hood:h,angle,rps:output,legal,runtime:legal&&ev.feasible,plane,
  flightTime:ev.targetPlaneCrossingTimeSec,centerHeight:ev.impactHeightMeters,entry:ev.entryAngleDeg,offset,lip,
  reason:legal?ev.reason:'actuator/velocity-limit'};
}
const models=['fixed63','active-blend','no-edge-fit','no-edge-search'];
const stationary=models.flatMap(m=>[1,1.05].flatMap(trim=>[4,4.5,4.9,5,5.5,6,7,8].map(d=>evaluate(m,d,0,0,trim))));
const summary=[];
for(const model of models.slice(0,3))for(const trim of [1,1.05]){
 let total=0,legal=0,runtime=0,plane=0;
 for(let i=0;i<=80;i++)for(let j=-5;j<=5;j++)for(const s of [-1,0,1]){
  const x=evaluate(model,4+i*.05,j*.5,s,trim)!;total++;legal+=+x.legal;runtime+=+x.runtime;plane+=+x.plane;
 }
 summary.push({model,trim,total,legal,runtime,plane});
}
writeFileSync('far-shot-comparison.json',JSON.stringify({assumptions:'Gravity-only, level robot, perfect actuators, current regression. Distance/velocity at release. Trim treated as physical speed change with nominal lead preserved. Plane diagnostic is horizontal rim and rectangular depth proxy, not full hex/net geometry. Not measured scoring probability.',stationary,summary},null,2)+'\n');
for(const trim of [1,1.05]){
 console.log('TRIM',trim);
 for(const x of stationary)if(x&&x.trim===trim&&[4,5,6,8].includes(x.distance))console.log(x.model,x.distance,x.angle.toFixed(1),x.rps.toFixed(1),'legal',x.legal,'runtime',x.runtime,'plane',x.plane,'offset',x.offset.toFixed(2),'lip',x.lip.toFixed(2));
}
console.log('GRID',JSON.stringify(summary));
