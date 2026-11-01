//! Persistent right-side encounter routes and finite directional gate batches.
//! Decisions change preferred routes and admission, never positions or ORCA outputs.
use std::collections::{BTreeMap,BTreeSet};
use bevy::math::Vec2;
use crate::{collision,formation,nav::{self,Cell,CostGrid,PathSearch,SearchStatus},orca::HalfPlane,world::Unit};

#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub enum Phase {#[default] Free,Passing,Assembling,GivingWay,FinalApproach,Queueing,Admitted,Traversing,ExitClearing,ExitBlocked}
#[derive(Clone,Copy,Debug,Default)]
pub struct Decision {pub point:Option<Vec2>,pub cap:Option<f32>,pub barrier:Option<HalfPlane>,pub phase:Phase,pub freeze_target:bool}
#[derive(Clone,Debug)]
pub struct Event {pub tick:u64,pub region:usize,pub command:u64,pub unit:i32,pub kind:&'static str,pub side:i32}
#[derive(Clone,Debug,Default)]
pub struct Report {pub passing:usize,pub waiting:usize,pub admitted:usize,pub gates:usize,
    pub batches:u64,pub switches:u64,pub max_wait:f64,pub incompatible:usize,pub unavailable:usize}
struct Origin {start:Vec2,goal:Vec2,radius:f32,area:f32,offsets:BTreeMap<i32,Vec2>}
struct Route {points:Vec<Vec2>,cursor:usize,region:usize,staging:Option<Deployment>}
struct Deployment {targets:BTreeMap<i32,formation::Target>,source:BTreeMap<i32,Vec2>,advancing:bool,region:usize,progress:f32,progress_mark:f32,last_progress_tick:u64,guide:Option<std::sync::Arc<crate::nav::FlowField>>}
fn deployment_matches(d:&Deployment,group:&[&Unit])->bool {
    d.source.len()==group.len()&&group.iter().all(|u|u.parking.as_ref().is_some_and(|t|t.valid&&d.source.get(&u.idx)==Some(&t.position)))
}
struct TerrainJob {radius:f32,goal:Vec2,search:PathSearch,staging:Deployment}
struct StagingYield {command:u64,requester:i32,requested_since:u64,original:Vec2,pocket:Vec2,anchor:Vec2,victims:Vec<i32>,since:u64,returning:bool,region:usize,target:formation::Target}
#[derive(Default)]
pub struct Traffic {
    origins:BTreeMap<u64,Origin>,routes:BTreeMap<u64,Route>,encountered:BTreeSet<(u64,u64)>,
    deployments:BTreeMap<u64,Deployment>,
    deployed:BTreeSet<u64>,
    terrain_jobs:BTreeMap<u64,TerrainJob>,terrain_attempted:BTreeSet<u64>,
    staging_yields:BTreeMap<i32,StagingYield>,review_requests:BTreeMap<(u64,i32),u64>,review_after:BTreeMap<(u64,i32),u64>,
    pub decisions:BTreeMap<i32,Decision>,pub events:Vec<Event>,pub report:Report,
    signature:Vec<u8>,width:i32,height:i32,gates:Vec<Gate>,next_region:usize,
}
impl Traffic {
    pub fn decision(&self,id:i32)->Decision {self.decisions.get(&id).copied().unwrap_or_default()}
    pub fn coordination_has_progress(&self,command:u64,tick:u64,dt:f32)->bool {
        self.deployments.get(&command).is_some_and(|d|d.advancing
            &&tick.saturating_sub(d.last_progress_tick)<(1.0/dt).ceil().max(1.0) as u64)
    }
    pub fn request_review(&mut self,id:i32,command:u64,tick:u64) {
        self.review_requests.entry((command,id)).or_insert(tick);
        if !self.routes.contains_key(&command)&&!self.deployments.contains_key(&command) {
            self.encountered.retain(|(a,b)|*a!=command&&*b!=command);
        }
        for (region,gate) in self.gates.iter_mut().enumerate() {
            if let Some(waiter)=gate.waiters.get(&id).filter(|w|w.command==command) {
                let side=waiter.side;
                if gate.tokens.get(&id).is_some_and(|t|!t.started&&!t.entered) {gate.tokens.remove(&id);}
                self.events.push(Event{tick,region,command,unit:id,kind:"readmission-review",side});
            }
        }
        self.events.push(Event{tick,region:0,command,unit:id,kind:"route-review",side:0});
    }
    pub fn assembly_pending(&self,id:i32,command:u64)->bool {
        if let Some(t)=self.staging_yields.get(&id).filter(|t|t.command==command) {return t.target.blocked==Some(formation::TargetBlock::SearchBudget);}
        self.deployments.get(&command).filter(|d|!d.advancing).and_then(|d|d.targets.get(&id))
            .is_some_and(|t|t.blocked==Some(formation::TargetBlock::SearchBudget))
    }
    pub fn assembly_guide(&self,command:u64)->Option<&crate::nav::FlowField> {
        self.deployments.get(&command).filter(|d|!d.advancing)?.guide.as_deref()
    }
    pub fn assembly_target(&self,id:i32,command:u64)->Option<&formation::Target> {
        if let Some(t)=self.staging_yields.get(&id).filter(|t|t.command==command) {return Some(&t.target);}
        self.deployments.get(&command)?.targets.get(&id)
    }
    pub fn progress_metric(&self,id:i32,command:u64,here:Vec2,goal:Vec2,g:&CostGrid)->Option<(f32,u8)> {
        if let Some(route)=self.routes.get(&command) {let origin=self.origins.get(&command)?;let offset=*origin.offsets.get(&id)?;
            let point=*route.points.get(route.cursor)?+offset;
            return Some((here.distance(point)+route.points[route.cursor..].windows(2).map(|p|p[0].distance(p[1])).sum::<f32>(),2));
        }
        if self.terrain_jobs.contains_key(&command) {return Some((here.distance(goal),7));}
        let d=self.deployments.get(&command)?;if d.advancing {return Some((here.distance(goal),4));}
        let target=d.targets.get(&id)?;let center=Vec2::new(target.center.x as f32,target.center.y as f32);
        if let Some(field)=&d.guide {if here.distance(center)>target.area_radius+formation::LOCAL_APPROACH_MARGIN
            ||!collision::disc_segment_clear(g,here,target.position,target.radius) {
            let cell=crate::nav::Cell::new(here.x.round() as i32,here.y.round() as i32);
            if crate::nav::flow_step(g,field,cell).is_some() {let distance=field.dist_at(cell.x,cell.y);if distance.is_finite() {return Some((distance,5));}}
        }}
        let remaining=target.remaining_distance(here,target.position);
        Some((if remaining.is_finite() {remaining}else{here.distance(target.position)},3))
    }
    pub fn assembly_direction(&mut self,id:i32,command:u64,here:Vec2,tick:u64,q:&mut crate::local_nav::Query<'_>,routes:&mut crate::local_nav::Routes)->Option<Vec2> {
        if self.decision(id).phase==Phase::GivingWay {
            let task=self.staging_yields.get_mut(&id).filter(|t|t.command==command)?;
            let goal=if task.returning {task.original}else{task.pocket};
            return Some(task.target.local_direction(here,goal,command,tick,q,routes));
        }
        if self.decision(id).phase!=Phase::Assembling {return None;}
        let deployment=self.deployments.get_mut(&command)?;
        if deployment.advancing {return None;}
        let target=deployment.targets.get_mut(&id)?;
        let center=Vec2::new(target.center.x as f32,target.center.y as f32);
        if let Some(field)=&deployment.guide {
            let blocked=!collision::disc_segment_clear(q.grid,here,target.position,target.radius);
            if blocked||(here.distance(center)>target.area_radius+formation::LOCAL_APPROACH_MARGIN&&!q.clear(here,target.position,true)) {
                let cell=crate::nav::Cell::new(here.x.round() as i32,here.y.round() as i32);
                if let Some(next)=crate::nav::flow_step(q.grid,field,cell) {let v=(Vec2::new(next.x as f32,next.y as f32)-Vec2::new(cell.x as f32,cell.y as f32)).normalize_or_zero();
                    if q.clear(here,here+v,true) {target.blocked=None;return Some(v);}
                }
            }
        }
        Some(target.local_direction(here,target.position,command,tick,q,routes))
    }
    pub fn update(&mut self,g:&CostGrid,units:&[Unit],tick:u64,dt:f32,budget:&mut nav::ReplanBudget) {
        self.decisions.clear();self.report.passing=0;self.report.waiting=0;self.report.admitted=0;
        self.report.incompatible=0;self.report.unavailable=0;
        self.refresh(g);
        let mut groups:BTreeMap<u64,Vec<&Unit>>=BTreeMap::new();
        for u in units {if u.on_ground&&u.parking.is_some() {if let Some(id)=u.command_id {groups.entry(id.0).or_default().push(u);}}}
        self.origins.retain(|id,_|groups.contains_key(id));self.routes.retain(|id,_|groups.contains_key(id));
        self.deployments.retain(|id,_|groups.contains_key(id));
        self.terrain_jobs.retain(|id,_|groups.contains_key(id));self.terrain_attempted.retain(|id|groups.contains_key(id));
        self.deployed.retain(|id|groups.contains_key(id));
        self.encountered.retain(|(a,b)|groups.contains_key(a)&&groups.contains_key(b));
        let mut centres=BTreeMap::new();
        for (&id,group) in &groups {
            let centre=group.iter().map(|u|formation::position(u)).sum::<Vec2>()/group.len() as f32;
            centres.insert(id,centre);
            self.origins.entry(id).or_insert_with(|| {
                let t=group[0].parking.as_ref().unwrap();
                Origin{start:centre,goal:Vec2::new(t.center.x as f32,t.center.y as f32),area:t.area_radius,
                    radius:group.iter().map(|u|formation::position(u).distance(centre)+u.parking.as_ref().unwrap().radius).fold(0.0,f32::max),
                    offsets:group.iter().map(|u|(u.idx,formation::position(u)-centre)).collect()}
            });
        }
        let mut stale=BTreeSet::new();
        self.terrain_jobs.retain(|id,j|{let valid=deployment_matches(&j.staging,&groups[id]);if !valid {stale.insert(*id);}valid});
        self.routes.retain(|id,r|{let valid=r.staging.as_ref().is_none_or(|d|deployment_matches(d,&groups[id]));if !valid {stale.insert(*id);}valid});
        for id in stale {self.terrain_attempted.remove(&id);self.deployed.remove(&id);
            self.events.push(Event{tick,region:0,command:id,unit:-1,kind:"terrain-stale",side:0});}
        let mut review=BTreeSet::new();
        self.deployments.retain(|id,d|{let valid=deployment_matches(d,&groups[id]);if !valid {review.insert(*id);}valid});
        for id in review {
            // A repaired member resumes individual parking; settled peers keep
            // their real positions rather than re-entering a temporary layout.
            self.deployed.insert(id);self.events.push(Event{tick,region:0,command:id,unit:-1,kind:"deployment-review",side:0});
        }
        // Group intersections form local encounter regions. Route geometry and side
        // are frozen for the encounter; no frame-dependent force or start delay.
        let ids:Vec<_>=groups.keys().copied().collect();let mut clusters:Vec<(Vec2,usize,BTreeSet<u64>)>=Vec::new();
        for (i,&a) in ids.iter().enumerate() {for &b in &ids[i+1..] {
            if self.encountered.contains(&(a,b))||groups[&a].iter().all(|u|u.arrival.settled())||groups[&b].iter().all(|u|u.arrival.settled()) {continue;}
            let (oa,ob)=(&self.origins[&a],&self.origins[&b]);
            if let Some(p)=intersection(oa.start,oa.goal,ob.start,ob.goal) {
                if (p-centres[&a]).dot(oa.goal-centres[&a])<0.0||(p-centres[&b]).dot(ob.goal-centres[&b])<0.0 {continue;}
                self.encountered.insert((a,b));
                if let Some(c)=clusters.iter_mut().find(|c|(c.0/c.1 as f32).distance(p)<(oa.radius+ob.radius).max(4.0)) {
                    c.0+=p;c.1+=1;c.2.extend([a,b]);
                }else{clusters.push((p,1,BTreeSet::from([a,b])));}
            }
        }}
        for (sum,count,members) in clusters {
            let centre=sum/count as f32;let radius=members.iter().map(|id|self.origins[id].radius).fold(0.0,f32::max);
            let ring=radius*2.2+0.5;let region=self.next_region;self.next_region+=1;
            let plans:Vec<_>=members.iter().filter_map(|id| {
                let o=&self.origins[id];let from=o.start-centre;let to=o.goal-centre;
                if from.length()<ring+radius||to.length()<ring+radius+o.area+formation::LOCAL_APPROACH_MARGIN {return None;}
                let start=from.to_angle();let mut turn=(to.to_angle()-start).rem_euclid(std::f32::consts::TAU);
                if turn<0.05 {return None;}if turn>std::f32::consts::TAU-0.05 {turn=0.0;}
                let mut points=Vec::new();for k in 0..=48 {points.push(centre+Vec2::from_angle(start+turn*k as f32/48.0)*ring);}
                let release=(ring+2.0*radius+2.0).min(to.length()-o.area-formation::LOCAL_APPROACH_MARGIN-radius).max(ring);
                points.push(centre+to.normalize()*release);
                let mut previous=o.start;
                if points.iter().any(|p|{let valid=collision::disc_segment_clear(g,previous,*p,radius+0.1);previous=*p;!valid}) {return None;}
                Some((*id,points))
            }).collect();
            if plans.len()!=members.len() {self.report.unavailable+=1;continue;}
            for (id,points) in plans {if self.routes.contains_key(&id) {continue;}
                self.routes.insert(id,Route{points,cursor:0,region,staging:None});
                self.events.push(Event{tick,region,command:id,unit:-1,kind:"right-route",side:1});
            }
        }
        self.terrain_detours(g,&groups,&centres,tick,budget);
        let mut done=Vec::new();
        for (&id,route) in &mut self.routes {
            let centre=centres[&id];let origin=&self.origins[&id];
            while route.cursor<route.points.len() {
                let point=route.points[route.cursor];let previous=if route.cursor==0 {origin.start}else{route.points[route.cursor-1]};
                if centre.distance(point)>1.0&&(centre-point).dot(point-previous)<0.0 {break;}
                route.cursor+=1;
            }
            if route.cursor==route.points.len() {done.push(id);continue;}
            let point=route.points[route.cursor];
            for u in &groups[&id] {if u.arrival.held()||(route.staging.is_none()&&u.parking.as_ref().is_some_and(|t|t.local_active)) {continue;}
                self.decisions.insert(u.idx,Decision{point:Some(point+origin.offsets[&u.idx]),phase:Phase::Passing,..Default::default()});self.report.passing+=1;
            }
        }
        for id in done {let route=self.routes.remove(&id).unwrap();self.events.push(Event{tick,region:route.region,command:id,unit:-1,kind:"route-cleared",side:1});
            if let Some(deployment)=route.staging.or_else(||deployment(g,&self.origins[&id],&groups[&id],route.region)) {
                self.deployments.insert(id,deployment);self.deployed.insert(id);
                self.events.push(Event{tick,region:route.region,command:id,unit:-1,kind:"deployment-start",side:1});
            }
        }
        // Large commands also need an orderly expansion after a single march.
        // This is an immediate route decision; it never delays issuing the order.
        for (&id,group) in &groups {if group.len()<64||self.routes.contains_key(&id)||self.terrain_jobs.contains_key(&id)||self.deployed.contains(&id)||group.iter().all(|u|u.arrival.settled()) {continue;}
            let region=self.next_region;self.next_region+=1;self.deployed.insert(id);
            if let Some(d)=deployment(g,&self.origins[&id],group,region) {self.deployments.insert(id,d);
                self.events.push(Event{tick,region,command:id,unit:-1,kind:"deployment-start",side:1});
            }else{self.report.unavailable+=1;}
        }
        let mut deployed=Vec::new();
        for (&id,d) in &mut self.deployments {
            let group=&groups[&id];
            if group.iter().all(|u|u.arrival.settled()) {deployed.push(id);continue;}
            if !d.advancing&&group.iter().all(|u|formation::position(u).distance(d.targets[&u.idx].position)<=0.06&&u.vx.hypot(u.vy)<=crate::arrival::QUIET_SPEED) {
                d.advancing=true;d.progress_mark=d.progress;d.last_progress_tick=tick;
                self.events.push(Event{tick,region:d.region,command:id,unit:-1,kind:"deployment-ready",side:1});
            }
            let mut max_length=0.0f32;let mut actual=1.0f32;let mut speed_rate=f32::INFINITY;let mut brake_rate=f32::INFINITY;let mut remaining=0.0f32;
            if d.advancing {for u in group {let start=d.targets[&u.idx].position;let goal=u.parking.as_ref().unwrap().position;let path=goal-start;let length=path.length();
                max_length=max_length.max(length);if length>1e-5 {actual=actual.min(((formation::position(u)-start).dot(path)/path.length_squared()).clamp(0.0,1.0));
                    speed_rate=speed_rate.min(u.speed/length);brake_rate=brake_rate.min(u.speed*6.0/length);
                    remaining=remaining.max(formation::position(u).distance(goal)/length);}
            }d.progress=d.progress.max(actual);
                // Renew only after measured common displacement. A moving
                // reference, elapsed time or small oscillations cannot renew it.
                if (d.progress-d.progress_mark)*max_length>=0.2 {d.progress_mark=d.progress;d.last_progress_tick=tick;}
            }
            let (progress,rate)=if d.advancing&&max_length>1e-5 {
                let point=(d.progress+0.5/max_length).min(1.0);let a=brake_rate*max_length;let distance=(remaining*max_length-formation::POINT_REACH_RADIUS).max(0.0);
                let v=((a*a*dt*dt+2.0*a*distance).sqrt()-a*dt).max(0.0).min(distance/dt).min(speed_rate*max_length);
                (point,v/max_length)
            }else{(1.0,0.0)};
            for u in group {if u.arrival.held() {continue;}
                let start=d.targets[&u.idx].position;let goal=u.parking.as_ref().unwrap().position;
                let point=if d.advancing {start+(goal-start)*progress}else{start};
                let cap=if d.advancing {Some((rate*start.distance(goal)).min(formation::position(u).distance(point)/dt).min(u.speed))}else{
                    let distance=formation::position(u).distance(point);
                    if distance<=0.06&&u.vx.hypot(u.vy)<=crate::arrival::QUIET_SPEED {Some(0.0)}else{
                        let distance=(distance-0.04).max(0.0);let a=u.speed*6.0;
                        Some(((a*a*dt*dt+2.0*a*distance).sqrt()-a*dt).max(0.0).min(distance/dt).min(u.speed))
                    }
                };
                self.decisions.insert(u.idx,Decision{point:Some(point),cap,phase:if d.advancing {Phase::FinalApproach}else{Phase::Assembling},freeze_target:true,..Default::default()});
                if !d.advancing {self.report.passing+=1;}
            }
        }
        for id in deployed {let d=self.deployments.remove(&id).unwrap();self.events.push(Event{tick,region:d.region,command:id,unit:-1,kind:"deployment-cleared",side:1});}
        self.staging_space(g,units,tick,dt);
        let now=tick as f64*dt as f64;
        for (index,gate) in self.gates.iter_mut().enumerate() {
            gate.update(index,g,units,tick,now,dt,&mut self.decisions,&mut self.events,&mut self.report);
        }
        self.report.gates=self.gates.len();
    }
    fn staging_space(&mut self,g:&CostGrid,units:&[Unit],tick:u64,dt:f32) {
        let live=units.iter().map(|u|(u.idx,u)).collect::<BTreeMap<_,_>>();
        let returned=self.staging_yields.iter().filter(|(id,t)|t.returning
            &&live.get(id).is_some_and(|u|u.command_id.is_some_and(|c|c.0==t.command)&&formation::position(u).distance(t.original)<=0.06&&u.vx.hypot(u.vy)<=crate::arrival::QUIET_SPEED)
            &&live.get(&t.requester).is_some_and(|u|u.command_id.is_some_and(|c|c.0==t.command))
            &&self.deployments.get(&t.command).is_some_and(|d|d.targets.get(id).is_some_and(|p|p.position==t.original)))
            .map(|(&id,_)|id).collect::<Vec<_>>();
        for id in returned {let task=self.staging_yields.remove(&id).unwrap();self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-done",side:0});}
        let stale=self.staging_yields.iter().filter(|(id,t)|
            !live.get(id).is_some_and(|u|u.command_id.is_some_and(|c|c.0==t.command))
            ||!live.get(&t.requester).is_some_and(|u|u.command_id.is_some_and(|c|c.0==t.command))
            ||!self.deployments.get(&t.command).is_some_and(|d|!d.advancing&&d.targets.get(id).is_some_and(|p|p.position==t.original)))
            .map(|(&id,_)|id).collect::<Vec<_>>();
        for id in stale {let task=self.staging_yields.remove(&id).unwrap();self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-stale",side:0});}
        let deadline=(12.0/dt).ceil() as u64;
        let expired=self.staging_yields.iter().filter(|(_,t)|tick.saturating_sub(t.since)>=deadline).map(|(&id,_)|id).collect::<Vec<_>>();
        for id in expired {let task=self.staging_yields.remove(&id).unwrap();
            // Release only this temporary controller. The ordinary deployment
            // and recovery retain the real command, target and progress age.
            self.review_requests.entry((task.command,task.requester)).and_modify(|since|*since=(*since).min(task.requested_since)).or_insert(task.requested_since);
            self.review_after.insert((task.command,task.requester),tick+(0.5/dt).ceil() as u64);
            self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-blocked",side:0});
            self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-expired",side:0});
        }
        let mut done=Vec::new();
        for (&id,task) in &mut self.staging_yields {
            let u=live[&id];let here=formation::position(u);let radius=u.parking.as_ref().unwrap().radius;
            let cleared=task.victims.iter().all(|v|live.get(v).is_none_or(|other|{
                if other.command_id.is_none_or(|c|c.0!=task.command)||other.arrival.held() {return true;}
                let reached=self.deployments[&task.command].targets.get(v).is_some_and(|t|
                    formation::position(other).distance(t.position)<=0.06&&other.vx.hypot(other.vy)<=crate::arrival::QUIET_SPEED);
                reached||formation::position(other).distance(task.anchor)>formation::PARKING_SPACING*2.0
            }));
            if !task.returning&&cleared&&yield_segment_clear(g,units,id,here,task.original,radius) {
                task.returning=true;task.target.position=task.original;task.target.blocked=None;
                self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-return",side:0});
            }
            if task.returning&&here.distance(task.original)<=0.06&&u.vx.hypot(u.vy)<=crate::arrival::QUIET_SPEED {
                done.push(id);continue;
            }
            let goal=if task.returning {task.original}else{task.pocket};
            self.decisions.insert(id,Decision{point:Some(goal),cap:Some(staging_yield_cap(here,goal,u.speed,dt)),phase:Phase::GivingWay,freeze_target:true,..Default::default()});
        }
        for id in done {let task=self.staging_yields.remove(&id).unwrap();self.events.push(Event{tick,region:task.region,command:task.command,unit:id,kind:"staging-yield-done",side:0});}
        self.review_requests.retain(|(command,id),_|self.deployments.get(command).is_some_and(|d|!d.advancing)
            &&live.get(id).is_some_and(|u|u.command_id.is_some_and(|c|c.0==*command)&&!u.arrival.held())
            &&self.decisions.get(id).is_some_and(|d|d.phase==Phase::Assembling&&d.cap!=Some(0.0)));
        self.review_after.retain(|(command,id),_|self.deployments.contains_key(command)&&live.get(id).is_some_and(|u|u.command_id.is_some_and(|c|c.0==*command)));
        if self.staging_yields.len()>=4 {return;}
        let mut requests=self.review_requests.iter().filter(|(key,_)|self.review_after.get(key).is_none_or(|after|tick>=*after))
            .filter(|((command,id),_)|!self.staging_yields.values().any(|t|t.command==*command&&t.requester==*id))
            .map(|(&(command,id),&since)|(since,command,id)).collect::<Vec<_>>();
        requests.sort_unstable();let Some((requested_since,command,requester))=requests.first().copied() else{return;};
        self.review_after.insert((command,requester),tick+(0.5/dt).ceil() as u64);
        let anchor=formation::position(live[&requester]);let deployment=&self.deployments[&command];
        let mut holders=units.iter().filter(|u|u.idx!=requester&&u.on_ground&&u.command_id.is_some_and(|c|c.0==command)&&!u.arrival.held()
            &&!self.staging_yields.contains_key(&u.idx)&&self.decisions.get(&u.idx).is_some_and(|d|d.phase==Phase::Assembling&&d.cap==Some(0.0)))
            .collect::<Vec<_>>();
        holders.sort_by(|a,b|formation::position(a).distance_squared(anchor).total_cmp(&formation::position(b).distance_squared(anchor)).then(a.idx.cmp(&b.idx)));
        for holder in holders.into_iter().take(8) {
            let here=formation::position(holder);if here.distance(anchor)>formation::PARKING_SPACING*4.0 {continue;}
            if self.staging_yields.iter().any(|(id,t)|{
                let other=live[id];let goal=if t.returning {t.original}else{t.pocket};
                formation::position(other).distance(here)<formation::PARKING_SPACING*4.0
                    &&(formation::position(other).distance(goal)>0.06||other.vx.hypot(other.vy)>crate::arrival::QUIET_SPEED)
            }) {continue;}
            let original=&deployment.targets[&holder.idx];let radius=original.radius;
            let away=(here-anchor).to_angle();
            for length in [formation::PARKING_SPACING,formation::PARKING_SPACING*1.5] {for k in 0..8 {
                let pocket=here+Vec2::from_angle(away+std::f32::consts::TAU*k as f32/8.0)*length;
                if !yield_segment_clear(g,units,holder.idx,here,pocket,radius) {continue;}
                if self.deployments.values().flat_map(|d|d.targets.iter()).any(|(id,t)|*id!=holder.idx&&pocket.distance(t.position)<radius+t.radius+0.0001) {continue;}
                if self.staging_yields.iter().any(|(id,t)|pocket.distance(t.pocket)<radius+live[id].parking.as_ref().unwrap().radius+0.0001) {continue;}
                let victims=units.iter().filter(|u|u.command_id.is_some_and(|c|c.0==command)&&!self.staging_yields.contains_key(&u.idx)
                    &&self.decisions.get(&u.idx).is_some_and(|d|d.phase==Phase::Assembling&&d.cap!=Some(0.0))
                    &&formation::position(u).distance(anchor)<=formation::PARKING_SPACING*4.0).map(|u|u.idx).collect();
                let mut target=original.clone();target.position=pocket;target.blocked=None;
                self.staging_yields.insert(holder.idx,StagingYield{command,requester,requested_since,original:original.position,pocket,anchor,victims,since:tick,returning:false,region:deployment.region,target});
                self.decisions.insert(holder.idx,Decision{point:Some(pocket),cap:Some(staging_yield_cap(here,pocket,holder.speed,dt)),phase:Phase::GivingWay,freeze_target:true,..Default::default()});
                self.review_requests.remove(&(command,requester));
                self.events.push(Event{tick,region:deployment.region,command,unit:holder.idx,kind:"staging-yield-start",side:0});return;
            }}
        }
        self.events.push(Event{tick,region:deployment.region,command,unit:requester,kind:"staging-space-unavailable",side:0});
    }
    fn refresh(&mut self,g:&CostGrid) {
        if self.width==g.w&&self.height==g.h&&self.signature==g.cost {return;}
        self.width=g.w;self.height=g.h;self.signature=g.cost.clone();self.gates=discover_gates(g);
        self.terrain_jobs.clear();self.terrain_attempted.clear();
        self.staging_yields.clear();self.review_requests.clear();self.review_after.clear();
        self.routes.retain(|_,r|r.staging.is_none());
    }
    fn terrain_detours(&mut self,g:&CostGrid,groups:&BTreeMap<u64,Vec<&Unit>>,centres:&BTreeMap<u64,Vec2>,tick:u64,budget:&mut nav::ReplanBudget) {
        self.terrain_jobs.retain(|id,_|!self.routes.contains_key(id));
        for (&id,group) in groups {
            if group.len()<64||self.routes.contains_key(&id)||self.deployed.contains(&id)||self.terrain_attempted.contains(&id)||group.iter().all(|u|u.arrival.settled())||group.iter().any(|u|!u.parking.as_ref().unwrap().valid) {continue;}
            self.terrain_attempted.insert(id);
            let origin=&self.origins[&id];let here=centres[&id];
            let radius=group.iter().map(|u|formation::position(u).distance(here)+u.parking.as_ref().unwrap().radius).fold(0.0,f32::max)+1.25;
            if collision::disc_segment_clear(g,here,origin.goal,radius) {continue;}
            let region=self.next_region;self.next_region+=1;
            let Some(staging)=deployment(g,origin,group,region) else{continue;};
            let goal=staging.targets.values().map(|t|t.position).sum::<Vec2>()/group.len() as f32;
            let start=Cell::new(here.x.round() as i32,here.y.round() as i32);let end=Cell::new(goal.x.round() as i32,goal.y.round() as i32);
            let point=|c:Cell|Vec2::new(c.x as f32,c.y as f32);
            if !collision::disc_segment_clear(g,here,point(start),radius)||!collision::disc_segment_clear(g,point(end),goal,radius) {continue;}
            self.terrain_jobs.insert(id,TerrainJob{radius,goal,search:PathSearch::new(g,start,end),staging});
            self.events.push(Event{tick,region,command:id,unit:-1,kind:"terrain-search",side:0});
        }
        let ids:Vec<_>=self.terrain_jobs.keys().copied().collect();
        for id in ids {
            let job=self.terrain_jobs.get_mut(&id).unwrap();let owner=groups[&id][0].idx;
            let key=nav::SearchKey{owner,command:id,x:job.goal.x.to_bits(),y:job.goal.y.to_bits(),class:2};
            let permit=budget.request(key,tick);let radius=job.radius;
            let point=|c:Cell|Vec2::new(c.x as f32,c.y as f32);
            let status=if permit==0 {SearchStatus::Pending}else{
                let before=job.search.expanded;
                let status=job.search.advance(g,permit,&|x,y|!collision::disc_clear(g,Vec2::new(x as f32,y as f32),radius),
                    &mut |a,b|collision::disc_segment_clear(g,point(a),point(b),radius).then_some(0.0));
                budget.record(job.search.expanded-before);status
            };
            if status==SearchStatus::Pending {
                let centre=centres[&id];let direction=(job.goal-centre).normalize_or_zero();let next=centre+direction*2.0;
                if collision::disc_segment_clear(g,centre,next,radius) {for u in &groups[&id] {
                    self.decisions.insert(u.idx,Decision{point:Some(next+self.origins[&id].offsets[&u.idx]),phase:Phase::Passing,..Default::default()});
                    self.report.passing+=1;
                }}
                continue;
            }
            let job=self.terrain_jobs.remove(&id).unwrap();budget.finish(key,status==SearchStatus::Found,0);
            if status!=SearchStatus::Found {self.report.unavailable+=1;
                self.events.push(Event{tick,region:job.staging.region,command:id,unit:-1,kind:"terrain-unavailable",side:0});continue;}
            let cells=&job.search.cells;let mut points=Vec::new();
            for i in 0..cells.len() {
                if i>0&&i+1<cells.len()&&cells[i].x-cells[i-1].x==cells[i+1].x-cells[i].x
                    &&cells[i].y-cells[i-1].y==cells[i+1].y-cells[i].y {continue;}
                points.push(point(cells[i]));
            }
            points.push(job.goal);let region=job.staging.region;let centre=centres[&id];
            let fits=groups[&id].iter().all(|u|formation::position(u).distance(centre)+u.parking.as_ref().unwrap().radius<=job.radius);
            let join=if fits {points.iter().position(|p|collision::disc_segment_clear(g,centre,*p,job.radius))}else{None};
            let Some(join)=join else {self.report.unavailable+=1;
                self.events.push(Event{tick,region,command:id,unit:-1,kind:"terrain-join-unavailable",side:0});continue;};
            let mut connected=vec![centre];connected.extend_from_slice(&points[join..]);let points=connected;
            self.deployments.remove(&id);self.deployed.remove(&id);
            self.routes.insert(id,Route{points,cursor:0,region,staging:Some(job.staging)});
            self.events.push(Event{tick,region,command:id,unit:-1,kind:"terrain-route",side:0});
        }
    }
}
fn yield_segment_clear(g:&CostGrid,units:&[Unit],owner:i32,a:Vec2,b:Vec2,radius:f32)->bool {
    collision::disc_segment_clear(g,a,b,radius)&&units.iter().filter(|u|u.on_ground&&u.idx!=owner).all(|u|
        collision::segment_avoids_disc(a,b,formation::position(u),radius+u.parking.as_ref().map_or(0.45,|t|t.radius)))
}
fn staging_yield_cap(here:Vec2,goal:Vec2,speed:f32,dt:f32)->f32 {
    let distance=(here.distance(goal)-0.04).max(0.0);let a=speed*6.0;
    ((a*a*dt*dt+2.0*a*distance).sqrt()-a*dt).max(0.0).min(distance/dt).min(speed)
}
fn deployment(g:&CostGrid,o:&Origin,group:&[&Unit],region:usize)->Option<Deployment> {
    let direction=o.goal-o.start;let axis=if direction.x.abs()>=direction.y.abs() {Vec2::X*direction.x.signum()}else{Vec2::Y*direction.y.signum()};
    let peer_radius=group.iter().map(|u|u.parking.as_ref().unwrap().radius).fold(0.0,f32::max);
    let shift=3.0*o.area+formation::LOCAL_APPROACH_MARGIN+2.0;
    for k in 0..=(shift/formation::PARKING_SPACING).ceil() as usize {
        let center=o.goal-axis*(shift-k as f32*formation::PARKING_SPACING).max(0.0);let cell=crate::nav::Cell::new(center.x.round() as i32,center.y.round() as i32);
        let mut targets=BTreeMap::new();
        for u in group {let mut t=u.parking.as_ref().unwrap().clone();t.position=center+(t.position-o.goal)*2.0;t.area_radius*=2.0;
            t.phase=t.phase*2.0+(center-Vec2::new(cell.x as f32,cell.y as f32))/formation::PARKING_SPACING;
            t.center=cell;t.predecessor=None;t.frozen=true;t.local_active=true;t.blocked=None;targets.insert(u.idx,t);
        }
        // A stationary staging member must leave room for a peer to pass
        // between it and terrain. Actual motion still uses each unit's radius.
        if !group.iter().all(|u|{let t=&targets[&u.idx];collision::disc_clear(g,t.position,t.radius+2.0*peer_radius)
            &&collision::disc_segment_clear(g,t.position,u.parking.as_ref().unwrap().position,t.radius)}) {continue;}
        let guide=if g.cost.contains(&0) {let sources=targets.values().map(|t|crate::nav::Cell::new(t.position.x.round() as i32,t.position.y.round() as i32)).collect::<Vec<_>>();
            let mut field=crate::nav::FlowField::default();crate::nav::compute_flow_field_cpu_multi(g,&sources,&mut field).then(||std::sync::Arc::new(field))
        }else{None};
        let source=group.iter().map(|u|(u.idx,u.parking.as_ref().unwrap().position)).collect();
        return Some(Deployment{targets,source,advancing:false,region,progress:0.0,progress_mark:0.0,last_progress_tick:0,guide});
    }None
}
fn intersection(a:Vec2,b:Vec2,c:Vec2,d:Vec2)->Option<Vec2> {
    let r=b-a;let s=d-c;let det=r.perp_dot(s);
    if det.abs()>1e-4 {
        let t=(c-a).perp_dot(s)/det;let u=(c-a).perp_dot(r)/det;
        return (t>0.05&&t<0.95&&u>0.05&&u<0.95).then_some(a+r*t);
    }
    if r.dot(s)>=0.0||r.length_squared()<1e-8||(c-a).perp_dot(r).abs()>0.1*r.length() {return None;}
    let axis=r.normalize();let lo=axis.dot(c-a).min(axis.dot(d-a)).max(0.0);let hi=axis.dot(c-a).max(axis.dot(d-a)).min(r.length());
    (hi-lo>2.0).then_some(a+axis*(lo+hi)*0.5)
}

struct Waiter {side:i32,command:u64,since:f64,slot:usize}
struct Token {side:i32,rank:usize,started:bool,entered:bool,aligned:bool}
struct Gate {axis:Vec2,lateral:Vec2,lo:f32,hi:f32,across:f32,
    waiters:BTreeMap<i32,Waiter>,tokens:BTreeMap<i32,Token>,owner:Option<i32>,last:Option<i32>,opened:f64,exit_blocked:bool}
fn discover_gates(g:&CostGrid)->Vec<Gate> {
    if !g.valid() {return Vec::new();}let mut keys=BTreeSet::new();let mut result=Vec::new();
    for b in crate::nav::detect_bottlenecks(g,1,g.w.max(g.h),2) {
        let (x,y)=if b.axis==0 {((b.c_min+b.c_max)/2,(b.t_min+b.t_max)/2)}else{((b.t_min+b.t_max)/2,(b.c_min+b.c_max)/2)};
        let horizontal=!g.walkable(x,y-1)&&!g.walkable(x,y+1);
        let vertical=!g.walkable(x-1,y)&&!g.walkable(x+1,y);
        if !horizontal&&!vertical {continue;}
        let (axis,lateral,mut lo,mut hi,across)=if horizontal {(Vec2::X,Vec2::Y,x,x,y)}else{(Vec2::Y,Vec2::X,y,y,x)};
        let narrow=|a:i32|{let (x,y)=if horizontal {(a,across)}else{(across,a)};
            g.walkable(x,y)&&if horizontal {!g.walkable(x,y-1)&&!g.walkable(x,y+1)}else{!g.walkable(x-1,y)&&!g.walkable(x+1,y)}};
        while narrow(lo-1) {lo-=1;}while narrow(hi+1) {hi+=1;}
        if !keys.insert((horizontal,lo,hi,across)) {continue;}
        result.push(Gate{axis,lateral,lo:lo as f32-0.5,hi:hi as f32+0.5,across:across as f32,
            waiters:BTreeMap::new(),tokens:BTreeMap::new(),owner:None,last:None,opened:0.0,exit_blocked:false});
    }result
}
impl Gate {
    fn point(&self,along:f32,across:f32)->Vec2 {self.axis*along+self.lateral*across}
    fn update(&mut self,region:usize,g:&CostGrid,units:&[Unit],tick:u64,now:f64,dt:f32,
        decisions:&mut BTreeMap<i32,Decision>,events:&mut Vec<Event>,report:&mut Report) {
        const BATCH:usize=4;const ADMISSION_SECONDS:f64=12.0;
        let was_blocked=self.exit_blocked;self.exit_blocked=false;
        let live:BTreeMap<_,_>=units.iter().map(|u|(u.idx,u)).collect();
        self.waiters.retain(|id,w|live.get(id).is_some_and(|u|u.command_id.is_some_and(|c|c.0==w.command)&&!u.arrival.held()));
        self.tokens.retain(|id,_|live.contains_key(id));
        let mut cleared=Vec::new();
        for (&id,token) in &mut self.tokens {let u=live[&id];let p=formation::position(u);let a=p.dot(self.axis);let r=u.parking.as_ref().map_or(0.45,|t|t.radius);
            if token.started&&!token.entered&&(p.dot(self.lateral)-self.across).abs()<0.5+r
                &&((token.side==1&&a+r>self.lo+0.001)||(token.side==-1&&a-r<self.hi-0.001)) {token.entered=true;events.push(Event{tick,region,command:u.command_id.map_or(0,|c|c.0),unit:id,kind:"entered",side:token.side});}
            let exit=if token.side==1 {self.hi+2.0*r+1.2}else{self.lo-2.0*r-1.2};
            if (a-exit)*token.side as f32>=0.0 {cleared.push(id);events.push(Event{tick,region,command:u.command_id.map_or(0,|c|c.0),unit:id,kind:"exit-cleared",side:token.side});}
        }
        for id in cleared {self.tokens.remove(&id);self.waiters.remove(&id);}
        // A new command revokes an unused permit. An actor already inside keeps
        // the physical direction until its rear clears, then follows its new goal.
        self.tokens.retain(|id,t|t.entered||self.waiters.contains_key(id));
        for (&id,token) in &self.tokens {
            let u=live[&id];
            if token.entered&&!u.arrival.held()&&!self.waiters.contains_key(&id) {
                if let Some(command)=u.command_id {
                    self.waiters.insert(id,Waiter{side:token.side,command:command.0,since:now,slot:0});
                    events.push(Event{tick,region,command:command.0,unit:id,kind:"retarget-drain",side:token.side});
                }
            }
        }
        for u in units {if !u.on_ground||u.arrival.held() {continue;}
            let Some(t)=&u.parking else{continue;};let Some(command)=u.command_id else{continue;};
            let p=formation::position(u).dot(self.axis);let goal=t.position.dot(self.axis);
            let side=if p<self.lo&&goal>self.hi {1}else if p>self.hi&&goal<self.lo {-1}else{0};
            let entry=if side==1 {self.lo}else{self.hi};
            if side!=0&&(p-entry).abs()<=12.0&&!self.waiters.contains_key(&u.idx)&&!self.tokens.contains_key(&u.idx) {
                let used:BTreeSet<_>=self.waiters.values().filter(|w|w.side==side).map(|w|w.slot).collect();let slot=(0..).find(|i|!used.contains(i)).unwrap();
                self.waiters.insert(u.idx,Waiter{side,command:command.0,since:now,slot});
                events.push(Event{tick,region,command:command.0,unit:u.idx,kind:"queue",side});
            }
        }
        if self.owner.is_some()&&now-self.opened>=ADMISSION_SECONDS {self.tokens.retain(|id,t|{
            if !t.entered {events.push(Event{tick,region,command:live[id].command_id.map_or(0,|c|c.0),unit:*id,kind:"admission-expired",side:t.side});}t.entered
        });}
        if self.tokens.is_empty() {if let Some(side)=self.owner.take() {self.last=Some(side);events.push(Event{tick,region,command:0,unit:-1,kind:"batch-drained",side});}
            let mut ready:Vec<_>=self.waiters.iter().filter(|(id,_)|!self.tokens.contains_key(id)).collect();
            ready.sort_by(|(ia,a),(ib,b)|a.since.total_cmp(&b.since).then(a.command.cmp(&b.command)).then(ia.cmp(ib)));
            let chosen=self.last.and_then(|last|ready.iter().find(|(_,w)|w.side==-last).map(|(_,w)|w.side)).or_else(||ready.first().map(|(_,w)|w.side));
            if let Some(side)=chosen {
                let exit=if side==1 {self.hi}else{self.lo};
                let blocked=units.iter().any(|u| {let p=formation::position(u);let a=p.dot(self.axis);
                    (a-exit)*(side as f32)>=0.0&&(a-exit)*(side as f32)<2.1&&(p.dot(self.lateral)-self.across).abs()<1.0});
                if !blocked {
                    self.owner=Some(side);self.opened=now;report.batches+=1;if self.last.is_some_and(|last|last!=side) {report.switches+=1;}
                    events.push(Event{tick,region,command:0,unit:-1,kind:"batch-start",side});
                    for (rank,row) in ready.iter().filter(|(_,w)|w.side==side).take(BATCH).enumerate() {let id=*row.0;self.tokens.insert(id,Token{side,rank,started:false,entered:false,aligned:false});
                        events.push(Event{tick,region,command:live[&id].command_id.map_or(0,|c|c.0),unit:id,kind:"admitted",side});}
                }else{self.exit_blocked=true;if !was_blocked {events.push(Event{tick,region,command:0,unit:-1,kind:"exit-blocked",side});}
                }
            }
        }
        // Grant a finite batch, but feed its alignment point in physical order.
        // A follower starts after the preceding member has entered with spacing.
        let launch:Vec<_>=self.tokens.iter().filter_map(|(&id,t)| {
            if t.started {return None;}
            let preceding=self.tokens.iter().filter(|(_,p)|p.rank<t.rank).max_by_key(|(_,p)|p.rank);
            let ready=preceding.is_none_or(|(previous,p)| {
                let unit=live[previous];let coordinate=formation::position(unit).dot(self.axis);
                let radius=unit.parking.as_ref().map_or(0.45,|target|target.radius);
                let alignment=if t.side==1 {self.lo-radius-1.0}else{self.hi+radius+1.0};
                p.entered&&(coordinate-alignment)*t.side as f32>=formation::PARKING_SPACING
            });ready.then_some(id)
        }).collect();
        for id in launch {let t=self.tokens.get_mut(&id).unwrap();t.started=true;events.push(Event{tick,region,command:live[&id].command_id.map_or(0,|c|c.0),unit:id,kind:"launch",side:t.side});}
        let inside:BTreeSet<_>=units.iter().filter_map(|u| {
            let p=formation::position(u);let a=p.dot(self.axis);let r=u.parking.as_ref().map_or(0.45,|t|t.radius);
            if a+r<=self.lo+0.001||a-r>=self.hi-0.001||(p.dot(self.lateral)-self.across).abs()>0.5+r {return None;}
            self.tokens.get(&u.idx).map(|t|t.side).or_else(||self.waiters.get(&u.idx).map(|w|w.side))
        }).collect();report.incompatible+=usize::from(inside.len()>1);
        for (&id,w) in &self.waiters {let u=live[&id];let here=formation::position(u);let radius=u.parking.as_ref().map_or(0.45,|t|t.radius);
            report.max_wait=report.max_wait.max(now-w.since);
            if let Some(token)=self.tokens.get_mut(&id).filter(|t|t.started) {
                let entry=if token.side==1 {self.lo-radius-1.0}else{self.hi+radius+1.0};
                if (here.dot(self.axis)-entry).abs()<0.3&&(here.dot(self.lateral)-self.across).abs()<0.03 {token.aligned=true;}
                let exit=if token.side==1 {self.hi+2.0*radius+1.2}else{self.lo-2.0*radius-1.2};
                let point=self.axis*if token.aligned||token.entered {exit}else{entry}+self.lateral*self.across;
                let distance=here.distance(point);let cap=if token.aligned||token.entered {u.speed}else{(2.0*u.speed*6.0*(distance-0.005).max(0.0)).sqrt().min(u.speed)};
                let phase=if token.entered {if here.dot(self.axis)>=self.lo&&here.dot(self.axis)<=self.hi {Phase::Traversing}else{Phase::ExitClearing}}else{Phase::Admitted};
                decisions.insert(id,Decision{point:Some(point),cap:Some(cap),phase,..Default::default()});report.admitted+=1;
            }else{
                let row=w.slot%8;let column=w.slot/8;let along=if w.side==1 {self.lo-2.0-column as f32*formation::PARKING_SPACING}else{self.hi+2.0+column as f32*formation::PARKING_SPACING};
                // Right-side pockets leave the through-axis and exit clear.
                let right=Vec2::new((self.axis*w.side as f32).y,-(self.axis*w.side as f32).x);
                let point=self.point(along,self.across)+right*(2.0+row as f32*formation::PARKING_SPACING);
                let legal=collision::disc_clear(g,point,radius);let distance=here.distance(point);
                let cap=if !legal||distance<=0.04 {0.0}else{(2.0*u.speed*6.0*(distance-0.02).max(0.0)).sqrt().min(u.speed)};
                let stop=if w.side==1 {self.lo-radius-collision::CONTACT_GAP}else{self.hi+radius+collision::CONTACT_GAP};
                let normal=-self.axis*w.side as f32;let offset=(here.dot(self.axis)-stop)*w.side as f32/dt;
                let barrier=HalfPlane::new(normal.as_dvec2(),offset as f64).ok();
                decisions.insert(id,Decision{point:Some(if legal {point}else{here}),cap:Some(cap),barrier,phase:if self.exit_blocked {Phase::ExitBlocked}else{Phase::Queueing},..Default::default()});report.waiting+=1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::*;
    use crate::{nav::Cell,nav_scenario::{self,SceneConfig},play::{self,Selection},world::SimEnv};
    fn safe(w:&World) {
        let e=w.resource::<SimEnv>();let c=&e.runtime.contacts;
        assert!(c.max_penetration<=0.02&&c.max_swept_penetration<=0.02);
        assert_eq!((c.illegal_static_trajectories,c.unresolved_pairs,c.initial_overlaps),(0,0,0));
        assert_eq!(e.traffic.report.incompatible,0);
    }
    fn staging_review_fixture()->(World,Schedule,Vec<(Entity,i32)>,BTreeMap<i32,Vec2>,u64) {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::Single,per_group:64,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);safe(&w);
        let entities=w.query::<(Entity,&Unit)>().iter(&w).map(|(e,u)|(e,u.idx)).collect::<Vec<_>>();
        let command=w.get::<Unit>(entities[0].0).unwrap().command_id.unwrap().0;
        let stage=w.resource::<SimEnv>().traffic.deployments[&command].targets.iter().map(|(id,t)|(*id,t.position)).collect::<BTreeMap<_,_>>();
        // Reproduce an already assembled group with one unresolved local entry.
        // From this initial snapshot all motion uses the production schedule.
        for (e,id) in &entities {let mut u=w.get_mut::<Unit>(*e).unwrap();let p=if *id==0 {stage[&1]+Vec2::splat(0.95)}else{stage[id]};
            u.cell_x=p.x;u.cell_y=p.y;u.vx=0.0;u.vy=0.0;u.preferred_velocity=None;}
        schedule.run(&mut w);safe(&w);
        let tick=w.resource::<SimEnv>().tick;w.resource_mut::<SimEnv>().traffic.request_review(0,command,tick);
        schedule.run(&mut w);safe(&w);
        assert!(w.resource::<SimEnv>().traffic.events.iter().any(|e|e.kind=="staging-yield-start"),
            "an explicit staging review did not create a physical giving-way task");
        (w,schedule,entities,stage,command)
    }
    #[test]
    fn a_staging_review_opens_a_pocket_and_restores_the_original_targets() {
        let (mut w,mut schedule,entities,stage,command)=staging_review_fixture();
        let original_positions=w.resource::<SimEnv>().traffic.deployments[&command].source.clone();
        let original=entities.iter().map(|(e,id)|(*id,(w.get::<Unit>(*e).unwrap().command_id,original_positions[id]))).collect::<BTreeMap<_,_>>();
        let holder=w.resource::<SimEnv>().traffic.events.iter().find(|e|e.kind=="staging-yield-start").unwrap().unit;
        let holder_entity=entities.iter().find(|(_,id)|*id==holder).unwrap().0;
        let mut parked=None;let mut completed=false;let mut moved_aside=false;
        for tick in 0..2400 {schedule.run(&mut w);safe(&w);
            let u=w.get::<Unit>(holder_entity).unwrap();if u.traffic_phase==Phase::GivingWay&&formation::position(u).distance(stage[&holder])>0.2 {moved_aside=true;}
            for (e,id) in &entities {let u=w.get::<Unit>(*e).unwrap();assert_eq!(u.command_id,original[id].0);assert_eq!(u.parking.as_ref().unwrap().position,original[id].1);}
            if nav_scenario::all_parked(&mut w) {let since=*parked.get_or_insert(tick);if tick-since>=180 {completed=true;break;}}else{parked=None;}
        }
        assert!(completed,"giving-way did not finish with real stable parking");
        assert!(moved_aside,"giving-way never produced a real displacement");
        assert!(w.resource::<SimEnv>().traffic.events.iter().any(|e|e.kind=="staging-yield-done"),"events={:?}",
            w.resource::<SimEnv>().traffic.events.iter().filter(|e|e.kind.starts_with("staging-")).take(32).collect::<Vec<_>>());
        assert!(w.query::<&Unit>().iter(&w).all(|u|u.arrival.max_drift<=0.05));
    }
    #[test]
    fn slow_coherent_members_do_not_start_individual_detours_while_the_group_advances() {
        let (mut w,_,command)=compression_fixture();let mut units=w.query::<&Unit>().iter(&w).cloned().collect::<Vec<_>>();units.sort_by_key(|u|u.idx);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let first=env.tick;
        let stage=env.traffic.deployments[&command].targets.iter().map(|(id,t)|(*id,t.position)).collect::<BTreeMap<_,_>>();
        assert!(units.iter().any(|u|{let d=stage[&u.idx].distance(u.parking.as_ref().unwrap().position);d>1.2&&d<3.0}),"fixture needs a short, unfinished contraction path");
        // Feed the progress observer legal, gently accelerated homothetic
        // snapshots. This isolates classification, not a motion acceptance run.
        // Short paths move less than 0.2 while the common layout keeps advancing.
        for n in 0..180 {let time=n as f32/60.0;let progress=if time<1.0 {0.01*time*time}else{0.01+0.02*(time-1.0)};let rate=0.02*time.min(1.0);
            for u in &mut units {let start=stage[&u.idx];let path=u.parking.as_ref().unwrap().position-start;let p=start+path*progress;
                u.cell_x=p.x;u.cell_y=p.y;u.vx=path.x*rate;u.vy=path.y*rate;}
            let tick=first+n;env.buffers.local_routes.budget.begin(tick);
            env.traffic.update(&env.nav,&units,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            for u in &mut units {let d=env.traffic.decision(u.idx);u.traffic_phase=d.phase;u.traffic_hold=d.cap==Some(0.0);}
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.traffic.deployments[&command].advancing);
            assert!(!env.recovery.events.iter().any(|e|e.tick==tick&&matches!(e.kind,"yield-start"|"pocket-start"|"search-start")),
                "individual recovery bent a still-progressing coherent deployment: progress={} tick={} events={:?}",progress,tick,
                env.recovery.events.iter().filter(|e|e.tick==tick).take(8).collect::<Vec<_>>());
        }
        assert!(env.traffic.deployments[&command].progress>0.04);
    }
    fn compression_fixture()->(World,Schedule,u64) {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleWidth,per_group:1024,seed:9,jitter:0.02,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();let command=w.query::<&Unit>().iter(&w).next().unwrap().command_id.unwrap().0;
        for _ in 0..240 {schedule.run(&mut w);safe(&w);if w.resource::<SimEnv>().traffic.deployments.contains_key(&command) {break;}}
        let stage=w.resource::<SimEnv>().traffic.deployments[&command].targets.iter().map(|(id,t)|(*id,t.position)).collect::<BTreeMap<_,_>>();
        // Start from a legal expanded physical snapshot, with the same small
        // alignment errors as the real compression boundary. Later motion is
        // performed only by the production schedule.
        for mut u in w.query::<&mut Unit>().iter_mut(&mut w) {let angle=u.idx as f32*1.37;let p=stage[&u.idx]+Vec2::from_angle(angle)*0.04;
            u.cell_x=p.x;u.cell_y=p.y;u.vx=0.0;u.vy=0.0;u.preferred_velocity=None;u.no_progress_seconds=0.0;}
        w.resource_mut::<SimEnv>().recovery=crate::recovery::Recovery::default();
        (w,schedule,command)
    }
    #[test]
    fn a_stationary_coherent_deployment_still_enters_recovery() {
        let (mut w,_,command)=compression_fixture();let mut units=w.query::<&Unit>().iter(&w).cloned().collect::<Vec<_>>();units.sort_by_key(|u|u.idx);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let first=env.tick;
        // Probe an unchanged physical snapshot. A finite coordination wait
        // cannot keep suppressing recovery when the group never advances.
        for n in 1..=180 {let tick=first+n;env.buffers.local_routes.budget.begin(tick);
            env.traffic.update(&env.nav,&units,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            for u in &mut units {let d=env.traffic.decision(u.idx);u.traffic_phase=d.phase;u.traffic_hold=d.cap==Some(0.0);}
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.recovery.report.expanded<=crate::recovery::EXPANSIONS_PER_TICK);
        }
        assert!(env.recovery.events.iter().any(|e|matches!(e.kind,"yield-start"|"pocket-start"|"replan-queued"|"search-start")),"a stationary group was hidden by an unlimited coordination wait");
        assert!(units.iter().all(|u|u.command_id.is_some_and(|c|c.0==command)&&!u.arrival.settled()&&u.no_progress_seconds>=2.9));
    }
    #[test]
    fn a_blocked_giving_way_task_returns_control_without_refreshing_wait_age() {
        let (mut w,_,_,_,command)=staging_review_fixture();
        let units=w.query::<&Unit>().iter(&w).cloned().collect::<Vec<_>>();
        let mut env=w.remove_resource::<SimEnv>().unwrap();
        let (&holder,task)=env.traffic.staging_yields.iter().next().unwrap();
        let started=task.since;let requester=task.requester;
        let requested=env.traffic.events.iter().filter(|e|e.kind=="route-review"&&e.unit==requester).map(|e|e.tick).min().unwrap();
        let before=units.iter().map(|u|(u.idx,(formation::position(u),u.command_id,u.parking.as_ref().unwrap().position,u.no_progress_seconds))).collect::<BTreeMap<_,_>>();
        // A persistently blocked physical snapshot must not hold the finite
        // giving-way controller forever. Only the clock advances in this probe.
        let tick=started+720;env.buffers.local_routes.budget.begin(tick);
        env.traffic.update(&env.nav,&units,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
        assert_ne!(env.traffic.decision(holder).phase,Phase::GivingWay,
            "an expired giving-way task still prevents ordinary recovery");
        assert!(env.traffic.events.iter().any(|e|e.unit==holder&&e.kind=="staging-yield-expired"));
        assert_eq!(env.traffic.review_requests.get(&(command,requester)),Some(&requested),"review wait age was refreshed");
        for u in &units {let p=&before[&u.idx];assert_eq!((formation::position(u),u.command_id,u.parking.as_ref().unwrap().position,u.no_progress_seconds),*p);}
    }
    #[test]
    fn retargeting_a_giving_way_member_discards_the_old_pocket() {
        let (mut w,mut schedule,entities,_,old_command)=staging_review_fixture();
        let holder=w.resource::<SimEnv>().traffic.events.iter().find(|e|e.kind=="staging-yield-start").unwrap().unit;
        let entity=entities.iter().find(|(_,id)|*id==holder).unwrap().0;
        play::issue_orders_at(&mut w,&[(vec![entity],Cell::new(20,20))]);let command=w.get::<Unit>(entity).unwrap().command_id;
        assert_ne!(command.map(|c|c.0),Some(old_command));
        schedule.run(&mut w);safe(&w);
        assert!(!w.resource::<SimEnv>().traffic.staging_yields.contains_key(&holder));
        assert_ne!(w.resource::<SimEnv>().traffic.decision(holder).phase,Phase::GivingWay);
        assert!(w.resource::<SimEnv>().traffic.events.iter().any(|e|e.unit==holder&&e.kind=="staging-yield-stale"));
        let mut completed=false;
        for _ in 0..2400 {schedule.run(&mut w);safe(&w);let u=w.get::<Unit>(entity).unwrap();assert_eq!(u.command_id,command);
            if u.arrival.settled() {completed=true;break;}}
        assert!(completed,"the old yielding task prevented completion of a replacement order");
    }
    #[test]
    fn group_route_keeps_front_members_until_the_staging_region() {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleObstacle,per_group:1024,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);
        let mut units:Vec<_>=w.query::<&Unit>().iter(&w).cloned().collect();units.sort_by_key(|u|u.idx);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let command=units[0].command_id.unwrap().0;
        let job=env.traffic.terrain_jobs.remove(&command).unwrap();
        env.traffic.routes.insert(command,Route{points:vec![job.goal],cursor:0,region:job.staging.region,staging:Some(job.staging)});
        for u in &mut units {u.cell_x+=100.0;assert!(collision::disc_clear(&env.nav,formation::position(u),u.parking.as_ref().unwrap().radius));}
        let front=&mut units[511];let t=front.parking.as_mut().unwrap();t.local_active=true;
        assert!(Vec2::new(front.cell_x,front.cell_y).distance(Vec2::new(t.center.x as f32,t.center.y as f32))<t.area_radius+formation::LOCAL_APPROACH_MARGIN);
        let tick=env.tick+1;env.buffers.local_routes.budget.begin(tick);
        env.traffic.update(&env.nav,&units,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
        assert_eq!(env.traffic.decision(511).phase,Phase::Passing,"front member abandoned a live group terrain route");
        assert!(env.traffic.decision(511).point.is_some());
    }
    #[test]
    fn group_route_uses_repaired_parking_targets() {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleObstacle,per_group:64,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);
        let owner=w.query::<(Entity,&Unit)>().iter(&w).find(|(_,u)|u.idx==0).unwrap().0;
        let command=w.get::<Unit>(owner).unwrap().command_id.unwrap();
        let old=w.resource::<SimEnv>().traffic.terrain_jobs[&command.0].staging.targets[&0].position;
        // A repair changes one valid reservation without issuing a new command.
        let repaired={let mut u=w.get_mut::<Unit>(owner).unwrap();let t=u.parking.as_mut().unwrap();t.position.y-=formation::PARKING_SPACING;
            let p=t.position;u.goal=Cell::new(p.x.round() as i32,p.y.round() as i32);p};
        {let mut env=w.resource_mut::<SimEnv>();env.reservations.claims.get_mut(&0).unwrap().position=repaired;env.reservations.rebuild_owners();}
        schedule.run(&mut w);safe(&w);
        let env=w.resource::<SimEnv>();let current=env.traffic.terrain_jobs[&command.0].staging.targets[&0].position;
        assert!(current.distance(old-Vec2::Y*formation::PARKING_SPACING*2.0)<0.001,"pending group route kept a repaired target's old position");
        assert_eq!(w.get::<Unit>(owner).unwrap().command_id,Some(command));assert_eq!(env.reservations.claims.len(),64);
    }
    #[test]
    fn group_route_rejoins_the_current_legal_snapshot() {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleObstacle,per_group:64,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);
        let mut units:Vec<_>=w.query::<&Unit>().iter(&w).cloned().collect();units.sort_by_key(|u|u.idx);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let command=units[0].command_id.unwrap().0;
        let job=env.traffic.terrain_jobs.get_mut(&command).unwrap();let end=Cell::new(job.goal.x.round() as i32,job.goal.y.round() as i32);
        // Feed a completed legal planner trail and a newer legal position snapshot.
        // This isolates route handoff; it is not a movement or completion benchmark.
        let controls=[Cell::new(32,80),Cell::new(84,45),Cell::new(106,45),end];let mut cells=vec![controls[0]];
        for edge in controls.windows(2) {let a=edge[0];let b=edge[1];let n=(b.x-a.x).abs().max((b.y-a.y).abs());
            for i in 1..=n {cells.push(Cell::new((a.x as f32+(b.x-a.x) as f32*i as f32/n as f32).round() as i32,
                (a.y as f32+(b.y-a.y) as f32*i as f32/n as f32).round() as i32));}}
        let point=|c:Cell|Vec2::new(c.x as f32,c.y as f32);
        assert!(cells.windows(2).all(|p|collision::disc_segment_clear(&env.nav,point(p[0]),point(p[1]),job.radius)));
        job.search.cells=cells;job.search.status=SearchStatus::Found;
        for u in &mut units {u.cell_x+=74.0;assert!(collision::disc_clear(&env.nav,formation::position(u),u.parking.as_ref().unwrap().radius));}
        let seen=env.tick+1;let key=nav::SearchKey{owner:units[0].idx,command,x:job.goal.x.to_bits(),y:job.goal.y.to_bits(),class:2};
        env.buffers.local_routes.budget.request(key,seen);
        let tick=seen+1;env.buffers.local_routes.budget.begin(tick);
        env.traffic.update(&env.nav,&units,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
        assert!(env.traffic.events.iter().any(|e|e.kind=="terrain-route"));
        for u in &units {let next=env.traffic.decision(u.idx).point.unwrap();
            assert!(collision::disc_segment_clear(&env.nav,formation::position(u),next,u.parking.as_ref().unwrap().radius),"route handoff crossed terrain from the current snapshot");}
    }
    #[test]
    fn retargeting_a_group_discards_its_pending_terrain_route() {
        let mut w=World::new();
        nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleObstacle,per_group:64,sample_every:1,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);safe(&w);
        let old=w.query::<&Unit>().iter(&w).next().unwrap().command_id.unwrap();
        let entities:Vec<_>=w.query::<(Entity,&Unit)>().iter(&w).map(|(e,_)|e).collect();
        assert_eq!(play::issue_orders_at(&mut w,&[(entities,Cell::new(32,120))])[0].accepted,64);
        let current=w.query::<&Unit>().iter(&w).next().unwrap().command_id.unwrap();assert_ne!(old,current);
        for _ in 0..3 {schedule.run(&mut w);safe(&w);}
        assert!(!w.resource::<SimEnv>().traffic.events.iter().any(|e|e.command==old.0&&e.kind=="terrain-route"));
        assert!(!w.resource::<SimEnv>().traffic.terrain_jobs.contains_key(&old.0));
        assert!(!w.resource::<SimEnv>().traffic.routes.contains_key(&old.0));
        assert!(w.query::<&Unit>().iter(&w).all(|u|u.command_id==Some(current)&&u.parking.as_ref().unwrap().center==Cell::new(32,120)&&!u.arrival.settled()));
        assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),64);
    }
    #[test]
    fn a_budgeted_group_detour_clears_terrain_and_stays_parked() {
        let mut w=World::new();
        nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleObstacle,per_group:64,sample_every:1,..default()}).unwrap();
        let commands:BTreeMap<_,_>=w.query::<&Unit>().iter(&w).map(|u|(u.idx,u.command_id)).collect();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);safe(&w);
        assert!(w.query::<&Unit>().iter(&w).all(|u|u.start_delay==0.0&&u.vx.hypot(u.vy)>0.01),"group planning staggered the command startup");
        let mut parked=None;let mut complete=None;let mut expanded=0;
        for tick in 2..=4200 {
            schedule.run(&mut w);safe(&w);
            let env=w.resource::<SimEnv>();assert!(env.recovery.report.expanded<=nav::ReplanBudget::LIMIT);
            expanded+=env.recovery.report.expanded;
            for u in w.query::<&Unit>().iter(&w) {assert_eq!(u.command_id,commands[&u.idx]);}
            if nav_scenario::all_parked(&mut w) {let since=*parked.get_or_insert(tick);if tick-since>=180 {complete=Some(since);break;}}
            else{parked=None;}
        }
        assert!(complete.is_some(),"coherent terrain detour did not complete and hold parking");
        let env=w.resource::<SimEnv>();assert!(env.traffic.events.iter().any(|e|e.kind=="terrain-route"));assert!(expanded>0);
        assert_eq!(env.reservations.claims.len(),64);
        assert!(w.query::<&Unit>().iter(&w).all(|u|u.arrival.max_drift<=0.05));
        println!("traffic_terrain_complete tick={} parked_ticks=180 expanded={expanded}",complete.unwrap());
    }
    #[test]
    fn a_wall_side_staging_target_leaves_room_for_a_full_sized_peer() {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig{kind:nav_scenario::SceneKind::SingleWidth,per_group:1024,..default()}).unwrap();
        let mut schedule=nav_scenario::schedule();schedule.run(&mut w);safe(&w);
        let command=w.query::<&Unit>().iter(&w).next().unwrap().command_id.unwrap().0;
        let env=w.resource::<SimEnv>();
        let staging=&env.traffic.terrain_jobs[&command].staging;
        let target=staging.targets.values().min_by(|a,b|a.position.x.total_cmp(&b.position.x)).unwrap();
        // The corridor wall ends at112.5. A moving peer's centre must stay to
        // its right while a staged member holds its actual expanded position.
        let lane=112.5+target.radius+0.05;
        let a=Vec2::new(lane,target.position.y-formation::PARKING_SPACING);
        let b=Vec2::new(lane,target.position.y+formation::PARKING_SPACING);
        assert!(collision::disc_segment_clear(&env.nav,a,b,target.radius));
        let bodies=[formation::Body{id:9999,position:target.position,radius:target.radius,departing:false,idle:false}];
        let mut index=crate::spatial::Index::default();index.rebuild(&[crate::spatial::Aabb::point(target.position)]);let mut neighbors=Vec::new();
        let mut q=crate::local_nav::Query{grid:&env.nav,bodies:&bodies,index:&mut index,neighbors:&mut neighbors,owner:-1,radius:target.radius,max_radius:target.radius};
        assert!(q.clear(a,b,true),"a staged member sealed the wall-side exit aisle for a full-sized peer");
    }
    #[test]
    fn assembly_guide_replans_around_a_confirmed_holding_body() {
        let mut g=CostGrid{w:192,h:160,cost:vec![1;192*160],..Default::default()};
        for y in 54..=106 {for x in 92..=98 {g.cost[y*192+x]=0;}}
        let here=Vec2::new(85.0,80.0);let body=formation::Body{id:7,position:here,radius:0.45,departing:true,idle:false};
        let mut plan=formation::plan(&g,&formation::components(&g),Cell::new(160,80),&[body],&[body],
            &BTreeSet::from([7]),&mut formation::Reservations::default(),None).unwrap();
        let mut target=plan.targets.remove(0).1;target.center=Cell::new(134,80);target.area_radius=48.0;
        let next=nav::flow_step(&g,&plan.guide,Cell::new(85,80)).unwrap();
        let direction=(Vec2::new(next.x as f32,next.y as f32)-here).normalize();
        let holding=formation::Body{id:8,position:here+direction*1.2,radius:0.45,departing:false,idle:false};
        assert!(here.distance(holding.position)>body.radius+holding.radius);
        let mut traffic=Traffic::default();traffic.deployments.insert(1,Deployment{source:BTreeMap::from([(7,target.position)]),
            targets:BTreeMap::from([(7,target)]),advancing:false,region:0,progress:0.0,progress_mark:0.0,last_progress_tick:0,guide:Some(plan.guide)});
        traffic.decisions.insert(7,Decision{phase:Phase::Assembling,..Default::default()});
        let mut index=crate::spatial::Index::default();index.rebuild(&[crate::spatial::Aabb::point(holding.position)]);let mut neighbors=Vec::new();
        let bodies=[holding];let mut q=crate::local_nav::Query{grid:&g,bodies:&bodies,index:&mut index,neighbors:&mut neighbors,owner:7,radius:0.45,max_radius:0.45};
        let mut routes=crate::local_nav::Routes::default();
        let actual=traffic.assembly_direction(7,1,here,0,&mut q,&mut routes).unwrap();
        assert!(actual==Vec2::ZERO||q.clear(here,here+actual,true),"shared guide bypassed a confirmed holding body");
        assert_eq!(routes.budget.pending(),1,"a blocked guide did not request the shared local planner");
        assert_eq!(traffic.deployments[&1].targets[&7].blocked,Some(formation::TargetBlock::SearchBudget));
    }
    #[test]
    fn assembly_keeps_the_static_guide_inside_a_large_local_region() {
        let mut g=CostGrid{w:192,h:160,cost:vec![1;192*160],..Default::default()};
        for y in 54..=106 {for x in 92..=98 {g.cost[y*192+x]=0;}}
        let here=Vec2::new(85.0,80.0);
        let body=formation::Body{id:7,position:here,radius:0.45,departing:true,idle:false};
        let mut plan=formation::plan(&g,&formation::components(&g),Cell::new(160,80),&[body],&[body],
            &BTreeSet::from([7]),&mut formation::Reservations::default(),None).unwrap();
        let mut target=plan.targets.remove(0).1;target.center=Cell::new(134,80);target.area_radius=48.0;
        target.blocked=Some(formation::TargetBlock::SearchBudget);
        assert!(here.distance(Vec2::new(134.0,80.0))<target.area_radius+formation::LOCAL_APPROACH_MARGIN);
        assert!(!collision::disc_segment_clear(&g,here,target.position,target.radius));
        let next=crate::nav::flow_step(&g,&plan.guide,Cell::new(85,80)).unwrap();
        let expected=(Vec2::new(next.x as f32,next.y as f32)-here).normalize();
        assert!(collision::disc_segment_clear(&g,here,here+expected,target.radius));
        let mut traffic=Traffic::default();traffic.deployments.insert(1,Deployment{source:BTreeMap::from([(7,target.position)]),targets:BTreeMap::from([(7,target)]),
            advancing:false,region:0,progress:0.0,progress_mark:0.0,last_progress_tick:0,guide:Some(plan.guide)});
        traffic.decisions.insert(7,Decision{phase:Phase::Assembling,..Default::default()});
        let mut index=crate::spatial::Index::default();let mut neighbors=Vec::new();
        let mut q=crate::local_nav::Query{grid:&g,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:7,radius:0.45,max_radius:0.45};
        let mut routes=crate::local_nav::Routes::default();
        let actual=traffic.assembly_direction(7,1,here,0,&mut q,&mut routes).unwrap();
        assert!(actual.dot(expected)>0.99,"existing terrain detour was replaced by a per-unit search");
        assert_eq!(routes.budget.pending(),0);
        assert_eq!(traffic.deployments[&1].targets[&7].blocked,None);
        let metric=traffic.progress_metric(7,1,here,Vec2::new(160.0,80.0),&g).unwrap();
        let advanced=traffic.progress_metric(7,1,here+expected,Vec2::new(160.0,80.0),&g).unwrap();
        assert_eq!(metric.1,5);assert!(advanced.0<metric.0);
    }
    #[test]
    fn encounter_routes_keep_their_side_and_all_four_groups_start_together() {
        let mut w=World::new();nav_scenario::populate(&mut w,&SceneConfig::default()).unwrap();let mut s=nav_scenario::schedule();s.run(&mut w);safe(&w);
        assert_eq!(w.resource::<SimEnv>().traffic.routes.len(),4);
        let paths:BTreeMap<_,_>=w.resource::<SimEnv>().traffic.routes.iter().map(|(id,r)|(*id,r.points.clone())).collect();
        let mut moved=BTreeSet::new();for u in w.query::<&Unit>().iter(&w) {assert_eq!(u.start_delay,0.0);if u.vx.hypot(u.vy)>0.01 {moved.insert(u.command_id.unwrap().0);}}
        assert_eq!(moved.len(),4);
        for _ in 0..120 {s.run(&mut w);safe(&w);}
        let traffic=&w.resource::<SimEnv>().traffic;assert_eq!(traffic.events.iter().filter(|e|e.kind=="right-route").count(),4);
        for (id,r) in &traffic.routes {assert_eq!(r.points,paths[id]);}
    }
    #[test]
    fn readmission_review_keeps_waiting_age_and_does_not_reverse_an_entered_token() {
        let mut traffic=Traffic::default();traffic.gates.push(Gate{axis:Vec2::X,lateral:Vec2::Y,lo:10.5,hi:20.5,across:12.0,
            waiters:BTreeMap::from([(7,Waiter{side:1,command:3,since:2.0,slot:0})]),
            tokens:BTreeMap::from([(7,Token{side:1,rank:0,started:false,entered:false,aligned:false})]),owner:Some(1),last:None,opened:3.0,exit_blocked:false});
        traffic.request_review(7,3,600);assert_eq!(traffic.gates[0].waiters[&7].since,2.0);assert!(!traffic.gates[0].tokens.contains_key(&7));
        traffic.gates[0].tokens.insert(7,Token{side:1,rank:0,started:true,entered:true,aligned:true});
        traffic.request_review(7,3,700);assert_eq!(traffic.gates[0].waiters[&7].since,2.0);assert_eq!(traffic.gates[0].tokens[&7].side,1);
    }
    #[test]
    fn retargeted_assembly_uses_only_the_current_command_and_phase() {
        let g=CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()};let here=Vec2::new(12.0,12.0);
        let body=formation::Body{id:7,position:here,radius:0.45,departing:true,idle:false};
        let mut target=formation::plan(&g,&formation::components(&g),Cell::new(18,12),&[body],&[body],
            &BTreeSet::from([7]),&mut formation::Reservations::default(),None).unwrap().targets.remove(0).1;
        target.position=Vec2::new(6.0,12.0);let mut traffic=Traffic::default();
        traffic.deployments.insert(1,Deployment{source:BTreeMap::from([(7,target.position)]),targets:BTreeMap::from([(7,target.clone())]),advancing:false,region:0,progress:0.0,progress_mark:0.0,last_progress_tick:0,guide:None});
        target.position=Vec2::new(18.0,12.0);
        traffic.deployments.insert(2,Deployment{source:BTreeMap::from([(7,target.position)]),targets:BTreeMap::from([(7,target)]),advancing:false,region:1,progress:0.0,progress_mark:0.0,last_progress_tick:0,guide:None});
        traffic.decisions.insert(7,Decision{phase:Phase::Assembling,..Default::default()});
        let mut index=crate::spatial::Index::default();let mut neighbors=Vec::new();
        let mut q=crate::local_nav::Query{grid:&g,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:7,radius:0.45,max_radius:0.45};
        let mut routes=crate::local_nav::Routes::default();
        assert_eq!(traffic.assembly_direction(7,2,here,0,&mut q,&mut routes),Some(Vec2::X));
        assert_eq!(traffic.assembly_direction(7,3,here,0,&mut q,&mut routes),None);
        traffic.decisions.get_mut(&7).unwrap().phase=Phase::Queueing;
        assert_eq!(traffic.assembly_direction(7,2,here,0,&mut q,&mut routes),None);
    }
    #[test]
    fn retarget_inside_a_gate_drains_before_reversing_and_preserves_the_new_order() {
        let mut g=CostGrid{w:48,h:26,cost:vec![0;48*26],..Default::default()};
        for y in 2..24 {for x in 1..18 {g.cost[y*48+x]=1;}for x in 30..47 {g.cost[y*48+x]=1;}}
        for x in 18..30 {g.cost[13*48+x]=1;}
        let mut w=World::new();let env=SimEnv{nav:g,..Default::default()};let mut time=Time::<Fixed>::from_duration(env.runtime.fixed_step());time.advance_by(env.runtime.fixed_step());
        w.insert_resource(time);w.insert_resource(env);w.insert_resource(Selection::default());
        let entity=w.spawn(Unit{idx:0,cell_x:8.0,cell_y:13.0,..Default::default()}).id();
        assert_eq!(play::issue_orders_at(&mut w,&[(vec![entity],Cell::new(40,13))])[0].accepted,1);
        let mut s=nav_scenario::schedule();let mut entered=false;
        for _ in 0..600 {s.run(&mut w);safe(&w);if w.get::<Unit>(entity).unwrap().cell_x>19.0 {entered=true;break;}}
        assert!(entered);let previous=w.get::<Unit>(entity).unwrap().command_id;
        assert_eq!(play::issue_orders_at(&mut w,&[(vec![entity],Cell::new(8,13))])[0].accepted,1);
        let command=w.get::<Unit>(entity).unwrap().command_id;assert_ne!(command,previous);
        s.run(&mut w);safe(&w);
        let u=w.get::<Unit>(entity).unwrap();assert_eq!(u.goal,Cell::new(8,13));assert_eq!(u.command_id,command);
        assert!(w.resource::<SimEnv>().traffic.decision(0).point.unwrap().x>u.cell_x);
        let mut completed=false;
        for _ in 0..1800 {s.run(&mut w);safe(&w);let u=w.get::<Unit>(entity).unwrap();
            assert_eq!(u.command_id,command);assert_eq!(u.goal,Cell::new(8,13));
            if u.arrival.settled() {completed=true;break;}
        }
        assert!(completed);let traffic=&w.resource::<SimEnv>().traffic;
        assert!(traffic.events.iter().any(|e|e.kind=="retarget-drain"&&e.command==command.unwrap().0));
        assert_eq!(traffic.events.iter().filter(|e|e.kind=="entered").map(|e|e.side).collect::<Vec<_>>(),vec![1,-1]);
    }
    #[test]
    fn four_groups_64_finish_symmetric_and_perturbed_crossings_and_stay_parked() {
        for jitter in [0.0,0.02] {
            let config=SceneConfig{seed:42,jitter,sample_every:1,..Default::default()};let mut w=World::new();nav_scenario::populate(&mut w,&config).unwrap();
            let mut s=nav_scenario::schedule();let mut complete=None;
            for tick in 1..=3600 {s.run(&mut w);safe(&w);
                if tick%300==0 {let count=w.query::<&Unit>().iter(&w).filter(|u|u.arrival.settled()).count();println!("traffic_cross jitter={jitter} tick={tick} arrived={count} report={:?}",w.resource::<SimEnv>().traffic.report);
                    if tick>=2400 {for u in w.query::<&Unit>().iter(&w).filter(|u|!u.arrival.settled()).take(3) {println!("traffic_parking id={} p={:?} v=({},{}) target={:?}",u.idx,formation::position(u),u.vx,u.vy,u.parking);}}
                }
                if w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()) {complete=Some(tick);break;}
            }
            if complete.is_none() {for u in w.query::<&Unit>().iter(&w).filter(|u|!u.arrival.settled()).take(8) {println!("traffic_incomplete id={} position={:?} goal={:?} phase={:?}",u.idx,formation::position(u),u.parking.as_ref().map(|t|t.position),u.traffic_phase);}}
            assert!(complete.is_some(),"four groups did not complete jitter={jitter}");
            let parked:BTreeMap<_,_>=w.query::<&Unit>().iter(&w).map(|u|(u.idx,formation::position(u))).collect();
            for _ in 0..180 {s.run(&mut w);safe(&w);for u in w.query::<&Unit>().iter(&w) {assert!(u.arrival.settled());assert!(formation::position(u).distance(parked[&u.idx])<=0.05);}}
            println!("traffic_cross_complete jitter={jitter} tick={} parked_ticks=180",complete.unwrap());
        }
    }
    #[test]
    fn narrow_bidirectional_teams_receive_bounded_batches_and_clear_the_exit() {
        let mut g=CostGrid{w:48,h:26,cost:vec![0;48*26],..Default::default()};
        for y in 2..24 {for x in 1..18 {g.cost[y*48+x]=1;}for x in 30..47 {g.cost[y*48+x]=1;}}
        for x in 18..30 {g.cost[13*48+x]=1;}
        let mut w=World::new();let env=SimEnv{nav:g,..Default::default()};let mut time=Time::<Fixed>::from_duration(env.runtime.fixed_step());time.advance_by(env.runtime.fixed_step());
        w.insert_resource(time);w.insert_resource(env);w.insert_resource(Selection::default());let mut a=Vec::new();let mut b=Vec::new();
        for i in 0..8 {a.push(w.spawn(Unit{idx:i,cell_x:7.0+(i%2) as f32*1.1,cell_y:11.35+(i/2) as f32*1.1,..Default::default()}).id());
            b.push(w.spawn(Unit{idx:8+i,cell_x:39.0+(i%2) as f32*1.1,cell_y:11.35+(i/2) as f32*1.1,..Default::default()}).id());}
        let reports=play::issue_orders_at(&mut w,&[(a,Cell::new(40,13)),(b,Cell::new(8,13))]);assert!(reports.iter().all(|r|r.accepted==8));
        let mut s=nav_scenario::schedule();let mut done=None;
        for tick in 1..=3600 {s.run(&mut w);safe(&w);
            let mut directions=BTreeSet::new();for u in w.query::<&Unit>().iter(&w) {if u.cell_x+0.45>17.501&&u.cell_x-0.45<29.499&&u.cell_y>12.05&&u.cell_y<13.95 {directions.insert(u.command_id.unwrap().0);}}
            assert!(directions.len()<=1,"incompatible directions in the physical passage");
            if tick%600==0 {println!("traffic_gate tick={tick} report={:?}",w.resource::<SimEnv>().traffic.report);}
            if w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()) {done=Some(tick);break;}
        }
        if done.is_none() {for u in w.query::<&Unit>().iter(&w).filter(|u|!u.arrival.settled()) {println!("gate_incomplete id={} p={:?} phase={:?}",u.idx,formation::position(u),u.traffic_phase);}}
        assert!(done.is_some(),"narrow queues never completed");let traffic=&w.resource::<SimEnv>().traffic;
        let sides:BTreeSet<_>=traffic.events.iter().filter(|e|e.kind=="batch-start").map(|e|e.side).collect();assert_eq!(sides,BTreeSet::from([-1,1]));
        assert!(traffic.report.switches>=3);assert!(traffic.gates.iter().all(|g|g.tokens.is_empty()));
        let mut count=0;for e in &traffic.events {if e.kind=="batch-start" {count=0;}if e.kind=="admitted" {count+=1;assert!(count<=4);}}
        println!("traffic_gate_complete tick={} switches={} max_wait={}",done.unwrap(),traffic.report.switches,traffic.report.max_wait);
    }
}
