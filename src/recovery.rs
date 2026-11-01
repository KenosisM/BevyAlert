//! Progress-based recovery with command versions and a shared, sliced A* queue.
use std::collections::{BTreeMap,BTreeSet,VecDeque};
use bevy::math::Vec2;
use crate::{arrival,collision,formation::{self,Body},local_nav::Query,nav::{self,Cell,CostGrid,PathSearch,SearchStatus},
    spatial::{Aabb,Index},traffic::{self,Traffic},world::Unit};

pub const EXPANSIONS_PER_TICK:usize=nav::ReplanBudget::LIMIT;
const MAX_JOBS:usize=8;
const DIAGNOSE_SECONDS:f64=1.0;
#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub enum Phase {#[default] Monitoring,ScheduledWait,Yielding,Pocket,Queued,Searching,Following,Cooldown,NoRoute,WaitingForSpace,Blocked}
#[derive(Clone,Debug)]
pub struct Event {pub tick:u64,pub unit:i32,pub command:u64,pub kind:&'static str}
#[derive(Clone,Debug,Default)]
pub struct Report {pub diagnosed:u64,pub yielding:usize,pub queued:usize,pub searching:usize,pub following:usize,
    pub no_route:usize,pub waiting_space:usize,pub scheduled_wait:usize,pub expanded:usize,pub budget_pending:usize,
    pub max_no_progress:f64,pub max_queue_age:f64,pub stale_results:u64,pub replans:u64,pub blocked:usize}
struct Task {points:Vec<Vec2>,cursor:usize,deadline:Option<f64>,kind:Phase,began:f64}
struct State {command:u64,target:Vec2,anchor:Vec2,best:f32,metric_key:(u32,u32,u8),last_progress:f64,
    phase:Phase,stage:u8,side:f32,attempts:u32,next_try:f64,diagnosed:bool,queued_since:f64,
    task:Option<Task>,epoch:u64,lease_epoch:u64,padding:f32,probe:bool}
struct Job {command:u64,target:Vec2,goal:Vec2,epoch:u64,grid:CostGrid,origin:Vec2,step:f32,search:PathSearch,probe:bool,has_leases:bool}
#[derive(Default)]
pub struct Recovery {
    states:BTreeMap<i32,State>,queue:VecDeque<i32>,jobs:BTreeMap<i32,Job>,running:VecDeque<i32>,
    hard:Vec<Body>,hard_index:Index,hard_neighbors:Vec<usize>,signature:Vec<u32>,epoch:u64,
    fixed:Vec<Body>,fixed_index:Index,fixed_neighbors:Vec<usize>,
    leases:Vec<Body>,lease_signature:Vec<i32>,lease_epoch:u64,
    soft:Vec<Body>,soft_index:Index,soft_neighbors:Vec<usize>,
    repair_queue:VecDeque<u64>,repair_after:BTreeMap<u64,f64>,repair_tick:Option<u64>,
    pub report:Report,pub events:Vec<Event>,
    pub renewals:Vec<(i32,u64)>,
}
fn point(c:Cell)->Vec2 {Vec2::new(c.x as f32,c.y as f32)}
fn gate_owns(p:traffic::Phase)->bool {matches!(p,traffic::Phase::GivingWay|traffic::Phase::Queueing|traffic::Phase::Admitted|traffic::Phase::Traversing|traffic::Phase::ExitClearing|traffic::Phase::ExitBlocked)}
fn command_distance(g:&CostGrid,u:&Unit,here:Vec2)->Option<f32> {
    let field=u.guide.as_deref()?;let cell=Cell::new(here.x.round() as i32,here.y.round() as i32);
    if !field.valid_for(g)||!field.in_(cell.x,cell.y) {return None;}
    let distance=field.dist_at(cell.x,cell.y);(distance.is_finite()&&distance<1e29).then_some(distance)
}
fn local_recovery_goal(u:&Unit,traffic:&Traffic,goal:Vec2,q:&mut Query<'_>)->Vec2 {
    let g=q.grid;
    let here=formation::position(u);if here.distance(goal)<=8.0 {return goal;}
    let decision=traffic.decision(u.idx);
    let field=if decision.phase==traffic::Phase::Assembling {
        u.command_id.and_then(|c|traffic.assembly_guide(c.0))
    }else if decision.phase==traffic::Phase::Free {u.guide.as_deref()}else{None};
    let Some(field)=field else{return goal;};
    let radius=u.parking.as_ref().map_or(0.45,|t|t.radius);
    let mut cell=Cell::new(here.x.round() as i32,here.y.round() as i32);
    let mut previous=here;let mut distance=0.0;let mut clear_goal=None;
    // Reconstruct a short prefix of an existing static route, rather than
    // repeatedly searching an across-wall box that excludes the known detour.
    for _ in 0..16 {
        let Some(next)=nav::flow_step(g,field,cell) else{break;};let p=point(next);
        if !collision::disc_segment_clear(g,previous,p,radius) {break;}
        distance+=previous.distance(p);previous=p;cell=next;
        if q.clear(p,p,true) {clear_goal=Some(p);if distance>=6.0 {return p;}}
        if distance>=12.0 {break;}
    }
    clear_goal.filter(|p|here.distance(*p)>=1.0).unwrap_or(goal)
}
fn lease_near(leases:&[Body],owner:i32,from:Vec2,goal:Vec2,padding:f32,radius:f32)->bool {
    let low=from.min(goal)-Vec2::splat(padding+radius);
    let high=from.max(goal)+Vec2::splat(padding+radius);
    leases.iter().any(|b|b.id!=owner&&b.position.x+b.radius>=low.x&&b.position.x-b.radius<=high.x
        &&b.position.y+b.radius>=low.y&&b.position.y-b.radius<=high.y)
}
fn search_job(g:&CostGrid,here:Vec2,goal:Vec2,s:&State,has_leases:bool,q:&mut Query<'_>)->Option<Job> {
    let step=formation::PARKING_SPACING*0.5;
    let low=((here.min(goal)-Vec2::splat(s.padding)).max(Vec2::splat(-0.5))-goal)/step;
    let high=((here.max(goal)+Vec2::splat(s.padding)).min(Vec2::new(g.w as f32-0.5,g.h as f32-0.5))-goal)/step;
    let low=low.floor();let high=high.ceil();let origin=goal+low*step;let w=(high.x-low.x) as i32+1;let h=(high.y-low.y) as i32+1;
    let mut grid=CostGrid{w,h,cost:vec![0;(w*h) as usize],cut_corners:true};
    for y in 0..h {for x in 0..w {let p=origin+Vec2::new(x as f32,y as f32)*step;
        if q.clear(p,p,true) {grid.cost[(y*w+x) as usize]=g.at(p.x.round() as i32,p.y.round() as i32) as u8;}}}
    let center=(here-origin)/step;let mut starts=Vec::new();
    for y in center.y.round() as i32-3..=center.y.round() as i32+3 {for x in center.x.round() as i32-3..=center.x.round() as i32+3 {
        let cell=Cell::new(x,y);let p=origin+point(cell)*step;if grid.walkable(x,y)&&q.clear(here,p,true) {starts.push((here.distance_squared(p),cell));}
    }}
    starts.sort_by(|a,b|a.0.total_cmp(&b.0).then(a.1.y.cmp(&b.1.y)).then(a.1.x.cmp(&b.1.x)));
    if starts.is_empty() {return None;}
    let finish=(goal-origin)/step;let starts=starts.into_iter().map(|(_,c)|c).collect::<Vec<_>>();
    let search=PathSearch::multi(&grid,&starts,&[Cell::new(finish.x.round() as i32,finish.y.round() as i32)]);
    Some(Job{command:s.command,target:s.target,goal,epoch:s.epoch,grid,origin,step,search,probe:s.probe,has_leases})
}
impl Recovery {
    /// Target allocation uses this same age-preserving coordinator. One command
    /// may rebuild per tick; each failed command has a half-second cooldown.
    pub fn target_repair_turn(&mut self,invalid:&BTreeSet<u64>,tick:u64,dt:f32)->Option<u64> {
        self.repair_queue.retain(|c|invalid.contains(c));self.repair_after.retain(|c,_|invalid.contains(c));
        for &command in invalid {if !self.repair_queue.contains(&command) {self.repair_queue.push_back(command);}}
        if self.repair_tick==Some(tick) {return None;}let now=tick as f64*dt as f64;
        let count=self.repair_queue.len();for _ in 0..count {let c=self.repair_queue.pop_front().unwrap();self.repair_queue.push_back(c);
            if self.repair_after.get(&c).is_none_or(|after|now>=*after) {self.repair_after.insert(c,now+0.5);self.repair_tick=Some(tick);return Some(c);}}
        None
    }
    pub fn phase(&self,id:i32)->Phase {self.states.get(&id).map_or(Phase::Monitoring,|s|s.phase)}
    pub fn task_goal(&self,id:i32)->Option<Vec2> {
        let task=self.states.get(&id)?.task.as_ref()?;task.points.get(task.cursor).copied()
    }
    pub fn no_progress(&self,id:i32,now:f64)->f64 {self.states.get(&id).map_or(0.0,|s|(now-s.last_progress).max(0.0))}
    pub fn preferred(&self,id:i32,here:Vec2,speed:f32,dt:f32)->Option<(Vec2,f32)> {
        let task=self.states.get(&id)?.task.as_ref()?;let goal=*task.points.get(task.cursor)?;
        let remaining=here.distance(goal)+task.points[task.cursor..].windows(2).map(|p|p[0].distance(p[1])).sum::<f32>();
        let a=(speed*6.0).max(0.001);let d=(remaining-0.04).max(0.0);
        let cap=((a*a*dt*dt+2.0*a*d).sqrt()-a*dt).max(0.0).min(d/dt).min(speed);
        Some(((goal-here).normalize_or_zero(),cap))
    }
    fn refresh(&mut self,g:&CostGrid,units:&[Unit]) {
        self.hard.clear();self.fixed.clear();self.leases.clear();self.soft.clear();let mut signature=vec![g.w as u32,g.h as u32,u32::from(g.cut_corners)];
        signature.extend(g.cost.iter().map(|c|*c as u32));
        let mut lease_signature=Vec::new();
        for u in units.iter().filter(|u|u.on_ground) {
            let mut body=formation::body(u,u.parking.as_ref().map_or(0.45,|t|t.radius));
            let confirmed_hold=u.traffic_hold&&u.vx.hypot(u.vy)<=arrival::QUIET_SPEED;
            if u.goal.x<0||u.arrival.held()||confirmed_hold {signature.extend([u.idx as u32,body.position.x.to_bits(),body.position.y.to_bits(),body.radius.to_bits()]);self.fixed.push(body);self.hard.push(body);}
            else if u.no_progress_seconds>=DIAGNOSE_SECONDS&&u.vx.hypot(u.vy)<=arrival::QUIET_SPEED {
                // Renew a local occupancy observation, never permanent reachability.
                // The same progress distance rejects jitter as a version change.
                lease_signature.extend([u.idx,(body.position.x/0.20).round() as i32,(body.position.y/0.20).round() as i32]);
                self.leases.push(body);self.hard.push(body);
            }
            else {body.position+=Vec2::new(u.vx,u.vy)*0.5;self.soft.push(body);}
        }
        if signature!=self.signature {self.signature=signature;self.epoch+=1;
            self.fixed_index.rebuild(&self.fixed.iter().map(|b|Aabb::point(b.position)).collect::<Vec<_>>());
        }
        if lease_signature!=self.lease_signature {self.lease_signature=lease_signature;self.lease_epoch+=1;}
        self.hard_index.rebuild(&self.hard.iter().map(|b|Aabb::point(b.position)).collect::<Vec<_>>());
        // Moving predictions add costs only. Stalled mobile reservations are
        // rebuilt separately and never change permanent reachability versions.
        self.soft_index.rebuild(&self.soft.iter().map(|b|Aabb::point(b.position)).collect::<Vec<_>>());
    }
    pub fn update(&mut self,g:&CostGrid,units:&mut [Unit],traffic:&Traffic,tick:u64,dt:f32,budget:&mut nav::ReplanBudget) {
        budget.begin(tick);self.renewals.clear();let now=tick as f64*dt as f64;self.refresh(g,units);
        let old=&self.report;self.report=Report{diagnosed:old.diagnosed,stale_results:old.stale_results,replans:old.replans,..Default::default()};
        let mut group_progress=BTreeMap::<u64,f64>::new();
        for s in self.states.values() {group_progress.entry(s.command).and_modify(|t|*t=t.max(s.last_progress)).or_insert(s.last_progress);}
        let live:BTreeSet<_>=units.iter().filter(|u|u.command_id.is_some()&&u.parking.is_some()&&!u.arrival.held()).map(|u|u.idx).collect();
        self.states.retain(|id,_|live.contains(id));
        for u in units.iter_mut().filter(|u|live.contains(&u.idx)) {
            let target=u.parking.as_ref().unwrap();let here=formation::position(u);let command=u.command_id.unwrap().0;
            let decision=traffic.decision(u.idx);let goal=decision.point.unwrap_or(target.position);
            let version_changed=self.states.get(&u.idx).is_none_or(|s|s.command!=command||s.target!=target.position);
            if version_changed {
                let previous=self.states.get(&u.idx).filter(|s|s.command==command);
                let last_progress=previous.map_or(now,|s|s.last_progress);let queued_since=previous.map_or(f64::NAN,|s|s.queued_since);
                let diagnosed=previous.is_some_and(|s|s.diagnosed);
                self.states.insert(u.idx,State{command,target:target.position,anchor:here,best:f32::INFINITY,
                metric_key:(0,0,0),last_progress,phase:Phase::Monitoring,stage:0,side:1.0,attempts:0,next_try:now,
                diagnosed,queued_since,task:None,epoch:self.epoch,lease_epoch:self.lease_epoch,padding:6.0,probe:false});}
            let s=self.states.get_mut(&u.idx).unwrap();
            if s.epoch!=self.epoch {s.epoch=self.epoch;s.probe=false;
                if s.phase==Phase::Blocked {s.attempts=0;}
                if matches!(s.phase,Phase::NoRoute|Phase::Searching|Phase::Following|Phase::Blocked) {
                s.task=None;s.phase=Phase::Queued;if !s.queued_since.is_finite() {s.queued_since=now;}s.next_try=now;
            }}
            if s.lease_epoch!=self.lease_epoch {
                if s.phase!=Phase::Blocked {s.lease_epoch=self.lease_epoch;}
                else if now>=s.next_try {s.lease_epoch=self.lease_epoch;s.phase=Phase::Queued;s.attempts=0;s.probe=false;s.next_try=now+0.5;
                    if !s.queued_since.is_finite() {s.queued_since=now;}
                    self.events.push(Event{tick,unit:u.idx,command,kind:"reservation-review"});
                }
            }
            if s.phase==Phase::NoRoute {
                self.report.max_no_progress=self.report.max_no_progress.max(now-s.last_progress);continue;
            }
            let budget_wait=s.task.is_none()&&(target.blocked==Some(formation::TargetBlock::SearchBudget)||traffic.assembly_pending(u.idx,command));
            let parking_wait=s.task.is_none()&&group_progress.get(&command).is_some_and(|last|now-*last<DIAGNOSE_SECONDS)
                &&(target.blocked==Some(formation::TargetBlock::Entrance)
                    ||(here.distance(target.position)<=formation::PARKING_SPACING&&target.blocked!=Some(formation::TargetBlock::NoLocalPath)));
            let coherent_wait=decision.phase==traffic::Phase::FinalApproach&&traffic.coordination_has_progress(command,tick,dt);
            if gate_owns(decision.phase)||coherent_wait||decision.cap==Some(0.0)||budget_wait||parking_wait {
                s.task=None;s.phase=Phase::ScheduledWait;self.report.scheduled_wait+=1;
                self.report.max_no_progress=self.report.max_no_progress.max(now-s.last_progress);continue;
            }
            if s.phase==Phase::ScheduledWait {s.phase=Phase::Monitoring;s.next_try=s.next_try.max(now+DIAGNOSE_SECONDS);}
            if !target.valid {if s.phase!=Phase::WaitingForSpace {self.renewals.push((u.idx,command));
                self.events.push(Event{tick,unit:u.idx,command,kind:"target-space-request"});}
                s.task=None;s.phase=Phase::WaitingForSpace;self.report.waiting_space+=1;continue;}
            if s.phase==Phase::WaitingForSpace {s.phase=Phase::Monitoring;}
            if let Some(task)=&mut s.task {
                while task.cursor<task.points.len()&&here.distance(task.points[task.cursor])<=0.12 {task.cursor+=1;}
                if task.cursor==task.points.len()||task.deadline.is_some_and(|end|now>=end) {
                    s.task=None;s.phase=Phase::Cooldown;s.next_try=now+0.25;
                    self.events.push(Event{tick,unit:u.idx,command,kind:"task-finished"});
                }
            }
            let (metric,key)=if let Some(task)=&s.task {
                let remaining=here.distance(task.points[task.cursor])+task.points[task.cursor..].windows(2).map(|p|p[0].distance(p[1])).sum::<f32>();
                // A replacement task has a new distance origin. Reset that
                // reference, while preserving age until actual progress occurs.
                let generation=task.began.to_bits();
                (remaining,(generation as u32,(generation>>32) as u32,1))
            }else{
                let (metric,kind)=traffic.progress_metric(u.idx,command,here,target.position,g).unwrap_or_else(||{
                    if decision.phase==traffic::Phase::Free&&here.distance(target.position)>target.area_radius+formation::LOCAL_APPROACH_MARGIN {
                        if let Some(distance)=command_distance(g,u,here) {return (distance,6);}
                    }
                    if decision.phase==traffic::Phase::FinalApproach {(here.distance(target.position),4)}else{
                        let remaining=target.remaining_distance(here,goal);(if remaining.is_finite() {remaining}else{here.distance(goal)},0)
                    }
                });
                (metric,(target.position.x.to_bits(),target.position.y.to_bits(),kind))
            };
            if key!=s.metric_key {s.metric_key=key;s.best=metric;s.anchor=here;}
            if metric+0.20<s.best&&here.distance(s.anchor)>0.20 {s.best=metric;s.anchor=here;s.last_progress=now;s.diagnosed=false;
                if matches!(s.phase,Phase::Queued|Phase::Searching|Phase::Blocked) {
                    s.phase=Phase::Monitoring;s.queued_since=f64::NAN;s.stage=0;s.attempts=0;s.next_try=now+0.25;
                    self.events.push(Event{tick,unit:u.idx,command,kind:"progress-resumed"});
                }}
            let quiet=target.frozen&&target.blocked.is_none()&&here.distance(target.position)<=arrival::ENTER_DISTANCE&&u.vx.hypot(u.vy)<=arrival::QUIET_SPEED;
            let stalled=now-s.last_progress;self.report.max_no_progress=self.report.max_no_progress.max(stalled);
            let stuck_path=s.task.as_ref().is_some_and(|t|t.kind==Phase::Following&&now-t.began>=DIAGNOSE_SECONDS&&stalled>=DIAGNOSE_SECONDS);
            if stuck_path {s.task=None;s.phase=if s.attempts>=8 {Phase::Blocked}else{Phase::Cooldown};s.stage=1;s.next_try=now+0.25;
                self.events.push(Event{tick,unit:u.idx,command,kind:if s.phase==Phase::Blocked {"motion-blocked"}else{"path-stalled"}});}
            if quiet||s.task.is_some()||matches!(s.phase,Phase::Queued|Phase::Searching|Phase::NoRoute|Phase::Blocked)||now<s.next_try||stalled<DIAGNOSE_SECONDS {continue;}
            if !s.diagnosed {s.diagnosed=true;self.report.diagnosed+=1;self.events.push(Event{tick,unit:u.idx,command,kind:"no-progress"});}
            let radius=target.radius;let mut q=Query{grid:g,bodies:&self.hard,index:&mut self.hard_index,neighbors:&mut self.hard_neighbors,
                owner:u.idx,radius,max_radius:self.hard.iter().map(|b|b.radius).fold(radius,f32::max)};
            let recovery_goal=local_recovery_goal(u,traffic,goal,&mut q);
            let forward=(recovery_goal-here).normalize_or_zero();let right=Vec2::new(forward.y,-forward.x);
            let candidates=if s.stage==0 {vec![(here+right*1.6,1.0),(here-right*1.6,-1.0)]}
                else if s.stage==1 {vec![(here+right*s.side*2.5+forward, s.side),(here+right*s.side*3.0-forward, s.side),(here-forward*2.0,s.side)]}else{Vec::new()};
            if let Some((p,side))=candidates.into_iter().find(|(p,_)|q.clear(here,*p,true)) {
                s.side=side;let kind=if s.stage==0 {Phase::Yielding}else{Phase::Pocket};s.stage+=1;s.phase=kind;
                s.task=Some(Task{points:vec![p],cursor:0,deadline:Some(now+0.75),kind,began:now});
                self.events.push(Event{tick,unit:u.idx,command,kind:if kind==Phase::Yielding {"yield-start"}else{"pocket-start"}});
            }else if s.stage==0 {s.stage=1;s.phase=Phase::Cooldown;s.next_try=now+0.1;}
            else{s.stage=2;s.phase=Phase::Queued;if !s.queued_since.is_finite() {s.queued_since=now;}self.events.push(Event{tick,unit:u.idx,command,kind:"replan-queued"});}
        }
        let jobs_before=self.jobs.len();
        self.jobs.retain(|id,j|self.states.get(id).is_some_and(|s|s.command==j.command&&s.target==j.target&&s.epoch==j.epoch&&s.phase==Phase::Searching));
        self.report.stale_results+=(jobs_before-self.jobs.len()) as u64;
        self.running.retain(|id|self.jobs.contains_key(id));
        self.queue.retain(|id|self.states.get(id).is_some_and(|s|s.phase==Phase::Queued));
        let mut requests:Vec<_>=self.states.iter().filter(|(id,s)|s.phase==Phase::Queued&&!self.queue.contains(id))
            .map(|(&id,s)|(s.queued_since,s.command,id)).collect();
        requests.sort_by(|a,b|a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        self.queue.extend(requests.into_iter().map(|r|r.2));
        let admissions=self.queue.len().min(MAX_JOBS.saturating_sub(self.jobs.len()));
        for _ in 0..admissions {let Some(id)=self.queue.pop_front() else{break;};
            let u=&units[units.binary_search_by_key(&id,|u|u.idx).unwrap()];let s=self.states.get_mut(&id).unwrap();
            let wanted=traffic.decision(id).point.unwrap_or(s.target);
            let here=formation::position(u);let radius=u.parking.as_ref().unwrap().radius;
            let (bodies,index,neighbors)=if s.probe {(&self.fixed,&mut self.fixed_index,&mut self.fixed_neighbors)}
                else{(&self.hard,&mut self.hard_index,&mut self.hard_neighbors)};
            let mut q=Query{grid:g,bodies,index,neighbors,owner:id,radius,
                max_radius:bodies.iter().map(|b|b.radius).fold(radius,f32::max)};
            let goal=local_recovery_goal(u,traffic,wanted,&mut q);
            let has_leases=lease_near(&self.leases,id,here,goal,s.padding,radius);
            if let Some(job)=search_job(g,here,goal,s,has_leases,&mut q) {
                self.jobs.insert(id,job);self.running.push_back(id);s.phase=Phase::Searching;self.report.replans+=1;
                self.events.push(Event{tick,unit:id,command:s.command,kind:"search-start"});
            }else if has_leases&&!s.probe {s.probe=true;s.phase=Phase::Queued;
                self.events.push(Event{tick,unit:id,command:s.command,kind:"reservation-entry-probe"});
            }else{s.probe=false;s.phase=Phase::NoRoute;self.renewals.push((id,s.command));self.events.push(Event{tick,unit:id,command:s.command,kind:"no-grid-entry"});}
        }
        let jobs_this_tick=self.running.len();
        for _ in 0..jobs_this_tick {let Some(id)=self.running.pop_front() else{break;};let job=self.jobs.get_mut(&id).unwrap();
            let request=nav::SearchKey{owner:id,command:job.command,x:job.goal.x.to_bits(),y:job.goal.y.to_bits(),class:1};
            let permit=budget.request(request,tick);if permit==0 {self.running.push_back(id);continue;}
            let u=&units[units.binary_search_by_key(&id,|u|u.idx).unwrap()];let radius=u.parking.as_ref().unwrap().radius;
            let (bodies,index,neighbors)=if job.probe {(&self.fixed,&mut self.fixed_index,&mut self.fixed_neighbors)}
                else{(&self.hard,&mut self.hard_index,&mut self.hard_neighbors)};
            let mut q=Query{grid:g,bodies,index,neighbors,owner:id,radius,
                max_radius:bodies.iter().map(|b|b.radius).fold(radius,f32::max)};
            job.has_leases|=lease_near(&self.leases,id,formation::position(u),job.goal,self.states[&id].padding,radius);
            let soft=&self.soft;let soft_index=&mut self.soft_index;let neighbors=&mut self.soft_neighbors;
            let before=job.search.expanded;let origin=job.origin;let step=job.step;
            let status=job.search.advance(&job.grid,permit,&|_,_|false,&mut |a,b|{
                let pa=origin+point(a)*step;let pb=origin+point(b)*step;if !q.clear(pa,pb,true) {return None;}
                soft_index.query_radius(pb,radius*2.0+0.1,neighbors);
                Some(neighbors.iter().filter(|i|soft[**i].id!=id&&soft[**i].position.distance(pb)<soft[**i].radius+radius).count() as f32*0.5)
            });let used=job.search.expanded-before;budget.record(used);self.report.expanded+=used;drop(q);
            let s=self.states.get_mut(&id).unwrap();
            match status {
                SearchStatus::Pending=>self.running.push_back(id),
                SearchStatus::Found=>{let mut points:Vec<_>=job.search.cells.iter().map(|c|job.origin+point(*c)*job.step).collect();points.push(job.goal);
                    let mut live=Query{grid:g,bodies:&self.hard,index:&mut self.hard_index,neighbors:&mut self.hard_neighbors,owner:id,radius,
                        max_radius:self.hard.iter().map(|b|b.radius).fold(radius,f32::max)};
                    let here=formation::position(u);let first=points.iter().position(|p|live.clear(here,*p,true));
                    let legal=first.is_some_and(|i|points[i..].windows(2).all(|e|live.clear(e[0],e[1],true)));
                    if legal {points.drain(..first.unwrap());s.task=Some(Task{points,cursor:0,deadline:None,kind:Phase::Following,began:now});s.phase=Phase::Following;s.attempts+=1;s.probe=false;
                        self.events.push(Event{tick,unit:id,command:s.command,kind:"path-ready"});budget.finish(request,true,0);
                    }else if job.probe {s.probe=false;s.phase=Phase::Cooldown;s.stage=2;s.padding=6.0;s.next_try=now+0.5;
                        self.renewals.push((id,s.command));
                        self.events.push(Event{tick,unit:id,command:s.command,kind:"reservation-wait"});budget.finish(request,false,30);
                    }else{s.probe=false;s.phase=Phase::Queued;self.report.stale_results+=1;
                        self.events.push(Event{tick,unit:id,command:s.command,kind:"stale-start"});budget.finish(request,false,0);}
                    self.jobs.remove(&id);}
                SearchStatus::NoPath|SearchStatus::Invalid=>{let probe=job.probe;let has_leases=job.has_leases;
                    self.jobs.remove(&id);budget.finish(request,false,0);s.attempts+=1;
                    if !probe&&has_leases {s.probe=true;s.phase=Phase::Queued;
                        self.events.push(Event{tick,unit:id,command:s.command,kind:"reservation-route-probe"});
                    }else if s.padding<(g.w.max(g.h) as f32) {s.padding=(s.padding*2.0).min(g.w.max(g.h) as f32);s.phase=Phase::Queued;
                        self.events.push(Event{tick,unit:id,command:s.command,kind:"expand-local-region"});}
                    else{s.probe=false;s.phase=Phase::NoRoute;self.renewals.push((id,s.command));self.events.push(Event{tick,unit:id,command:s.command,kind:"no-grid-route"});}
                }
            }
        }
        for s in self.states.values() {match s.phase {
            Phase::Yielding|Phase::Pocket=>self.report.yielding+=1,Phase::Queued=>{self.report.queued+=1;self.report.max_queue_age=self.report.max_queue_age.max(now-s.queued_since);},
            Phase::Searching=>{self.report.searching+=1;self.report.max_queue_age=self.report.max_queue_age.max(now-s.queued_since);},
            Phase::Following=>self.report.following+=1,Phase::NoRoute=>self.report.no_route+=1,Phase::Blocked=>self.report.blocked+=1,_=>{}
        }}
        self.report.budget_pending=budget.pending();
        for u in units {u.recovery_phase=self.phase(u.idx);u.no_progress_seconds=self.no_progress(u.idx,now);}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::local_nav::Query;
    use bevy::prelude::*;
    use crate::{nav_scenario,play::{self,Selection},world::SimEnv};
    fn world(grid:CostGrid)->World {
        let mut w=World::new();let env=SimEnv{nav:grid,..Default::default()};let step=env.runtime.fixed_step();
        let mut time=Time::<Fixed>::from_duration(step);time.advance_by(step);w.insert_resource(time);w.insert_resource(env);w.insert_resource(Selection::default());w
    }
    fn safe(w:&World) {
        let env=w.resource::<SimEnv>();let c=&env.runtime.contacts;
        assert!(c.max_penetration<=0.02&&c.max_swept_penetration<=0.02);
        assert_eq!((c.initial_overlaps,c.illegal_static_trajectories,c.unresolved_pairs),(0,0,0));
        assert!(env.recovery.report.expanded<=EXPANSIONS_PER_TICK);
    }
    #[test]
    fn stationary_blocking_starts_effective_recovery_within_two_seconds_and_finishes() {
        let mut w=world(CostGrid{w:48,h:32,cost:vec![1;48*32],..Default::default()});
        let mover=w.spawn(Unit{idx:0,cell_x:12.0,cell_y:16.0,..Default::default()}).id();
        for (i,y) in [14.9,16.0,17.1].into_iter().enumerate() {w.spawn(Unit{idx:10+i as i32,cell_x:13.0,cell_y:y,..Default::default()});}
        assert_eq!(play::issue_orders_at(&mut w,&[(vec![mover],Cell::new(44,16))])[0].accepted,1);
        let command=w.get::<Unit>(mover).unwrap().command_id;let mut schedule=nav_scenario::schedule();let mut first=None;let mut moved=false;let mut done=None;
        for tick in 1..=1800 {schedule.run(&mut w);safe(&w);let env=w.resource::<SimEnv>();let u=w.get::<Unit>(mover).unwrap();
            assert_eq!(u.command_id,command);assert_eq!(u.start_delay,0.0);
            if first.is_none()&&env.recovery.events.iter().any(|e|e.unit==0&&matches!(e.kind,"yield-start"|"pocket-start"|"search-start")) {first=Some(tick);}
            if tick<=120&&(u.cell_y-16.0).abs()>0.2 {moved=true;}
            if u.arrival.settled() {done=Some(tick);break;}
        }
        if done.is_none() {println!("recovery_block_incomplete {:?} p={:?} events={:?}",w.get::<Unit>(mover).unwrap().recovery_phase,formation::position(w.get::<Unit>(mover).unwrap()),w.resource::<SimEnv>().recovery.events);}
        assert!(first.is_some_and(|t|t<=120),"no recovery within2 seconds: {first:?}");assert!(moved);assert!(done.is_some());
        for u in w.query::<&Unit>().iter(&w).filter(|u|u.idx>=10) {assert_eq!(u.cell_x,13.0);assert_eq!(u.vx.hypot(u.vy),0.0);}
        println!("recovery_block first_tick={} complete_tick={}",first.unwrap(),done.unwrap());
    }
    #[test]
    fn queued_recovery_is_withdrawn_only_after_real_progress() {
        for progressed in [false,true] {
            let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});
            let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
            play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);
            let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
            let command=units[0].command_id;let target=units[0].parking.as_ref().unwrap().position;
            env.traffic.decisions.insert(0,traffic::Decision{phase:traffic::Phase::FinalApproach,..Default::default()});
            env.recovery.update(&env.nav,&mut units,&env.traffic,0,1.0/60.0,&mut env.buffers.local_routes.budget);
            let state=env.recovery.states.get_mut(&0).unwrap();state.phase=Phase::Queued;state.stage=2;state.queued_since=0.0;
            // Recovery observes the displacement produced by the movement layer.
            units[0].cell_x+=if progressed {0.4}else{0.01};
            env.recovery.update(&env.nav,&mut units,&env.traffic,120,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert_eq!(units[0].command_id,command);assert_eq!(units[0].parking.as_ref().unwrap().position,target);
            assert!(!units[0].arrival.settled());
            if progressed {
                assert_eq!(units[0].recovery_phase,Phase::Monitoring);
                assert_eq!(env.recovery.report.queued+env.recovery.report.searching,0);
                assert!(units[0].no_progress_seconds<0.01);
            }else {
                assert_ne!(units[0].recovery_phase,Phase::Monitoring);
                assert!(units[0].no_progress_seconds>=1.99);
            }
        }
    }
    #[test]
    fn persistent_mobile_occupancy_is_a_short_reservation_and_releases_on_progress() {
        let g=CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()};
        let mut units=vec![Unit{idx:0,cell_x:5.0,cell_y:9.0,goal:Cell::new(25,9),..Default::default()},
            Unit{idx:1,cell_x:6.2,cell_y:9.0,goal:Cell::new(25,10),no_progress_seconds:1.2,..Default::default()}];
        let mut recovery=Recovery::default();recovery.refresh(&g,&units);let static_epoch=recovery.epoch;
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(!q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true),
            "a recovery route crossed a persistently stalled mobile peer");
        units[1].vx=6.0;units[1].no_progress_seconds=0.0;recovery.refresh(&g,&units);
        assert_eq!(recovery.epoch,static_epoch,"a short reservation changed permanent reachability");
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true));
        units[1].vx=0.0;units[1].no_progress_seconds=0.4;recovery.refresh(&g,&units);
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true),"an ordinary short pause became a reservation");
    }
    #[test]
    fn temporary_reservations_wait_for_space_without_claiming_static_no_route() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});
        let mover=w.spawn(Unit{idx:0,cell_x:9.0,cell_y:12.0,..Default::default()}).id();
        let peers=(0..24).map(|y|w.spawn(Unit{idx:y+1,cell_x:12.0,cell_y:y as f32,..Default::default()}).id()).collect::<Vec<_>>();
        play::issue_orders_at(&mut w,&[(vec![mover],Cell::new(17,12)),(peers.clone(),Cell::new(10,12))]);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=std::iter::once(mover).chain(peers).map(|e|w.get::<Unit>(e).unwrap().clone()).collect::<Vec<_>>();
        let command=units[0].command_id;let mut waited=false;
        for tick in 0..1800 {env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.recovery.report.expanded<=EXPANSIONS_PER_TICK);
            assert_ne!(units[0].recovery_phase,Phase::NoRoute,"a temporary mobile wall became permanent no-route");
            waited|=env.recovery.events.iter().any(|e|e.unit==0&&e.kind=="reservation-wait");
            if waited {break;}
        }
        assert!(waited,"temporary occupancy did not obtain a budgeted static reachability result");
        assert_eq!(units[0].command_id,command);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn temporary_reservations_preserve_a_genuine_static_no_route_result() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});
        let mover=w.spawn(Unit{idx:0,cell_x:9.0,cell_y:12.0,..Default::default()}).id();
        let peer=w.spawn(Unit{idx:1,cell_x:10.0,cell_y:15.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![mover],Cell::new(17,12)),(vec![peer],Cell::new(10,18))]);
        let mut env=w.remove_resource::<SimEnv>().unwrap();for y in 0..24 {env.nav.cost[y*30+12]=0;}
        let mut units=vec![w.get::<Unit>(mover).unwrap().clone(),w.get::<Unit>(peer).unwrap().clone()];
        units[1].no_progress_seconds=2.0;let command=units[0].command_id;
        for tick in 0..2000 {env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.recovery.report.expanded<=EXPANSIONS_PER_TICK);}
        assert_eq!(units[0].recovery_phase,Phase::NoRoute);assert!(units[0].no_progress_seconds>20.0);
        assert_eq!(units[0].command_id,command);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn a_completed_recovery_search_rejoins_the_current_physical_snapshot() {
        let mut g=CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()};
        for x in 5..=20 {g.cost[14*30+x]=0;}
        let mut w=world(g);let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        let command=units[0].command_id;let mut moved=false;let mut ready=false;
        for tick in 0..1600 {
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.recovery.report.expanded<=EXPANSIONS_PER_TICK);
            if !moved&&units[0].recovery_phase==Phase::Searching {
                units[0].cell_y=17.0;moved=true;
                assert!(collision::disc_clear(&env.nav,formation::position(&units[0]),0.45));
            }else if moved&&units[0].recovery_phase==Phase::Following {
                let task=env.recovery.states[&0].task.as_ref().unwrap();let here=formation::position(&units[0]);
                assert!(collision::disc_segment_clear(&env.nav,here,task.points[task.cursor],0.45),
                    "a completed recovery search accepted an unreachable old origin");ready=true;break;
            }
        }
        assert!(moved&&ready);assert_eq!(units[0].command_id,command);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn recovery_routes_avoid_confirmed_holds_and_release_them_on_readmission() {
        let g=CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()};
        let mut units=vec![
            Unit{idx:0,cell_x:5.0,cell_y:9.0,goal:Cell::new(25,9),..Default::default()},
            Unit{idx:1,cell_x:6.2,cell_y:9.0,goal:Cell::new(25,10),traffic_hold:true,..Default::default()},
            Unit{idx:2,cell_x:6.2,cell_y:12.0,goal:Cell::new(25,12),..Default::default()},
        ];
        let mut recovery=Recovery::default();recovery.refresh(&g,&units);
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(!q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true),
            "a recovery route crossed an explicitly held stationary unit");
        assert!(q.clear(Vec2::new(5.0,12.0),Vec2::new(7.4,12.0),true),
            "ordinary stationary mobile units became hard obstacles");
        let held_epoch=recovery.epoch;
        units[1].traffic_hold=false;recovery.refresh(&g,&units);
        assert!(recovery.epoch>held_epoch);
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true),
            "readmitted traffic retained a stale hard footprint");
        units[1].traffic_hold=true;units[1].vx=1.0;recovery.refresh(&g,&units);
        let mut q=Query{grid:&g,bodies:&recovery.hard,index:&mut recovery.hard_index,neighbors:&mut recovery.hard_neighbors,
            owner:0,radius:0.45,max_radius:0.45};
        assert!(q.clear(Vec2::new(5.0,9.0),Vec2::new(7.4,9.0),true),
            "a still-moving admission stop became a fixed obstacle");
    }
    #[test]
    fn changed_confirmed_occupancy_retries_blocked_motion_without_resetting_age() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});
        let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:9.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,9))]);
        let mut env=w.remove_resource::<SimEnv>().unwrap();
        let mut units=vec![w.get::<Unit>(e).unwrap().clone(),
            Unit{idx:1,cell_x:6.2,cell_y:9.0,goal:Cell::new(25,10),traffic_hold:true,..Default::default()}];
        let command=units[0].command_id;let target=units[0].parking.as_ref().unwrap().position;
        env.recovery.update(&env.nav,&mut units,&env.traffic,0,1.0/60.0,&mut env.buffers.local_routes.budget);
        let s=env.recovery.states.get_mut(&0).unwrap();s.phase=Phase::Blocked;s.stage=2;s.attempts=8;s.queued_since=0.5;
        units[1].traffic_hold=false;
        env.recovery.update(&env.nav,&mut units,&env.traffic,120,1.0/60.0,&mut env.buffers.local_routes.budget);
        assert!(matches!(units[0].recovery_phase,Phase::Queued|Phase::Searching),
            "changed occupancy left an obsolete motion failure in place");
        assert!(units[0].no_progress_seconds>=1.99);
        assert_eq!(units[0].command_id,command);assert_eq!(units[0].parking.as_ref().unwrap().position,target);
        assert!(!units[0].arrival.settled());
    }
    #[test]
    fn a_local_replan_reuses_the_static_detour_before_searching() {
        let mut g=CostGrid{w:192,h:160,cost:vec![1;192*160],..Default::default()};
        for y in 54..=106 {for x in 92..=98 {g.cost[y*192+x]=0;}}
        let mut w=world(g);let e=w.spawn(Unit{idx:0,cell_x:32.0,cell_y:80.0,..Default::default()}).id();
        assert_eq!(play::issue_orders_at(&mut w,&[(vec![e],Cell::new(160,80))])[0].accepted,1);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        assert!(units[0].guide.is_some());let command=units[0].command_id;let mut found=None;
        for tick in 0..=240 {
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(env.recovery.report.expanded<=EXPANSIONS_PER_TICK);
            if env.recovery.events.iter().any(|e|e.kind=="path-ready") {found=Some(tick);break;}
        }
        assert!(found.is_some(),"known static detour was repeatedly excluded from local searches");
        assert_eq!(units[0].command_id,command);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn a_static_route_can_progress_while_moving_away_from_the_final_target() {
        let mut g=CostGrid{w:192,h:160,cost:vec![1;192*160],..Default::default()};
        for y in 54..=106 {for x in 92..=98 {g.cost[y*192+x]=0;}}
        let mut w=world(g);let e=w.spawn(Unit{idx:0,cell_x:91.0,cell_y:80.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(160,80))]);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        let goal=units[0].parking.as_ref().unwrap().position;let here=formation::position(&units[0]);
        let hint={let mut index=Index::default();let mut neighbors=Vec::new();
            let mut q=Query{grid:&env.nav,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:0,radius:0.45,max_radius:0.45};
            local_recovery_goal(&units[0],&env.traffic,goal,&mut q)};
        assert!(hint.distance(here)<=8.0&&hint.distance(here)>=4.0);
        assert!(hint.distance(goal)>here.distance(goal));
        let start_distance=command_distance(&env.nav,&units[0],here).unwrap();
        assert!(command_distance(&env.nav,&units[0],hint).unwrap()<start_distance);
        for tick in 0..=60 {
            let p=here.lerp(hint,tick as f32/60.0);
            assert!(collision::disc_segment_clear(&env.nav,here,p,units[0].parking.as_ref().unwrap().radius));
            units[0].cell_x=p.x;units[0].cell_y=p.y;
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
        }
        assert_eq!(env.recovery.report.diagnosed,0);assert!(units[0].no_progress_seconds<0.2);
    }
    #[test]
    fn a_local_route_hint_does_not_end_inside_a_stopped_body() {
        let mut w=world(CostGrid{w:192,h:160,cost:vec![1;192*160],..Default::default()});
        let e=w.spawn(Unit{idx:0,cell_x:32.0,cell_y:80.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(160,80))]);
        let env=w.remove_resource::<SimEnv>().unwrap();let u=w.get::<Unit>(e).unwrap();let goal=u.parking.as_ref().unwrap().position;
        let first={let mut index=Index::default();let mut neighbors=Vec::new();
            let mut q=Query{grid:&env.nav,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:0,radius:0.45,max_radius:0.45};
            local_recovery_goal(u,&env.traffic,goal,&mut q)};
        let bodies=vec![Body{id:99,position:first,radius:0.45,departing:false,idle:true}];
        let mut index=Index::default();index.rebuild(&[Aabb::point(first)]);let mut neighbors=Vec::new();
        let mut q=Query{grid:&env.nav,bodies:&bodies,index:&mut index,neighbors:&mut neighbors,owner:0,radius:0.45,max_radius:0.45};
        let hint=local_recovery_goal(u,&env.traffic,goal,&mut q);
        assert!(hint.distance(first)>=0.89998);assert!(q.clear(hint,hint,true));
        assert!(hint.distance(formation::position(u))<=12.0);
    }
    #[test]
    fn oscillation_without_net_route_progress_is_diagnosed_and_keeps_its_age() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        for tick in 0..180 {units[0].cell_x=5.0+if tick%2==0 {0.01}else{-0.01};if tick==90 {units[0].parking.as_mut().unwrap().position.x+=0.55;}
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);}
        assert!(env.recovery.report.diagnosed>0);assert!(units[0].no_progress_seconds>2.9);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn a_replacement_path_measures_progress_from_its_own_start() {
        for progressed in [true,false] {
            let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});
            let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
            play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);
            let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
            let command=units[0].command_id;let target=units[0].parking.as_ref().unwrap().position;
            env.recovery.update(&env.nav,&mut units,&env.traffic,0,1.0/60.0,&mut env.buffers.local_routes.budget);
            let state=env.recovery.states.get_mut(&0).unwrap();state.phase=Phase::Pocket;
            state.task=Some(Task{points:vec![Vec2::new(5.2,12.0)],cursor:0,deadline:None,kind:Phase::Pocket,began:0.1});
            env.recovery.update(&env.nav,&mut units,&env.traffic,6,1.0/60.0,&mut env.buffers.local_routes.budget);
            let state=env.recovery.states.get_mut(&0).unwrap();state.phase=Phase::Following;
            state.task=Some(Task{points:vec![Vec2::new(12.0,12.0)],cursor:0,deadline:None,kind:Phase::Following,began:0.2});
            env.recovery.update(&env.nav,&mut units,&env.traffic,12,1.0/60.0,&mut env.buffers.local_routes.budget);
            assert!(units[0].no_progress_seconds>=0.19,"replacing a task must not itself reset waiting age");
            for tick in 13..=90 {
                // Feed the monitor observed movement along the replacement path.
                units[0].cell_x=5.0+if progressed {(tick-12) as f32*0.04}else if tick%2==0 {0.01}else{-0.01};
                env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
            }
            assert_eq!(units[0].command_id,command);assert_eq!(units[0].parking.as_ref().unwrap().position,target);
            assert!(!units[0].arrival.settled());
            let stalled=env.recovery.events.iter().any(|e|e.kind=="path-stalled");
            if progressed {
                assert!(!stalled,"real progress inherited the previous short task's distance minimum");
                assert_eq!(units[0].recovery_phase,Phase::Following);assert!(units[0].no_progress_seconds<0.2);
            }else{assert!(stalled);assert!(units[0].no_progress_seconds>=1.49);}
        }
    }
    #[test]
    fn a_following_path_that_stops_progressing_escalates_without_resetting_age() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);let mut schedule=nav_scenario::schedule();schedule.run(&mut w);
        let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        let state=env.recovery.states.get_mut(&0).unwrap();state.phase=Phase::Following;
        state.task=Some(Task{points:vec![Vec2::new(9.0,12.0)],cursor:0,deadline:None,kind:Phase::Following,began:0.0});
        env.recovery.update(&env.nav,&mut units,&env.traffic,120,1.0/60.0,&mut env.buffers.local_routes.budget);
        assert!(env.recovery.events.iter().any(|e|e.kind=="path-stalled"));assert!(units[0].no_progress_seconds>=1.99);
        assert_ne!(units[0].recovery_phase,Phase::Following);assert!(!units[0].arrival.settled());
    }
    #[test]
    fn a_moving_approach_reference_does_not_hide_real_route_progress() {
        let mut w=world(CostGrid{w:40,h:24,cost:vec![1;40*24],..Default::default()});let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(35,12))]);let mut env=w.remove_resource::<SimEnv>().unwrap();let mut units=vec![w.get::<Unit>(e).unwrap().clone()];
        for tick in 0..180 {units[0].cell_x=5.0+tick as f32*0.04;
            env.traffic.decisions.insert(0,traffic::Decision{point:Some(formation::position(&units[0])+Vec2::X*0.5),phase:traffic::Phase::FinalApproach,..Default::default()});
            env.recovery.update(&env.nav,&mut units,&env.traffic,tick,1.0/60.0,&mut env.buffers.local_routes.budget);
        }
        assert_eq!(env.recovery.report.diagnosed,0);assert!(units[0].no_progress_seconds<0.2);
    }
    #[test]
    fn a_queued_old_command_result_cannot_replace_a_new_order() {
        let mut w=world(CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()});let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(25,12))]);let mut schedule=nav_scenario::schedule();schedule.run(&mut w);
        {let mut env=w.resource_mut::<SimEnv>();let grid=env.nav.clone();let state=env.recovery.states.get_mut(&0).unwrap();state.phase=Phase::Searching;
            let job=Job{command:state.command,target:state.target,goal:state.target,epoch:state.epoch,origin:Vec2::ZERO,step:1.0,
                search:PathSearch::new(&grid,Cell::new(5,12),Cell::new(25,12)),grid,probe:false,has_leases:false};env.recovery.jobs.insert(0,job);env.recovery.running.push_back(0);}
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(8,12))]);let command=w.get::<Unit>(e).unwrap().command_id;
        schedule.run(&mut w);safe(&w);assert!(w.resource::<SimEnv>().recovery.report.stale_results>=1);
        for _ in 0..600 {schedule.run(&mut w);safe(&w);assert_eq!(w.get::<Unit>(e).unwrap().command_id,command);}
        assert!(w.get::<Unit>(e).unwrap().arrival.settled());assert!(w.get::<Unit>(e).unwrap().cell_x<8.2);
    }
    #[test]
    fn newly_closed_route_reports_no_route_without_completing_or_refreshing_wait() {
        let mut w=world(CostGrid{w:20,h:16,cost:vec![1;20*16],..Default::default()});let e=w.spawn(Unit{idx:0,cell_x:3.0,cell_y:8.0,..Default::default()}).id();
        play::issue_orders_at(&mut w,&[(vec![e],Cell::new(17,8))]);let command=w.get::<Unit>(e).unwrap().command_id;
        for y in 0..16 {w.resource_mut::<SimEnv>().nav.cost[y*20+10]=0;}
        let mut schedule=nav_scenario::schedule();let mut observed=None;
        for tick in 1..=1200 {schedule.run(&mut w);safe(&w);let u=w.get::<Unit>(e).unwrap();assert_eq!(u.command_id,command);assert!(!u.arrival.settled());
            if u.recovery_phase==Phase::NoRoute {observed=Some(tick);break;}
        }
        assert!(observed.is_some(),"no explicit no-route outcome");let age=w.get::<Unit>(e).unwrap().no_progress_seconds;
        for _ in 0..120 {schedule.run(&mut w);safe(&w);}
        let u=w.get::<Unit>(e).unwrap();
        assert_eq!(u.recovery_phase,Phase::NoRoute,"first={observed:?}, position={:?}, age={}, events={:?}",
            formation::position(u),u.no_progress_seconds,w.resource::<SimEnv>().recovery.events.iter().rev().take(20).collect::<Vec<_>>());
        assert!(u.no_progress_seconds>=age+1.9);assert_eq!(u.arrival.command,arrival::CommandStatus::Active);
        println!("recovery_no_route tick={} age={}",observed.unwrap(),u.no_progress_seconds);
    }
    #[test]
    fn t_junction_traffic_has_a_finite_completion_result() {
        let mut g=CostGrid{w:36,h:30,cost:vec![0;36*30],..Default::default()};
        for y in 14..=16 {for x in 2..34 {g.cost[y*36+x]=1;}}
        for y in 2..=16 {for x in 16..=18 {g.cost[y*36+x]=1;}}
        let mut w=world(g);let a=w.spawn(Unit{idx:0,cell_x:6.0,cell_y:15.0,..Default::default()}).id();
        let b=w.spawn(Unit{idx:1,cell_x:29.0,cell_y:15.0,..Default::default()}).id();let c=w.spawn(Unit{idx:2,cell_x:17.0,cell_y:5.0,..Default::default()}).id();
        let reports=play::issue_orders_at(&mut w,&[(vec![a],Cell::new(29,15)),(vec![b],Cell::new(17,5)),(vec![c],Cell::new(6,15))]);assert!(reports.iter().all(|r|r.accepted==1));
        let mut schedule=nav_scenario::schedule();let mut done=None;
        for tick in 1..=1800 {schedule.run(&mut w);safe(&w);if w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()) {done=Some(tick);break;}}
        assert!(done.is_some(),"T junction did not finish");println!("recovery_t_complete tick={}",done.unwrap());
    }
}
