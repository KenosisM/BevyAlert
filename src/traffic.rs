//! Persistent right-side encounter routes and finite directional gate batches.
//! Decisions change preferred routes and admission, never positions or ORCA outputs.
use std::collections::{BTreeMap,BTreeSet};
use bevy::math::Vec2;
use crate::{collision,formation,nav::CostGrid,orca::HalfPlane,world::Unit};

#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub enum Phase {#[default] Free,Passing,Assembling,FinalApproach,Queueing,Admitted,Traversing,ExitClearing,ExitBlocked}
#[derive(Clone,Copy,Debug,Default)]
pub struct Decision {pub point:Option<Vec2>,pub cap:Option<f32>,pub barrier:Option<HalfPlane>,pub phase:Phase,pub freeze_target:bool}
#[derive(Clone,Debug)]
pub struct Event {pub tick:u64,pub region:usize,pub command:u64,pub unit:i32,pub kind:&'static str,pub side:i32}
#[derive(Clone,Debug,Default)]
pub struct Report {pub passing:usize,pub waiting:usize,pub admitted:usize,pub gates:usize,
    pub batches:u64,pub switches:u64,pub max_wait:f64,pub incompatible:usize,pub unavailable:usize}
struct Origin {start:Vec2,goal:Vec2,radius:f32,area:f32,offsets:BTreeMap<i32,Vec2>}
struct Route {points:Vec<Vec2>,cursor:usize,region:usize}
struct Deployment {targets:BTreeMap<i32,formation::Target>,advancing:bool,region:usize}
#[derive(Default)]
pub struct Traffic {
    origins:BTreeMap<u64,Origin>,routes:BTreeMap<u64,Route>,encountered:BTreeSet<(u64,u64)>,
    deployments:BTreeMap<u64,Deployment>,
    pub decisions:BTreeMap<i32,Decision>,pub events:Vec<Event>,pub report:Report,
    signature:Vec<u8>,width:i32,height:i32,gates:Vec<Gate>,next_region:usize,
}
impl Traffic {
    pub fn decision(&self,id:i32)->Decision {self.decisions.get(&id).copied().unwrap_or_default()}
    pub fn assembly_direction(&mut self,id:i32,command:u64,here:Vec2,tick:u64,q:&mut crate::local_nav::Query<'_>,routes:&mut crate::local_nav::Routes)->Option<Vec2> {
        if self.decision(id).phase!=Phase::Assembling {return None;}
        let deployment=self.deployments.get_mut(&command)?;
        if deployment.advancing {return None;}
        let target=deployment.targets.get_mut(&id)?;
        Some(target.local_direction(here,target.position,command,tick,q,routes))
    }
    pub fn update(&mut self,g:&CostGrid,units:&[Unit],tick:u64,dt:f32) {
        self.decisions.clear();self.report.passing=0;self.report.waiting=0;self.report.admitted=0;
        self.report.incompatible=0;self.report.unavailable=0;
        self.refresh(g);
        let mut groups:BTreeMap<u64,Vec<&Unit>>=BTreeMap::new();
        for u in units {if u.on_ground&&u.parking.is_some() {if let Some(id)=u.command_id {groups.entry(id.0).or_default().push(u);}}}
        self.origins.retain(|id,_|groups.contains_key(id));self.routes.retain(|id,_|groups.contains_key(id));
        self.deployments.retain(|id,_|groups.contains_key(id));
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
                self.routes.insert(id,Route{points,cursor:0,region});
                self.events.push(Event{tick,region,command:id,unit:-1,kind:"right-route",side:1});
            }
        }
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
            for u in &groups[&id] {if u.arrival.held()||u.parking.as_ref().is_some_and(|t|t.local_active) {continue;}
                self.decisions.insert(u.idx,Decision{point:Some(point+origin.offsets[&u.idx]),phase:Phase::Passing,..Default::default()});self.report.passing+=1;
            }
        }
        for id in done {let route=self.routes.remove(&id).unwrap();self.events.push(Event{tick,region:route.region,command:id,unit:-1,kind:"route-cleared",side:1});
            let o=&self.origins[&id];let direction=o.goal-o.start;
            let axis=if direction.x.abs()>=direction.y.abs() {Vec2::X*direction.x.signum()}else{Vec2::Y*direction.y.signum()};
            // Temporary lanes must admit a disc between already aligned owners.
            // Final reservations keep their original spacing and positions.
            let shift=axis*(3.0*o.area+formation::LOCAL_APPROACH_MARGIN+2.0);
            let center=o.goal-shift;let cell=crate::nav::Cell::new(center.x.round() as i32,center.y.round() as i32);
            let mut targets=BTreeMap::new();
            for u in &groups[&id] {let mut t=u.parking.as_ref().unwrap().clone();t.position=center+(t.position-o.goal)*2.0;t.area_radius*=2.0;
                t.phase=t.phase*2.0+(center-Vec2::new(cell.x as f32,cell.y as f32))/formation::PARKING_SPACING;
                t.center=cell;t.predecessor=None;t.frozen=true;t.local_active=true;t.blocked=None;
                targets.insert(u.idx,t);
            }
            if targets.values().all(|t|collision::disc_clear(g,t.position,t.radius)) {
                self.deployments.insert(id,Deployment{targets,advancing:false,region:route.region});
                self.events.push(Event{tick,region:route.region,command:id,unit:-1,kind:"deployment-start",side:1});
            }
        }
        let mut deployed=Vec::new();
        for (&id,d) in &mut self.deployments {
            let group=&groups[&id];
            if group.iter().all(|u|u.arrival.settled()) {deployed.push(id);continue;}
            if !d.advancing&&group.iter().all(|u|formation::position(u).distance(d.targets[&u.idx].position)<=0.06&&u.vx.hypot(u.vy)<=crate::arrival::QUIET_SPEED) {
                d.advancing=true;self.events.push(Event{tick,region:d.region,command:id,unit:-1,kind:"deployment-ready",side:1});
            }
            for u in group {if u.arrival.held() {continue;}
                let point=if d.advancing {u.parking.as_ref().unwrap().position}else{d.targets[&u.idx].position};
                let cap=if d.advancing {None}else{
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
        let now=tick as f64*dt as f64;
        for (index,gate) in self.gates.iter_mut().enumerate() {
            gate.update(index,g,units,tick,now,dt,&mut self.decisions,&mut self.events,&mut self.report);
        }
        self.report.gates=self.gates.len();
    }
    fn refresh(&mut self,g:&CostGrid) {
        if self.width==g.w&&self.height==g.h&&self.signature.len()==g.cost.len()&&self.signature.iter().zip(&g.cost).all(|(a,b)|*a==u8::from(*b>0)) {return;}
        self.width=g.w;self.height=g.h;self.signature=g.cost.iter().map(|v|u8::from(*v>0)).collect();self.gates=discover_gates(g);
    }
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
    fn retargeted_assembly_uses_only_the_current_command_and_phase() {
        let g=CostGrid{w:30,h:24,cost:vec![1;30*24],..Default::default()};let here=Vec2::new(12.0,12.0);
        let body=formation::Body{id:7,position:here,radius:0.45,departing:true,idle:false};
        let mut target=formation::plan(&g,&formation::components(&g),Cell::new(18,12),&[body],&[body],
            &BTreeSet::from([7]),&mut formation::Reservations::default(),None).unwrap().targets.remove(0).1;
        target.position=Vec2::new(6.0,12.0);let mut traffic=Traffic::default();
        traffic.deployments.insert(1,Deployment{targets:BTreeMap::from([(7,target.clone())]),advancing:false,region:0});
        target.position=Vec2::new(18.0,12.0);
        traffic.deployments.insert(2,Deployment{targets:BTreeMap::from([(7,target)]),advancing:false,region:1});
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
