//! Exclusive parking reservations and stable spatial matching.
use std::{collections::{BTreeMap,BTreeSet,VecDeque},sync::Arc,time::Duration};
use bevy::math::Vec2;
use crate::{collision,nav::{self,Cell,CostGrid,FlowField},spatial::{Aabb,Index}};

pub const PARKING_SPACING:f32=1.1;
/// Navigation's point tolerance, separate from physical radius and command completion.
pub const POINT_REACH_RADIUS:f32=0.05;
pub const LOCAL_APPROACH_MARGIN:f32=4.4;
const GAP:f32=0.2;
const EPS:f32=0.0001;

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Outcome {Assigned,Partial,InvalidTarget,NoReachableUnits,WaitingForSpace,BatchAborted}
#[derive(Clone,Debug)]
pub struct OrderReport {pub outcome:Outcome,pub requested:usize,pub accepted:usize,pub unreachable:usize,pub available:usize}
impl OrderReport {
    pub fn failed(outcome:Outcome,requested:usize,available:usize)->Self {
        Self{outcome,requested,accepted:0,unreachable:0,available}
    }
}

#[derive(Clone,Debug)]
pub struct Claim {pub command:u64,pub position:Vec2,pub radius:f32}
#[derive(Clone,Default,Debug)]
pub struct Reservations {
    pub claims:BTreeMap<i32,Claim>,pub last_order:Option<OrderReport>,pub reassignments:u64,
    pub planning_time:Duration,
    owners:BTreeMap<(u32,u32),i32>,pub entry_refinements:u64,
}
impl Reservations {
    pub fn release(&mut self,id:i32) {if let Some(c)=self.claims.remove(&id) {self.owners.remove(&(c.position.x.to_bits(),c.position.y.to_bits()));}}
    pub fn rebuild_owners(&mut self) {
        self.owners.clear();for (id,c) in &self.claims {
            assert!(self.owners.insert((c.position.x.to_bits(),c.position.y.to_bits()),*id).is_none(),"duplicate parking owner");
        }
    }
    fn owner_at(&self,p:Vec2)->Option<i32> {self.owners.get(&(p.x.to_bits(),p.y.to_bits())).copied()}
    pub fn prune(&mut self,units:&[crate::world::Unit]) {
        self.claims.retain(|id,c|units.binary_search_by_key(id,|u|u.idx).ok()
            .is_some_and(|i|units[i].command_id.is_some_and(|v|v.0==c.command)&&units[i].parking.as_ref().is_some_and(|t|t.valid)));
    }
}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum TargetBlock {Entrance,InvalidPosition,OccupiedPosition,NoLocalPath}
#[derive(Clone,Debug)]
pub struct Target {
    pub position:Vec2,pub center:Cell,pub radius:f32,pub area_radius:f32,pub side:usize,pub phase:Vec2,
    pub approach:Vec2,pub predecessor:Option<Vec2>,pub gate:Vec2,pub front_depth:f32,pub waiting_goal:Vec2,
    pub frozen:bool,pub local_active:bool,pub valid:bool,pub blocked:Option<TargetBlock>,
    route:Vec<Vec2>,cursor:usize,route_goal:Option<Vec2>,
}
impl Target {
    pub fn local_direction(&mut self,from:Vec2,goal:Vec2,command:u64,tick:u64,
        q:&mut crate::local_nav::Query<'_>,routes:&mut crate::local_nav::Routes)->Vec2 {
        if q.clear(from,goal,true) {
            self.blocked=None;self.route.clear();self.cursor=0;self.route_goal=None;
            return (goal-from).normalize_or_zero();
        }
        let broken=self.route.get(self.cursor..).is_some_and(|p|p.windows(2).any(|e|!q.clear(e[0],e[1],true)));
        if self.route_goal!=Some(goal)||self.cursor>=self.route.len()||broken {
            self.route.clear();self.cursor=0;self.route_goal=Some(goal);
            let path=routes.path(self,from,goal,command,tick,q);
            if path.is_empty() {self.blocked=Some(TargetBlock::NoLocalPath);return Vec2::ZERO;}
            self.route.extend(path);
        }
        while self.cursor+1<self.route.len() {
            let p=self.route[self.cursor+1];
            if !q.clear(from,p,true) {break;}
            self.cursor+=1;
        }
        let waypoint=self.route[self.cursor];
        if !q.clear(from,waypoint,true) {
            self.route_goal=None;self.blocked=Some(TargetBlock::NoLocalPath);return Vec2::ZERO;
        }
        self.blocked=None;(waypoint-from).normalize_or_zero()
    }
    pub fn remaining_distance(&self,here:Vec2,goal:Vec2)->f32 {
        if self.blocked==Some(TargetBlock::NoLocalPath)||!self.valid {return f32::INFINITY;}
        if self.route_goal==Some(goal)&&self.cursor<self.route.len() {
            here.distance(self.route[self.cursor])+self.route[self.cursor..].windows(2).map(|p|p[0].distance(p[1])).sum::<f32>()
        }else{here.distance(goal)}
    }
}

#[derive(Clone,Copy,Debug)]
pub struct Body {pub id:i32,pub position:Vec2,pub radius:f32,pub departing:bool,pub idle:bool}
pub struct Plan {pub targets:Vec<(i32,Target)>,pub guide:Arc<FlowField>,pub available:usize}
#[derive(Debug)]
pub struct PlanFailure {pub outcome:Outcome,pub available:usize}

/// Reachability is symmetric even though entering terrain costs are directed.
pub fn components(g:&CostGrid)->Vec<u32> {
    if !g.valid() {return Vec::new();}
    let mut result=vec![0;g.cost.len()];let mut label=0;let mut queue=VecDeque::new();
    for y in 0..g.h {for x in 0..g.w {
        let i=(y*g.w+x) as usize;if result[i]!=0||!g.walkable(x,y) {continue;}
        label+=1;result[i]=label;queue.push_back(Cell::new(x,y));
        while let Some(a)=queue.pop_front() {for d in 0..8 {
            let b=Cell::new(a.x+nav::K_DIR_X[d],a.y+nav::K_DIR_Y[d]);
            if nav::step_cost(g,a,b).is_none() {continue;}
            let j=(b.y*g.w+b.x) as usize;if result[j]==0 {result[j]=label;queue.push_back(b);}
        }}
    }}result
}
pub fn component(g:&CostGrid,labels:&[u32],c:Cell)->u32 {
    if !g.in_(c.x,c.y) {return 0;}labels.get((c.y*g.w+c.x) as usize).copied().unwrap_or(0)
}

fn matching(units:&mut [Body],slots:&mut [Vec2],out:&mut Vec<(Body,Vec2)>) {
    if units.is_empty() {return;}if units.len()==1 {out.push((units[0],slots[0]));return;}
    let mut from_lo=Vec2::splat(f32::INFINITY);let mut from_hi=Vec2::splat(f32::NEG_INFINITY);
    let mut to_lo=from_lo;let mut to_hi=from_hi;
    for u in units.iter() {from_lo=from_lo.min(u.position);from_hi=from_hi.max(u.position);}
    for p in slots.iter().copied() {to_lo=to_lo.min(p);to_hi=to_hi.max(p);}
    // Choose the partition axis from each shape's own extent. The translation
    // between shapes must not make every recursion split the travel axis: tiny
    // within-column input jitter would then randomly permute all lateral rows.
    let extent=(from_hi-from_lo).max(to_hi-to_lo);let x=extent.x>=extent.y;
    let key=|p:Vec2|if x {(p.x,p.y)}else{(p.y,p.x)};
    units.sort_by(|a,b|{let (ax,ay)=key(a.position);let (bx,by)=key(b.position);ax.total_cmp(&bx).then(ay.total_cmp(&by)).then(a.id.cmp(&b.id))});
    slots.sort_by(|a,b|{let (ax,ay)=key(*a);let (bx,by)=key(*b);ax.total_cmp(&bx).then(ay.total_cmp(&by))});
    let mid=units.len()/2;let (a,b)=units.split_at_mut(mid);let (c,d)=slots.split_at_mut(mid);
    matching(a,c,out);matching(b,d,out);
}

/// Build a whole target region before committing any command. The caller removes
/// superseded claims only in a staged table and commits the entire batch atomically.
pub fn plan(g:&CostGrid,labels:&[u32],center:Cell,requested:&[Body],all:&[Body],departing:&BTreeSet<i32>,
    table:&mut Reservations,layout:Option<(usize,Vec2)>)->Result<Plan,PlanFailure> {
    let fail=|outcome,available|PlanFailure{outcome,available};
    let group=component(g,labels,center);
    if group==0||requested.is_empty() {return Err(fail(Outcome::InvalidTarget,0));}
    let radius=requested.iter().map(|u|u.radius).fold(0.0f32,f32::max);
    let spacing=PARKING_SPACING.max(2.0*radius+GAP);
    let side=layout.map(|v|v.0).unwrap_or_else(||(requested.len() as f64).sqrt().ceil() as usize).max(1);
    let low=-(side as i32)/2;let high=low+side as i32-1;
    let preferred=layout.map(|v|v.1).unwrap_or(Vec2::splat(if side%2==0 {0.5}else{0.0}));
    let center_pos=Vec2::new(center.x as f32,center.y as f32);
    let average=requested.iter().map(|u|u.position).sum::<Vec2>()/requested.len() as f32;
    let mut approach=(center_pos-average).normalize_or_zero();if approach==Vec2::ZERO {approach=Vec2::X;}
    let mut obstacles:Vec<(Vec2,f32)>=table.claims.values().map(|c|(c.position,c.radius)).collect();
    obstacles.extend(all.iter().filter(|u|!u.departing&&!departing.contains(&u.id)).map(|u|(u.position,u.radius)));
    let max_other=obstacles.iter().map(|o|o.1).fold(radius,f32::max);
    let mut index=Index::default();index.rebuild(&obstacles.iter().map(|o|Aabb::point(o.0)).collect::<Vec<_>>());
    let mut neighbors=Vec::new();let mut candidates=Vec::new();let mut phase=preferred;let mut best_available=0;
    let max_ring=((g.w.max(g.h) as f32/spacing).ceil() as i32)+side as i32;
    // Alternate grid phases allow a legal single-cell-wide target corridor.
    for try_phase in [preferred,Vec2::new(preferred.x,0.0),Vec2::new(0.0,preferred.y),Vec2::ZERO] {
        candidates.clear();phase=try_phase;
        let min=((Vec2::splat(-0.5+radius)-center_pos)/spacing-phase).ceil();
        let max=((Vec2::new(g.w as f32-0.5-radius,g.h as f32-0.5-radius)-center_pos)/spacing-phase).floor();
        if min.x>max.x||min.y>max.y {continue;}
        for ring in 0..=max_ring {
        for y in (low-ring).max(min.y as i32)..=(high+ring).min(max.y as i32) {for x in (low-ring).max(min.x as i32)..=(high+ring).min(max.x as i32) {
            if ring>0&&x>low-ring&&x<high+ring&&y>low-ring&&y<high+ring {continue;}
            let p=center_pos+(Vec2::new(x as f32,y as f32)+phase)*spacing;
            let cell=Cell::new(p.x.round() as i32,p.y.round() as i32);
            if component(g,labels,cell)!=group||!collision::disc_clear(g,p,radius) {continue;}
            index.query_radius(p,radius+max_other+GAP,&mut neighbors);
            if neighbors.iter().any(|j|p.distance(obstacles[*j].0)+EPS<radius+obstacles[*j].1+GAP) {continue;}
            candidates.push((ring,p));
        }}
        if candidates.len()>=requested.len() {break;}
        }
        best_available=best_available.max(candidates.len());
        if candidates.len()>=requested.len() {break;}
    }
    let available=candidates.len();
    if available<requested.len() {return Err(fail(Outcome::WaitingForSpace,best_available));}
    candidates.sort_by(|a,b|a.0.cmp(&b.0).then_with(||b.1.dot(approach).total_cmp(&a.1.dot(approach)))
        .then(a.1.x.total_cmp(&b.1.x)).then(a.1.y.total_cmp(&b.1.y)));
    let mut slots:Vec<_>=candidates.into_iter().take(requested.len()).map(|c|c.1).collect();
    let mut units=requested.to_vec();let mut assigned=Vec::with_capacity(units.len());matching(&mut units,&mut slots,&mut assigned);
    assigned.sort_by_key(|a|a.0.id);
    // Stable local swaps improve the squared travel distance without rematching every tick.
    for _ in 0..2 {for i in 0..assigned.len() {for j in i+1..(i+9).min(assigned.len()) {
        let cost=|a:Vec2,b:Vec2|{let d=a.as_dvec2()-b.as_dvec2();d.length_squared()};
        let before=cost(assigned[i].0.position,assigned[i].1)+cost(assigned[j].0.position,assigned[j].1);
        let after=cost(assigned[i].0.position,assigned[j].1)+cost(assigned[j].0.position,assigned[i].1);
        if after+1e-8<before {let p=assigned[i].1;assigned[i].1=assigned[j].1;assigned[j].1=p;}
    }}}
    let area_radius=assigned.iter().map(|a|a.1.distance(center_pos)).fold(0.0f32,f32::max);
    let front_depth=assigned.iter().map(|a|a.1.dot(approach)).fold(f32::INFINITY,f32::min);
    let lateral=Vec2::new(-approach.y,approach.x);let mut targets=Vec::new();
    for &(body,p) in &assigned {
        let predecessor=assigned.iter().filter(|(other,q)|other.id!=body.id&&(q-p).dot(approach)>spacing*0.5
            &&(q-p).dot(lateral).abs()<spacing*0.55).min_by(|a,b|((a.1-p).dot(approach)).total_cmp(&((b.1-p).dot(approach))).then(a.0.id.cmp(&b.0.id)))
            .map(|a|a.1);
        let gate=p-approach*(p.dot(approach)-front_depth+spacing);
        targets.push((body.id,Target{position:p,center,radius:body.radius,area_radius,side,phase,approach,predecessor,gate,front_depth,waiting_goal:gate,
            frozen:false,local_active:false,valid:true,blocked:None,route:Vec::new(),cursor:0,route_goal:None}));
        table.claims.insert(body.id,Claim{command:0,position:p,radius:body.radius});
    }
    let sources:Vec<_>=targets.iter().map(|(_,t)|Cell::new(t.position.x.round() as i32,t.position.y.round() as i32)).collect();
    let mut field=FlowField::default();
    if !nav::compute_flow_field_cpu_multi(g,&sources,&mut field) {return Err(fail(Outcome::NoReachableUnits,available));}
    Ok(Plan{targets,guide:Arc::new(field),available})
}

pub fn position(u:&crate::world::Unit)->Vec2 {Vec2::new(u.cell_x,u.cell_y)}
pub fn parking_position(u:&crate::world::Unit)->Vec2 {
    u.parking.as_ref().map_or(Vec2::new(u.goal.x as f32,u.goal.y as f32),|t|t.position)
}
pub fn body(u:&crate::world::Unit,radius:f32)->Body {
    Body{id:u.idx,position:position(u),radius:u.parking.as_ref().map_or(radius,|t|t.radius),
        idle:u.goal.x<0,
        departing:u.goal.x>=0&&!u.arrival.held()&&(position(u).distance(parking_position(u))>if u.parking.is_some(){POINT_REACH_RADIUS}else{0.45}
            ||u.vx.hypot(u.vy)>0.05)}
}

fn entered(t:&Target,p:Vec2)->bool {
    p.dot(t.approach)>=t.front_depth-0.5
        &&p.distance(Vec2::new(t.center.x as f32,t.center.y as f32))<=t.area_radius+LOCAL_APPROACH_MARGIN
}

/// Entry events refine only the remaining, unfrozen owners. The complete set of
/// reserved spaces stays unchanged; frozen owners never participate in a swap.
fn refine_entries(env:&mut crate::world::SimEnv,units:&mut [crate::world::Unit]) {
    let mut groups=BTreeMap::<u64,Vec<usize>>::new();
    for (i,u) in units.iter().enumerate() {
        if u.parking.as_ref().is_some_and(|t|t.valid&&!t.frozen) {if let Some(command)=u.command_id {groups.entry(command.0).or_default().push(i);}}
    }
    for (_,indices) in groups {
        if !indices.iter().any(|i|entered(units[*i].parking.as_ref().unwrap(),position(&units[*i]))) {continue;}
        let templates:Vec<_>=indices.iter().map(|i|units[*i].parking.as_ref().unwrap().clone()).collect();
        let approach=templates[0].approach;let lateral=Vec2::new(-approach.y,approach.x);
        // Parking is an axis-aligned lattice. Select its row/column by the
        // dominant approach axis, so column membership is an exact equivalence
        // relation even when small input jitter tilts the approach vector.
        let column=|p:Vec2|if approach.x.abs()>=approach.y.abs(){p.y.to_bits()}else{p.x.to_bits()};
        let mut arriving:Vec<_>=indices.iter().filter(|i|entered(units[**i].parking.as_ref().unwrap(),position(&units[**i])))
            .map(|i|body(&units[*i],env.crowd_params.radius_default())).collect();
        arriving.sort_by(|a,b|b.position.dot(approach).total_cmp(&a.position.dot(approach))
            .then(a.position.dot(lateral).total_cmp(&b.position.dot(lateral))).then(a.id.cmp(&b.id)));
        let mut slots:Vec<_>=templates.iter().map(|t|t.position).collect();let mut assigned=Vec::new();
        for body in arriving {
            // Preserve the owner's lateral lane. Taking the globally deepest
            // free slot would send an early member across neighbouring columns
            // and could seal those columns with parked bodies.
            let own=units.binary_search_by_key(&body.id,|u|u.idx).unwrap();
            let lane=column(units[own].parking.as_ref().unwrap().position);
            let choice=slots.iter().enumerate().filter(|(_,p)|column(**p)==lane)
                .max_by(|(_,a),(_,b)|a.dot(approach).total_cmp(&b.dot(approach))
                    .then(a.x.total_cmp(&b.x)).then(a.y.total_cmp(&b.y))).unwrap().0;
            assigned.push((body,slots.remove(choice)));
        }
        // A refinement only changes owners who enter. The remaining owners keep
        // their current lane and consume its shallow slots in spatial order.
        let mut remaining:Vec<_>=indices.iter().filter(|i|!entered(units[**i].parking.as_ref().unwrap(),position(&units[**i])))
            .map(|i|body(&units[*i],env.crowd_params.radius_default())).collect();
        remaining.sort_by(|a,b|a.position.dot(approach).total_cmp(&b.position.dot(approach)).then(a.id.cmp(&b.id)));
        for body in remaining {
            let own=units.binary_search_by_key(&body.id,|u|u.idx).unwrap();
            let lane=column(units[own].parking.as_ref().unwrap().position);
            let choice=slots.iter().enumerate().filter(|(_,p)|column(**p)==lane)
                .min_by(|(_,a),(_,b)|a.dot(approach).total_cmp(&b.dot(approach))
                    .then(a.x.total_cmp(&b.x)).then(a.y.total_cmp(&b.y))).unwrap().0;
            assigned.push((body,slots.remove(choice)));
        }
        for (body,p) in assigned {
            let i=units.binary_search_by_key(&body.id,|u|u.idx).unwrap();
            let mut target=templates.iter().find(|t|t.position==p).unwrap().clone();
            if units[i].parking.as_ref().unwrap().position!=p {env.reservations.entry_refinements+=1;}
            target.route.clear();target.cursor=0;target.route_goal=None;
            target.frozen=entered(&target,position(&units[i]));
            units[i].parking=Some(target);units[i].goal=Cell::new(p.x.round() as i32,p.y.round() as i32);
            env.reservations.claims.get_mut(&body.id).unwrap().position=p;
        }
    }
}

/// Only invalid positions are automatically reassigned. Other owners and frozen
/// targets remain unchanged. Failed repairs retain an explicit waiting target.
pub fn maintain(env:&mut crate::world::SimEnv,units:&mut [crate::world::Unit]) {
    env.reservations.prune(units);
    let active:BTreeSet<_>=units.iter().filter_map(|u|u.command_id.map(|c|c.0)).collect();env.buffers.local_routes.prune(&active);
    env.buffers.idle_index.begin_tick();env.buffers.idle_bodies.clear();
    env.buffers.idle_bodies.extend(units.iter().filter(|u|u.on_ground&&(u.goal.x<0||u.parking.as_ref().is_some_and(|t|
        u.vx.hypot(u.vy)<=0.05&&(position(u).distance(t.position)<=POINT_REACH_RADIUS||u.arrival.held()||t.blocked==Some(TargetBlock::Entrance)||u.contact_hold||u.traffic_hold))))
        .map(|u|body(u,env.crowd_params.radius_default())));
    let idle_radius=env.buffers.idle_bodies.iter().map(|b|b.radius).fold(0.0f32,f32::max);
    env.buffers.idle_bounds.clear();env.buffers.idle_bounds.extend(env.buffers.idle_bodies.iter().map(|b|Aabb::point(b.position)));
    env.buffers.idle_index.rebuild(&env.buffers.idle_bounds);
    let mut invalid=BTreeMap::<u64,Vec<usize>>::new();
    for (i,u) in units.iter_mut().enumerate() {if let Some(t)=u.parking.as_mut() {
        if !collision::disc_clear(&env.nav,t.position,t.radius) {t.valid=false;t.blocked=Some(TargetBlock::InvalidPosition);env.reservations.release(u.idx);}
        if t.valid&&!env.buffers.idle_bodies.is_empty() {
            env.buffers.idle_index.query_radius(t.position,t.radius+idle_radius,&mut env.buffers.idle_neighbors);
            if env.buffers.idle_neighbors.iter().any(|j|{let b=&env.buffers.idle_bodies[*j];b.idle&&b.id!=u.idx&&b.position.distance(t.position)+EPS<b.radius+t.radius}) {
                t.valid=false;t.blocked=Some(TargetBlock::OccupiedPosition);env.reservations.release(u.idx);
            }
        }
        if !t.valid {if let Some(command)=u.command_id {invalid.entry(command.0).or_default().push(i);}}
    }}
    if !invalid.is_empty()&&env.tick%30==0 {
        let labels=components(&env.nav);let all:Vec<_>=units.iter().filter(|u|u.on_ground).map(|u|body(u,env.crowd_params.radius_default())).collect();
        for (command,indices) in invalid {
            let old=units[indices[0]].parking.as_ref().unwrap();let layout=(old.side,old.phase);
            let Some(center)=crate::play::nearest_usable(&env.nav,old.center) else{continue;};
            let requested:Vec<_>=indices.iter().map(|i|body(&units[*i],env.crowd_params.radius_default())).collect();
            let departing=requested.iter().map(|b|b.id).collect();let mut staged=env.reservations.clone();
            if let Ok(plan)=plan(&env.nav,&labels,center,&requested,&all,&departing,&mut staged,Some(layout)) {
                for (id,target) in plan.targets {
                    let i=units.binary_search_by_key(&id,|u|u.idx).unwrap();let u=&mut units[i];
                    u.goal=Cell::new(target.position.x.round() as i32,target.position.y.round() as i32);
                    u.parking=Some(target);u.guide=Some(plan.guide.clone());
                    staged.claims.get_mut(&id).unwrap().command=command;staged.reassignments+=1;
                }
                env.reservations=staged;
                let sources:Vec<_>=units.iter().filter(|u|u.command_id.is_some_and(|c|c.0==command))
                    .filter_map(|u|u.parking.as_ref().filter(|t|t.valid).map(|t|Cell::new(t.position.x.round() as i32,t.position.y.round() as i32))).collect();
                let mut field=FlowField::default();if nav::compute_flow_field_cpu_multi(&env.nav,&sources,&mut field) {
                    let shared=Arc::new(field);for u in units.iter_mut().filter(|u|u.command_id.is_some_and(|c|c.0==command)) {u.guide=Some(shared.clone());}
                }
            }
        }
    }
    refine_entries(env,units);env.reservations.rebuild_owners();
    let snapshot=&mut env.buffers.formation_positions;snapshot.clear();
    snapshot.extend(units.iter().map(|u|(u.idx,position(u),u.command_id)));
    for u in units.iter_mut() {
        let here=position(u);let command=u.command_id;
        if let Some(t)=u.parking.as_mut() {
            if !t.valid {continue;}
            if here.distance(Vec2::new(t.center.x as f32,t.center.y as f32))<=t.area_radius+LOCAL_APPROACH_MARGIN {t.local_active=true;}
            if entered(t,here) {t.frozen=true;}
            if t.blocked==Some(TargetBlock::Entrance) {t.blocked=None;}
            if t.frozen&&here.distance(t.position)<=PARKING_SPACING {if let Some(slot)=t.predecessor {if let Some(pred)=env.reservations.owner_at(slot) {
                if let Ok(i)=snapshot.binary_search_by_key(&pred,|u|u.0) {
                    if snapshot[i].2==command&&(snapshot[i].1-t.position).dot(t.approach)<t.radius {
                        t.blocked=Some(TargetBlock::Entrance);
                        t.waiting_goal=if (snapshot[i].1-here).dot(t.approach)>=-EPS {here}else{t.gate};
                    }
                }
            }}}
        }
    }
}

pub fn motion_goal(u:&crate::world::Unit)->Vec2 {
    let here=position(u);
    let Some(t)=&u.parking else{return parking_position(u);};
    if !t.valid||matches!(t.blocked,Some(TargetBlock::InvalidPosition|TargetBlock::NoLocalPath)) {return here;}
    if t.blocked==Some(TargetBlock::Entrance) {t.waiting_goal}else{t.position}
}

pub fn direction(u:&mut crate::world::Unit,here:Vec2,tick:u64,q:&mut crate::local_nav::Query<'_>,routes:&mut crate::local_nav::Routes)->Option<Vec2> {
    let command=u.command_id.map_or(0,|c|c.0);let g=q.grid;
    let t=u.parking.as_mut()?;
    if !t.valid {return Some(Vec2::ZERO);}
    if !t.local_active {
        // Read the shared vector without pulling every member onto a grid centre.
        // Near walls retain the centre-following fallback and the same physical guard.
        if let Some(field)=&u.guide {
            let cell=Cell::new(here.x.round() as i32,here.y.round() as i32);
            if let Some(next)=nav::flow_step(g,field,cell) {
                let v=Vec2::new((next.x-cell.x) as f32,(next.y-cell.y) as f32).normalize_or_zero();
                if collision::disc_segment_clear(g,here,here+v,t.radius) {return Some(v);}
            }
        }
        return None;
    }
    let waiting=t.blocked==Some(TargetBlock::Entrance);
    let goal=if waiting {t.waiting_goal}else{t.position};
    if waiting&&!collision::disc_clear(g,goal,t.radius) {return Some(Vec2::ZERO);}
    let direction=t.local_direction(here,goal,command,tick,q,routes);
    if waiting&&t.blocked.is_none() {t.blocked=Some(TargetBlock::Entrance);}
    Some(direction)
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::*;
    use crate::{play::{self,Selection},world::{self,SimEnv,Unit}};

    fn world(w:i32,h:i32)->World {
        let mut world=World::new();world.insert_resource(SimEnv{nav:CostGrid{w,h,cost:vec![1;(w*h) as usize],..default()},..default()});
        world.insert_resource(Selection::default());world
    }
    fn group(w:&mut World,first:i32,n:usize,center:Vec2)->Vec<Entity> {
        let side=(n as f32).sqrt().ceil() as usize;
        (0..n).map(|i|w.spawn(Unit{idx:first+i as i32,
            cell_x:center.x+(i%side) as f32*PARKING_SPACING-(side-1) as f32*PARKING_SPACING*0.5,
            cell_y:center.y+(i/side) as f32*PARKING_SPACING-(side-1) as f32*PARKING_SPACING*0.5,..default()}).id()).collect()
    }
    fn step(w:&mut World,ticks:usize) {
        let mut schedule=Schedule::default();schedule.add_systems(world::update_sim_crowd);
        for _ in 0..ticks {schedule.run(w);}
    }
    fn assert_claims(w:&World,n:usize) {
        let env=w.resource::<SimEnv>();assert_eq!(env.reservations.claims.len(),n);
        let claims:Vec<_>=env.reservations.claims.values().collect();
        for i in 0..n {
            assert!(collision::disc_clear(&env.nav,claims[i].position,claims[i].radius));
            for j in i+1..n {assert!(claims[i].position.distance(claims[j].position)+EPS>=claims[i].radius+claims[j].radius+GAP);}
        }
    }
    #[test]
    fn simultaneous_swaps_reserve_departing_occupants_and_preserve_spatial_order() {
        for n in [64,256] {
            let mut w=world(96,72);let a=group(&mut w,0,n,Vec2::new(20.0,20.0));let b=group(&mut w,n as i32,n,Vec2::new(70.0,50.0));
            let initial:Vec<_>=a.iter().map(|e|position(w.get::<Unit>(*e).unwrap())).collect();
            let reports=play::issue_orders_at(&mut w,&[(a.clone(),Cell::new(70,50)),(b,Cell::new(20,20))]);
            assert!(reports.iter().all(|r|r.accepted==n));assert_claims(&w,2*n);
            for (e,from) in a.iter().zip(initial) {
                let u=w.get::<Unit>(*e).unwrap();assert_eq!(position(u),from);assert_eq!(u.start_delay,0.0);
                let t=u.parking.as_ref().unwrap();assert!((t.position-from-Vec2::new(50.0,30.0)).length()<0.0001);
            }
        }
    }
    #[test]
    fn overlapping_group_requests_expand_without_duplicate_parking_space() {
        let mut w=world(72,64);let a=group(&mut w,0,64,Vec2::new(10.0,10.0));let b=group(&mut w,64,64,Vec2::new(10.0,45.0));
        let result=play::issue_orders_at(&mut w,&[(a,Cell::new(45,30)),(b,Cell::new(45,30))]);
        assert!(result.iter().all(|r|r.accepted==64));assert_claims(&w,128);
    }
    #[test]
    fn retarget_cancel_despawn_and_failed_orders_do_not_leak_claims() {
        let mut w=world(40,30);let a=group(&mut w,0,16,Vec2::new(5.0,5.0));w.resource_mut::<Selection>().0=a.clone();
        assert_eq!(play::issue_order_at(&mut w,Cell::new(25,10)),16);let first=w.get::<Unit>(a[0]).unwrap().command_id;
        assert_eq!(play::issue_order_at(&mut w,Cell::new(25,20)),16);assert_claims(&w,16);
        assert_ne!(w.get::<Unit>(a[0]).unwrap().command_id,first);
        let before=w.get::<Unit>(a[0]).unwrap().parking.as_ref().unwrap().position;
        let current=w.get::<Unit>(a[0]).unwrap().command_id;
        let failed=play::issue_order(&mut w,Cell::new(-1,20));assert_eq!(failed.outcome,Outcome::InvalidTarget);
        assert_eq!(w.get::<Unit>(a[0]).unwrap().parking.as_ref().unwrap().position,before);
        assert_eq!(w.get::<Unit>(a[0]).unwrap().command_id,current);assert_claims(&w,16);
        w.resource_mut::<Selection>().0=vec![a[0],a[0]];let from=position(w.get::<Unit>(a[0]).unwrap());
        assert_eq!(play::cancel_selected(&mut w),1);assert_eq!(position(w.get::<Unit>(a[0]).unwrap()),from);
        assert!(w.get::<Unit>(a[0]).unwrap().command_id.is_none());assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),15);
        w.despawn(a[1]);step(&mut w,1);assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),14);
    }
    #[test]
    fn insufficient_space_and_disconnected_requests_have_explicit_results() {
        let mut w=world(1,1);let a=w.spawn(Unit{idx:0,..default()}).id();let b=w.spawn(Unit{idx:1,..default()}).id();
        w.resource_mut::<Selection>().0=vec![a,b];let result=play::issue_order(&mut w,Cell::new(0,0));
        assert_eq!(result.outcome,Outcome::WaitingForSpace);assert_eq!(result.accepted,0);assert_eq!(result.available,1);
        assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),0);assert!(w.get::<Unit>(a).unwrap().command_id.is_none());
        let mut w=world(30,12);for y in 0..12 {w.resource_mut::<SimEnv>().nav.cost[y*30+15]=0;}
        let a=w.spawn(Unit{idx:0,cell_x:3.0,cell_y:5.0,..default()}).id();w.resource_mut::<Selection>().0=vec![a];
        assert_eq!(play::issue_order_at(&mut w,Cell::new(10,5)),1);let command=w.get::<Unit>(a).unwrap().command_id;
        let failed=play::issue_order(&mut w,Cell::new(24,5));assert_eq!(failed.outcome,Outcome::NoReachableUnits);
        assert_eq!(w.get::<Unit>(a).unwrap().command_id,command);assert_claims(&w,1);
    }
    #[test]
    fn cancelling_a_departing_swap_occupant_reassigns_only_the_blocked_target() {
        let mut w=world(32,20);let a=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:10.0,..default()}).id();
        let b=w.spawn(Unit{idx:1,cell_x:25.0,cell_y:10.0,..default()}).id();
        let result=play::issue_orders_at(&mut w,&[(vec![a],Cell::new(25,10)),(vec![b],Cell::new(5,10))]);
        assert!(result.iter().all(|r|r.accepted==1));let command=w.get::<Unit>(a).unwrap().command_id;
        w.resource_mut::<Selection>().0=vec![b];play::cancel_selected(&mut w);step(&mut w,1);
        assert_claims(&w,1);let u=w.get::<Unit>(a).unwrap();let target=u.parking.as_ref().unwrap();
        assert!(target.valid);assert!(target.position.distance(position(w.get::<Unit>(b).unwrap()))>=PARKING_SPACING-EPS);
        assert_eq!(u.command_id,command);assert_eq!(position(w.get::<Unit>(b).unwrap()),Vec2::new(25.0,10.0));
    }
    #[test]
    fn narrow_target_regions_use_a_legal_lattice_phase() {
        let mut w=world(24,1);let units:Vec<_>=(0..4).map(|i|w.spawn(Unit{idx:i,cell_x:2.0+i as f32*1.1,cell_y:0.0,..default()}).id()).collect();
        w.resource_mut::<Selection>().0=units.clone();assert_eq!(play::issue_order_at(&mut w,Cell::new(18,0)),4);assert_claims(&w,4);
        assert!(units.iter().all(|e|w.get::<Unit>(*e).unwrap().parking.as_ref().unwrap().position.y==0.0));
        step(&mut w,900);
        for e in &units {let u=w.get::<Unit>(*e).unwrap();let t=u.parking.as_ref().unwrap();println!("formation_narrow id={} pos={:?} target={:?} velocity={},{} frozen={} block={:?} waiting={:?}",u.idx,position(u),t.position,u.vx,u.vy,t.frozen,t.blocked,t.waiting_goal);}
        assert!(units.iter().all(|e|{let u=w.get::<Unit>(*e).unwrap();position(u).distance(u.parking.as_ref().unwrap().position)<0.47}));
    }
    #[test]
    fn only_invalid_targets_are_reassigned_and_command_identity_is_retained() {
        let mut w=world(40,40);let a=group(&mut w,0,4,Vec2::new(5.0,5.0));w.resource_mut::<Selection>().0=a.clone();
        assert_eq!(play::issue_order_at(&mut w,Cell::new(28,28)),4);
        let before:Vec<_>=a.iter().map(|e|w.get::<Unit>(*e).unwrap().parking.as_ref().unwrap().position).collect();
        let command=w.get::<Unit>(a[0]).unwrap().command_id;let p=before[0];
        w.resource_mut::<SimEnv>().nav.cost[p.y.round() as usize*40+p.x.round() as usize]=0;
        step(&mut w,1);assert_claims(&w,4);
        assert_ne!(w.get::<Unit>(a[0]).unwrap().parking.as_ref().unwrap().position,before[0]);
        assert_eq!(w.get::<Unit>(a[0]).unwrap().command_id,command);
        for (e,p) in a.iter().zip(before).skip(1) {assert_eq!(w.get::<Unit>(*e).unwrap().parking.as_ref().unwrap().position,p);}
    }
    #[test]
    fn local_navigation_uses_physical_edges_when_point_cutting_is_enabled() {
        let mut w=world(12,12);w.resource_mut::<SimEnv>().nav.cut_corners=true;w.resource_mut::<SimEnv>().nav.cost[4*12+5]=0;
        let a=w.spawn(Unit{idx:0,cell_x:4.0,cell_y:4.0,..default()}).id();w.resource_mut::<Selection>().0=vec![a];
        assert_eq!(play::issue_order_at(&mut w,Cell::new(5,5)),1);
        for _ in 0..600 {step(&mut w,1);assert_eq!(w.resource::<SimEnv>().runtime.contacts.illegal_static_trajectories,0);}
        let u=w.get::<Unit>(a).unwrap();assert!(position(u).distance(u.parking.as_ref().unwrap().position)<0.47);
    }
    #[test]
    fn formal_near_navigation_routes_around_an_idle_occupant_without_pushing_it() {
        let mut w=world(32,24);let a=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..default()}).id();
        let idle=w.spawn(Unit{idx:1,cell_x:12.0,cell_y:12.0,..default()}).id();
        w.resource_mut::<Selection>().0=vec![a];assert_eq!(play::issue_order_at(&mut w,Cell::new(15,12)),1);
        for _ in 0..900 {
            step(&mut w,1);let env=w.resource::<SimEnv>();assert_eq!(env.runtime.contacts.illegal_static_trajectories,0);
            assert!(env.runtime.contacts.max_swept_penetration<=0.02);assert_eq!(position(w.get::<Unit>(idle).unwrap()),Vec2::new(12.0,12.0));
        }
        let u=w.get::<Unit>(a).unwrap();assert!(position(u).distance(u.parking.as_ref().unwrap().position)<0.15);
    }
    #[test]
    fn single_groups_64_and_256_keep_frozen_targets_and_reach_their_near_region() {
        for (n,jitter) in [(64,0.0f32),(256,0.0),(64,0.02),(256,0.02)] {
            let mut w=world(64,56);let a=group(&mut w,0,n,Vec2::new(18.0,28.0));w.resource_mut::<Selection>().0=a.clone();
            w.resource_mut::<SimEnv>().runtime.sample_every=1;
            let mut seed=42u64;
            for e in &a {
                let mut u=w.get_mut::<Unit>(*e).unwrap();
                let mut noise=||{seed=seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    ((seed>>32) as u32 as f64/u32::MAX as f64-0.5) as f32*jitter};
                u.cell_x+=noise();u.cell_y+=noise();
            }
            assert_eq!(play::issue_order_at(&mut w,Cell::new(38,28)),n);assert_claims(&w,n);
            for e in &a {let u=w.get::<Unit>(*e).unwrap();
                assert!((u.parking.as_ref().unwrap().position-position(u)-Vec2::new(20.0,0.0)).length()<0.02,
                    "small input jitter permuted a translated formation");
            }
            let spaces:BTreeSet<_>=w.resource::<SimEnv>().reservations.claims.values().map(|c|(c.position.x.to_bits(),c.position.y.to_bits())).collect();
            let lanes:BTreeMap<_,_>=a.iter().map(|e|{let u=w.get::<Unit>(*e).unwrap();(u.idx,u.parking.as_ref().unwrap().position.y)}).collect();
            let mut frozen=BTreeMap::new();let mut schedule=Schedule::default();schedule.add_systems(world::update_sim_crowd);
            for _ in 0..1800 {
                schedule.run(&mut w);let env=w.resource::<SimEnv>();
                assert!(env.runtime.contacts.max_swept_penetration<=0.02);assert_eq!(env.runtime.contacts.illegal_static_trajectories,0);
                let current:BTreeSet<_>=env.reservations.claims.values().map(|c|(c.position.x.to_bits(),c.position.y.to_bits())).collect();assert_eq!(current,spaces);
                for e in &a {let u=w.get::<Unit>(*e).unwrap();let t=u.parking.as_ref().unwrap();
                    assert_eq!(env.reservations.owner_at(t.position),Some(u.idx));
                    assert!((t.position.y-lanes[&u.idx]).abs()<EPS,"entry refinement crossed another parking column");
                    if let Some(p)=frozen.get(&u.idx) {assert_eq!(*p,t.position);assert!(t.frozen);}
                    if t.frozen {frozen.insert(u.idx,t.position);}
                }
            }
            let mut max_remaining=0.0f32;let mut frozen_count=0;let mut worst=None;
            for e in &a {
                let u=w.get::<Unit>(*e).unwrap();let t=u.parking.as_ref().unwrap();let p=t.position;
                frozen_count+=usize::from(t.frozen);
                if !t.frozen {println!("formation_far id={} pos={},{} velocity={},{} target={:?} valid={} block={:?} escape={}",u.idx,u.cell_x,u.cell_y,u.vx,u.vy,t.position,t.valid,t.blocked,u.escape_ticks);}
                let remaining=position(u).distance(p);
                if remaining>=max_remaining {max_remaining=remaining;worst=Some((u.idx,position(u),p,u.vx,u.vy,t.blocked));}
            }
            println!("formation_near count={n} jitter={jitter} ticks=1800 max_remaining={max_remaining} frozen={frozen_count} worst={worst:?}");
            let contact=&w.resource::<SimEnv>().runtime.contacts;
            println!("formation_contacts count={n} limits={} safety_stops={} max_penetration={} initial={}",contact.limited_trajectories,
                contact.reasons.iter().filter(|r|**r==crate::collision::Blocked::SafetyStop).count(),contact.max_penetration,contact.initial_overlaps);
            assert_eq!(frozen_count,n);
            assert!(max_remaining<0.15,"count={n} remaining={max_remaining}");
            assert!(a.iter().all(|e|w.get::<Unit>(*e).unwrap().arrival.settled()));
            assert_eq!(w.resource::<SimEnv>().runtime.last_sample.as_ref().and_then(|s|s.arrived),Some(n));
        }
    }
}
