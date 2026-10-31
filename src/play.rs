//! 指令链 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\engine\play.cpp` 的 `nearestUsable`（149-160）与
//! `issueOrderAt`（166-230）。
//!
//! 上游 `play.cpp` 还含窗口 / DIB / blit / 相机 / PNG 输出 —— 那些属渲染与平台层，
//! 由 Bevy 侧承担，**不搬**。

use std::collections::BTreeSet;

use bevy::prelude::*;

use crate::nav::{Cell,CostGrid};
use crate::formation::{self,Body,Outcome,OrderReport};
use crate::world::{SimEnv, Unit};

/// 当前选中的单位。上游是 `Play::sel`（entities 的下标列表）。
#[derive(Resource, Default)]
pub struct Selection(pub Vec<Entity>);

/// 一次指令落在障碍上时，就近找一格可用的：点在障碍上仍然要让部队尽量靠近，而不是什么都不做。
/// 上游签名带一个用不上的 `Footprint`；这里省掉。
pub fn nearest_usable(g:&CostGrid,want:Cell)->Option<Cell> {
    if !g.valid() || !g.in_(want.x,want.y) {return None;}
    if g.walkable(want.x,want.y) {return Some(want);}
    let mut best=None;let mut best_dist=i64::MAX;
    for y in 0..g.h {for x in 0..g.w {
        if !g.walkable(x,y) {continue;}
        let dx=(x-want.x) as i64;let dy=(y-want.y) as i64;let d=dx*dx+dy*dy;
        if d<best_dist {best_dist=d;best=Some(Cell::new(x,y));}
    }}best
}

/// Compatibility count; callers needing the reason use issue_order or issue_orders_at.
pub fn issue_order_at(world:&mut World,dest:Cell)->usize {issue_order(world,dest).accepted}

pub fn issue_order(world:&mut World,dest:Cell)->OrderReport {
    let selected=world.resource::<Selection>().0.clone();
    let report=issue_orders_at(world,&[(selected,dest)]).remove(0);
    world.resource_mut::<SimEnv>().reservations.last_order=Some(report.clone());report
}

/// Simultaneous orders reserve atomically. Departing occupants are classified
/// before any new region is allocated, so swaps do not depend on request order.
pub fn issue_orders_at(world:&mut World,requests:&[(Vec<Entity>,Cell)])->Vec<OrderReport> {
    let start=std::time::Instant::now();let reports=issue_orders_inner(world,requests);
    let mut env=world.resource_mut::<SimEnv>();env.reservations.planning_time=start.elapsed();
    env.reservations.last_order=reports.last().cloned();reports
}

fn issue_orders_inner(world:&mut World,requests:&[(Vec<Entity>,Cell)])->Vec<OrderReport> {
    let (nav,footprint,radius)= {
        let env=world.resource::<SimEnv>();(env.nav.clone(),env.footprint,env.crowd_params.radius_default())
    };
    let reject=|index:usize,outcome:Outcome,available:usize|requests.iter().enumerate().map(|(i,(units,_))|
        OrderReport::failed(if i==index {outcome}else{Outcome::BatchAborted},units.len(),available)).collect::<Vec<_>>();
    if requests.is_empty() {return Vec::new();}
    if !nav.valid()||footprint.w<=0||footprint.h<=0 {return reject(0,Outcome::InvalidTarget,0);}
    let mut snapshots:Vec<_>=world.query::<&Unit>().iter(world).cloned().collect();snapshots.sort_by_key(|u|u.idx);
    let all:Vec<_>=snapshots.iter().filter(|u|u.on_ground).map(|u|formation::body(u,radius)).collect();
    let labels=formation::components(&nav);let mut normalized=Vec::new();let mut departing=BTreeSet::new();
    for (index,(selected,want)) in requests.iter().enumerate() {
        let Some(dest)=nearest_usable(&nav,*want) else{return reject(index,Outcome::InvalidTarget,0);};
        let group=formation::component(&nav,&labels,dest);
        let mut entities:Vec<_>=selected.iter().copied().filter(|e|world.get::<Unit>(*e).is_some_and(|u|
            u.on_ground&&u.cell_x.is_finite()&&u.cell_y.is_finite()&&crate::collision::disc_clear(&nav,
                bevy::math::Vec2::new(u.cell_x,u.cell_y),radius))).collect();
        entities.sort_by_key(|e|world.get::<Unit>(*e).unwrap().idx);entities.dedup();
        let mut reachable=Vec::new();let mut unreachable=0;
        for e in entities {
            let u=world.get::<Unit>(e).unwrap();let c=Cell::new(u.cell_x.round() as i32,u.cell_y.round() as i32);
            if formation::component(&nav,&labels,c)!=group {unreachable+=1;continue;}
            if !departing.insert(u.idx) {return reject(index,Outcome::BatchAborted,0);}
            reachable.push(e);
        }
        if reachable.is_empty() {return reject(index,Outcome::NoReachableUnits,0);}
        normalized.push((dest,reachable,unreachable));
    }
    let mut staged=world.resource::<SimEnv>().reservations.clone();staged.prune(&snapshots);
    for id in &departing {staged.release(*id);}
    let mut plans=Vec::new();
    for (index,(dest,entities,_)) in normalized.iter().enumerate() {
        let bodies:Vec<Body>=entities.iter().map(|e|formation::body(world.get::<Unit>(*e).unwrap(),radius)).collect();
        match formation::plan(&nav,&labels,*dest,&bodies,&all,&departing,&mut staged,None) {
            Ok(plan)=>plans.push(plan),Err(f)=>return reject(index,f.outcome,f.available),
        }
    }
    let mut reports=Vec::new();
    for (index,((_,entities,unreachable),plan)) in normalized.into_iter().zip(plans).enumerate() {
        let accepted=entities.len();
        let command=world.resource_mut::<SimEnv>().runtime.allocate_command();
        let ap_budget=(2.0/world.resource::<SimEnv>().runtime.dt()).ceil() as i32;
        for ((id,target),e) in plan.targets.into_iter().zip(entities) {
            let mut u=world.get_mut::<Unit>(e).unwrap();assert_eq!(u.idx,id);
            u.goal=Cell::new(target.position.x.round() as i32,target.position.y.round() as i32);
            u.parking=Some(target);u.command_id=Some(command);u.guide=Some(plan.guide.clone());
            u.contact_hold=false;
            u.start_delay=0.0;u.ap_budget=ap_budget;u.stall_ticks=0;u.still_ticks=0;u.escape_ticks=0;u.turret_idle=false;
            if u.path.len()!=2 {u.path.clear();u.leg_t=0.0;}
            staged.claims.get_mut(&id).unwrap().command=command.0;
        }
        let report=OrderReport{outcome:if unreachable>0||accepted<requests[index].0.len(){Outcome::Partial}else{Outcome::Assigned},
            requested:requests[index].0.len(),accepted,unreachable,available:plan.available};
        reports.push(report);
    }
    staged.last_order=reports.last().cloned();staged.rebuild_owners();world.resource_mut::<SimEnv>().reservations=staged;reports
}

/// Cancellation releases the selected owners' claims and retains their current positions.
pub fn cancel_selected(world:&mut World)->usize {
    let selected=world.resource::<Selection>().0.clone();let mut ids=BTreeSet::new();
    for e in selected {
        if let Some(mut u)=world.get_mut::<Unit>(e) {
            ids.insert(u.idx);u.goal=Cell::new(-1,-1);u.command_id=None;u.parking=None;u.guide=None;
            u.contact_hold=false;
            u.path.clear();u.leg_t=0.0;u.vx=0.0;u.vy=0.0;u.stall_ticks=0;u.escape_ticks=0;
        }
    }
    let mut env=world.resource_mut::<SimEnv>();for id in &ids {env.reservations.release(*id);}ids.len()
}
