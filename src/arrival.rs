//! Command completion observes committed motion; steering and contact never count as arrival.
use bevy::math::Vec2;
use crate::formation::{Target,TargetBlock,POINT_REACH_RADIUS};

pub const ENTER_DISTANCE:f32=0.15;
pub const EXIT_DISTANCE:f32=0.30;
pub const QUIET_SPEED:f32=0.05;
pub const HOLD_SECONDS:f64=0.5;

#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub enum CommandStatus {#[default] Idle,Accepted,Active,Completed,Cancelled,Failed}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum WaitReason {Stopped,Space,Entrance,LocalPath}
#[derive(Clone,Copy,Debug,Default,PartialEq,Eq)]
pub enum Motion {#[default] Idle,Moving,Arriving,Waiting(WaitReason),Replanning,Settled,Cancelled,Failed}
impl Motion {
    pub fn tag(self)->u32 {match self {Self::Idle=>0,Self::Moving=>1,Self::Arriving=>2,Self::Waiting(r)=>3+r as u32,
        Self::Replanning=>7,Self::Settled=>8,Self::Cancelled=>9,Self::Failed=>10}}
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum WakeReason {NewOrder,Resume,YieldTask,TargetChanged,Displaced}

#[derive(Clone,Debug,Default)]
pub struct Arrival {
    pub motion:Motion,pub command:CommandStatus,pub quiet_seconds:f64,
    pub anchor:Option<Vec2>,pub max_drift:f32,pub last_wake:Option<WakeReason>,
    stopped:bool,goal:Option<Vec2>,
}
impl Arrival {
    pub fn accept(&mut self) {*self=Self{motion:Motion::Moving,command:CommandStatus::Accepted,
        last_wake:Some(WakeReason::NewOrder),..Self::default()};}
    pub fn cancel(&mut self) {*self=Self{motion:Motion::Cancelled,command:CommandStatus::Cancelled,..Self::default()};}
    pub fn stop(&mut self) {self.stopped=true;self.motion=Motion::Waiting(WaitReason::Stopped);
        self.command=CommandStatus::Active;self.quiet_seconds=0.0;self.anchor=None;}
    pub fn wake(&mut self,reason:WakeReason) {self.stopped=false;self.motion=Motion::Moving;
        self.command=CommandStatus::Active;self.quiet_seconds=0.0;self.anchor=None;self.last_wake=Some(reason);}
    pub fn fail(&mut self) {self.motion=Motion::Failed;self.command=CommandStatus::Failed;
        self.quiet_seconds=0.0;self.anchor=None;}
    pub fn settled(&self)->bool {self.motion==Motion::Settled}
    pub fn held(&self)->bool {self.stopped||self.settled()||self.motion==Motion::Failed}
    pub fn before(&mut self,target:&Target,here:Vec2) {
        if self.command==CommandStatus::Idle {self.accept();}
        if self.command==CommandStatus::Accepted {self.command=CommandStatus::Active;}
        if self.goal.is_some_and(|g|g!=target.position)&&!self.stopped&&self.motion!=Motion::Failed {self.wake(WakeReason::TargetChanged);}
        self.goal=Some(target.position);
        if self.stopped {self.motion=Motion::Waiting(WaitReason::Stopped);return;}
        if self.motion==Motion::Failed {return;}
        if !target.valid {self.motion=Motion::Replanning;self.command=CommandStatus::Active;self.quiet_seconds=0.0;return;}
        if self.settled() {
            if let Some(anchor)=self.anchor {self.max_drift=self.max_drift.max(anchor.distance(here));}
            if here.distance(target.position)<=EXIT_DISTANCE {return;}
            self.wake(WakeReason::Displaced);self.motion=Motion::Replanning;
        }
        self.set_block(target.blocked);
    }
    fn set_block(&mut self,blocked:Option<TargetBlock>) {
        if let Some(b)=blocked {
            self.quiet_seconds=0.0;self.motion=match b {
                TargetBlock::Entrance=>Motion::Waiting(WaitReason::Entrance),
                TargetBlock::NoLocalPath=>Motion::Waiting(WaitReason::LocalPath),
                TargetBlock::InvalidPosition|TargetBlock::OccupiedPosition=>Motion::Replanning,
            };
        }else if matches!(self.motion,Motion::Waiting(_)|Motion::Replanning) {self.motion=Motion::Moving;}
    }
    /// The near navigator supplies actual remaining waypoint length. Far travel
    /// stays outside the braking region and may use its Euclidean lower bound.
    pub fn preferred_speed(&mut self,target:&Target,remaining:f32,max_speed:f32,brake:f32,dt:f32)->f32 {
        if self.held() {return 0.0;}
        self.set_block(target.blocked);
        if !target.valid||!remaining.is_finite()||matches!(self.motion,Motion::Waiting(WaitReason::LocalPath|WaitReason::Space)) {return 0.0;}
        let max_speed=max_speed.max(0.0);let brake=brake.max(0.001);let dt=dt.max(1e-6);
        let start=(max_speed*max_speed/(2.0*brake)+0.3).max(1.0);
        if !matches!(self.motion,Motion::Waiting(_)) {
            if remaining<=start {self.motion=Motion::Arriving;}
            else if self.motion!=Motion::Arriving||remaining>start+0.5 {self.motion=Motion::Moving;}
        }
        let distance=(remaining-POINT_REACH_RADIUS).max(0.0);
        // v*dt plus continuous braking fits the remaining route. The positive
        // root includes the first fixed interval, rather than braking too late.
        ((brake*brake*dt*dt+2.0*brake*distance).sqrt()-brake*dt).max(0.0)
            .min(distance/dt).min(max_speed)
    }
    pub fn observe(&mut self,target:&Target,here:Vec2,speed:f32,dt:f64) {
        if self.settled() {if let Some(anchor)=self.anchor {self.max_drift=self.max_drift.max(anchor.distance(here));}return;}
        if self.held() {return;}
        let quiet=target.valid&&target.frozen&&target.blocked.is_none()&&here.distance(target.position)<=ENTER_DISTANCE
            &&speed<=QUIET_SPEED&&speed.is_finite();
        if quiet {
            self.motion=Motion::Arriving;
            self.quiet_seconds+=dt;
            if self.quiet_seconds>=HOLD_SECONDS {self.motion=Motion::Settled;self.command=CommandStatus::Completed;
                self.anchor=Some(here);self.max_drift=0.0;}
        }else {self.quiet_seconds=0.0;}
    }
}

pub fn status(u:&crate::world::Unit)->CommandStatus {
    if u.arrival.command!=CommandStatus::Idle {u.arrival.command}
    else if u.goal.x>=0 {CommandStatus::Active}else{CommandStatus::Idle}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{formation::{self,Body,Reservations},nav::{Cell,CostGrid}};
    use std::collections::BTreeSet;
    fn install_clock(w:&mut bevy::prelude::World) {
        use bevy::prelude::{Time,Fixed};
        let step=w.resource::<crate::world::SimEnv>().runtime.fixed_step();let mut clock=Time::<Fixed>::from_duration(step);
        clock.advance_by(step);w.insert_resource(clock);
    }
    fn target()->Target {
        let g=CostGrid{w:20,h:20,cost:vec![1;400],..Default::default()};let b=Body{id:0,position:Vec2::new(2.0,10.0),radius:0.45,departing:true,idle:false};
        let mut t=formation::plan(&g,&formation::components(&g),Cell::new(10,10),&[b],&[b],&BTreeSet::from([0]),&mut Reservations::default(),None)
            .unwrap().targets.remove(0).1;t.frozen=true;t
    }
    #[test]
    fn completion_requires_continuous_quiet_time_in_seconds_and_has_hysteresis() {
        for hz in [30.0,60.0,120.0] {
            let t=target();let mut a=Arrival::default();a.accept();a.before(&t,t.position);
            for _ in 0..(hz*0.4) as usize {a.observe(&t,t.position,0.0,1.0/hz);}
            assert!(!a.settled());a.observe(&t,t.position,0.051,1.0/hz);assert_eq!(a.quiet_seconds,0.0);
            for _ in 0..(hz*0.6) as usize {a.observe(&t,t.position,0.0,1.0/hz);}
            assert!(a.settled());assert_eq!(a.command,CommandStatus::Completed);
            a.before(&t,t.position+Vec2::X*0.2);assert!(a.settled());
            a.before(&t,t.position+Vec2::X*0.31);assert!(!a.settled());assert_eq!(a.last_wake,Some(WakeReason::Displaced));
        }
    }
    #[test]
    fn braking_bounds_route_travel_without_reversing_or_hiding_motion() {
        let mut t=target();t.blocked=None;let mut a=Arrival::default();a.accept();a.before(&t,Vec2::new(2.0,10.0));
        let dt=1.0/60.0;let mut remaining=5.0f32;let mut previous=6.0f32;
        for _ in 0..600 {
            let v=a.preferred_speed(&t,remaining,6.0,36.0,dt);
            assert!((0.0..=6.0).contains(&v));assert!(v*dt<=remaining+1e-6);assert!(v<=previous+1e-5);
            remaining-=v*dt;previous=v;
        }
        assert!((remaining-POINT_REACH_RADIUS).abs()<0.001);
        // A detour's route length, rather than its nearby Euclidean endpoint,
        // keeps the requested speed high until the actual route can brake.
        assert_eq!(a.preferred_speed(&t,5.0,6.0,36.0,dt),6.0);
    }
    #[test]
    fn stop_wake_cancel_failure_and_blocked_targets_never_count_as_completion() {
        let mut t=target();let mut a=Arrival::default();a.accept();a.before(&t,t.position);a.stop();
        for _ in 0..120 {a.observe(&t,t.position,0.0,1.0/60.0);}assert!(!a.settled());assert!(a.held());
        a.wake(WakeReason::YieldTask);assert!(!a.held());assert_eq!(a.quiet_seconds,0.0);
        t.blocked=Some(TargetBlock::Entrance);a.before(&t,t.position);
        for _ in 0..120 {a.observe(&t,t.position,0.0,1.0/60.0);}assert!(!a.settled());
        a.fail();assert_eq!(a.command,CommandStatus::Failed);a.wake(WakeReason::Resume);assert_eq!(a.command,CommandStatus::Active);
        a.cancel();assert_eq!(a.command,CommandStatus::Cancelled);a.accept();assert_eq!(a.command,CommandStatus::Accepted);
    }
    #[test]
    fn boundary_jitter_resets_the_entire_hold_interval() {
        let t=target();let mut a=Arrival::default();a.accept();a.before(&t,t.position);
        for i in 0..240 {
            let distance=if i%12==0 {0.151}else{0.14};
            a.observe(&t,t.position+Vec2::X*distance,0.0,1.0/60.0);assert!(!a.settled());
        }
        a.stop();let mut changed=t.clone();changed.position.x+=1.1;a.before(&changed,t.position);
        assert_eq!(a.motion,Motion::Waiting(WaitReason::Stopped));
    }

    #[test]
    fn formal_single_64_and_256_complete_and_keep_their_spaces_for_three_seconds() {
        use bevy::prelude::*;
        use crate::{nav_scenario::{self,SceneConfig,SceneKind},world::{SimEnv,Unit}};
        for count in [64,256] {
            let config=SceneConfig{kind:SceneKind::Single,per_group:count,seed:42,jitter:0.02,sample_every:1,..Default::default()};
            let mut w=World::new();nav_scenario::populate(&mut w,&config).unwrap();let mut schedule=nav_scenario::schedule();
            let mut tick=0;
            for _ in 0..1800 {
                schedule.run(&mut w);tick=w.resource::<SimEnv>().tick;
                let contacts=&w.resource::<SimEnv>().runtime.contacts;assert!(contacts.max_swept_penetration<=0.02);
                assert_eq!(contacts.illegal_static_trajectories,0);assert_eq!(contacts.unresolved_pairs,0);
                if w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()) {break;}
            }
            let mut units:Vec<_>=w.query::<(Entity,&Unit)>().iter(&w).map(|(e,u)|(e,u.idx,formation::position(u))).collect();units.sort_by_key(|u|u.1);
            if !w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()) {
                for u in w.query::<&Unit>().iter(&w).filter(|u|!u.arrival.settled()).take(12) {
                    println!("incomplete id={} position=({},{}) velocity=({},{}) preferred={:?} target={:?}",u.idx,u.cell_x,u.cell_y,u.vx,u.vy,u.preferred_velocity,u.parking);
                }
            }
            assert!(w.query::<&Unit>().iter(&w).all(|u|u.arrival.settled()),"count={count} did not finish by tick {tick}");
            assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),count);
            let mut max_drift=0.0f32;
            for _ in 0..180 {
                schedule.run(&mut w);
                for &(e,_,p) in &units {let u=w.get::<Unit>(e).unwrap();max_drift=max_drift.max(formation::position(u).distance(p));
                    assert!(u.arrival.settled());assert_eq!(u.arrival.command,CommandStatus::Completed);
                    assert!(u.command_id.is_some());assert!(u.parking.as_ref().unwrap().valid);assert!(u.vx.hypot(u.vy)<=QUIET_SPEED);
                }
            }
            assert!(max_drift<=0.05);assert_eq!(w.resource::<SimEnv>().runtime.last_sample.as_ref().unwrap().arrived,Some(count));
            println!("arrival_formal count={count} completion_tick={tick} drift_ticks=180 max_drift={max_drift}");
        }
    }

    #[test]
    fn formal_stop_resume_retarget_and_cancel_preserve_command_boundaries() {
        use bevy::prelude::*;
        use crate::{nav::{Cell,CostGrid},play::{self,Selection},world::{SimEnv,Unit}};
        let mut w=World::new();w.insert_resource(SimEnv{nav:CostGrid{w:32,h:24,cost:vec![1;32*24],..Default::default()},..Default::default()});
        install_clock(&mut w);
        let e=w.spawn(Unit{idx:0,cell_x:5.0,cell_y:12.0,..Default::default()}).id();w.insert_resource(Selection(vec![e]));
        assert_eq!(play::issue_order_at(&mut w,Cell::new(25,12)),1);let id=w.get::<Unit>(e).unwrap().command_id;
        assert_eq!(w.get::<Unit>(e).unwrap().arrival.command,CommandStatus::Accepted);
        let mut schedule=crate::nav_scenario::schedule();for _ in 0..60 {schedule.run(&mut w);}
        assert_eq!(play::stop_selected(&mut w),1);let stopped=formation::position(w.get::<Unit>(e).unwrap());
        for _ in 0..180 {schedule.run(&mut w);assert_eq!(formation::position(w.get::<Unit>(e).unwrap()),stopped);}
        assert_eq!(w.get::<Unit>(e).unwrap().arrival.motion,Motion::Waiting(WaitReason::Stopped));
        assert_eq!(w.get::<Unit>(e).unwrap().command_id,id);assert_eq!(w.resource::<SimEnv>().reservations.claims.len(),1);
        assert_eq!(play::wake_selected(&mut w,WakeReason::Resume),1);
        for _ in 0..600 {schedule.run(&mut w);}
        assert!(w.get::<Unit>(e).unwrap().arrival.settled());
        assert_eq!(play::wake_selected(&mut w,WakeReason::YieldTask),1);assert!(!w.get::<Unit>(e).unwrap().arrival.settled());
        assert_eq!(w.get::<Unit>(e).unwrap().arrival.quiet_seconds,0.0);
        assert_eq!(play::issue_order_at(&mut w,Cell::new(10,12)),1);assert_ne!(w.get::<Unit>(e).unwrap().command_id,id);
        assert_eq!(play::cancel_selected(&mut w),1);assert_eq!(w.get::<Unit>(e).unwrap().arrival.command,CommandStatus::Cancelled);
        assert_eq!(play::wake_selected(&mut w,WakeReason::Resume),0);assert!(w.resource::<SimEnv>().reservations.claims.is_empty());
    }

    #[test]
    fn ordinary_traffic_does_not_wake_or_push_a_completed_owner() {
        use bevy::prelude::*;
        use crate::{nav::{Cell,CostGrid},play::{self,Selection},world::{SimEnv,Unit}};
        let mut w=World::new();w.insert_resource(SimEnv{nav:CostGrid{w:32,h:24,cost:vec![1;32*24],..Default::default()},..Default::default()});
        install_clock(&mut w);
        let a=w.spawn(Unit{idx:0,cell_x:12.0,cell_y:12.0,..Default::default()}).id();
        let b=w.spawn(Unit{idx:1,cell_x:4.0,cell_y:12.0,..Default::default()}).id();w.insert_resource(Selection(vec![a]));
        assert_eq!(play::issue_order_at(&mut w,Cell::new(12,12)),1);let mut schedule=crate::nav_scenario::schedule();
        for _ in 0..60 {schedule.run(&mut w);}assert!(w.get::<Unit>(a).unwrap().arrival.settled());
        let parked=formation::position(w.get::<Unit>(a).unwrap());w.resource_mut::<Selection>().0=vec![b];
        assert_eq!(play::issue_order_at(&mut w,Cell::new(20,12)),1);
        for _ in 0..400 {schedule.run(&mut w);let u=w.get::<Unit>(a).unwrap();assert_eq!(formation::position(u),parked);assert!(u.arrival.settled());}
    }
}
