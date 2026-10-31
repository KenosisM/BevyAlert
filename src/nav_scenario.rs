//! Shared, asset-free scenario inputs and observations for the window and CLI.

use bevy::prelude::*;
use std::{fs, path::{Path, PathBuf}, time::Instant};
use crate::{ini::Ini, nav::{Cell, CostGrid}, nav_headless::SeededInput, play::{self, Selection},
    simulation::Runtime, world::{self, GroundField, SimEnv, Unit}};

pub const WIDTH: i32 = 192;
pub const HEIGHT: i32 = 160;
pub const TICK_HZ: f64 = 60.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SceneKind { Crossing, HeadOn, Single }

impl SceneKind {
    pub fn name(self) -> &'static str {
        match self { Self::Crossing => "crossing", Self::HeadOn => "head-on", Self::Single => "single" }
    }
    pub fn parse(value: &str) -> Result<Self, String> {
        match value {
            "crossing" => Ok(Self::Crossing), "head-on" => Ok(Self::HeadOn),
            "single" => Ok(Self::Single), _ => Err("scene must be crossing, head-on or single".into()),
        }
    }
    /// All routes receive their orders at tick zero, before either runner advances.
    pub fn routes(self) -> Vec<(Cell, Cell)> {
        let a = Cell::new(32, 40); let b = Cell::new(32, 120);
        let c = Cell::new(160, 40); let d = Cell::new(160, 120);
        match self {
            Self::Crossing => vec![(a,d), (b,c), (c,b), (d,a)],
            Self::HeadOn => vec![(Cell::new(32,80),Cell::new(160,80)), (Cell::new(160,80),Cell::new(32,80))],
            Self::Single => vec![(Cell::new(32,80),Cell::new(160,80))],
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SceneConfig {
    pub kind: SceneKind,
    pub per_group: usize,
    pub seed: u64,
    /// Maximum full jitter width. Zero is the exact symmetric acceptance layout.
    pub jitter: f32,
    pub spacing: f32,
    pub continuous: bool,
    pub sample_every: u64,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self { kind: SceneKind::Crossing, per_group: 64, seed: 0, jitter: 0.0,
            spacing: 1.1, continuous: true, sample_every: 60 }
    }
}

impl SceneConfig {
    pub fn validate(&self) -> Result<(), String> {
        if ![64,256,1024].contains(&self.per_group) { return Err("per-group must be 64, 256 or 1024".into()); }
        if !self.jitter.is_finite() || !(0.0..=0.1).contains(&self.jitter) { return Err("jitter must be in 0..=0.1".into()); }
        if (self.spacing - 1.1).abs() > 1e-6 { return Err("these presets require spacing 1.1".into()); }
        if self.sample_every == 0 { return Err("sample interval must be positive".into()); }
        Ok(())
    }
    pub fn mode(&self) -> &'static str { if self.continuous { "continuous-legacy" } else { "discrete-legacy" } }
    pub fn load(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path).map_err(|e| format!("cannot read scenario: {e}"))?;
        Self::decode(&text)
    }
    pub fn decode(text: &str) -> Result<Self,String> {
        let ini = Ini::parse(text);
        let s = ini.get("Scenario").ok_or("missing [Scenario]")?;
        for entry in &s.entries {
            if !["scene","pergroup","seed","jitter","spacing","mode","sampleevery","width","height","tickhz"].contains(&entry.key.to_ascii_lowercase().as_str()) {
                return Err(format!("unknown scenario key: {}",entry.key));
            }
        }
        let mut c = Self::default();
        if let Some(v)=s.value("Scene") { c.kind=SceneKind::parse(v)?; }
        if let Some(v)=s.value("PerGroup") { c.per_group=v.parse().map_err(|_| "invalid PerGroup")?; }
        if let Some(v)=s.value("Seed") { c.seed=v.parse().map_err(|_| "invalid Seed")?; }
        if let Some(v)=s.value("Jitter") { c.jitter=v.parse().map_err(|_| "invalid Jitter")?; }
        if let Some(v)=s.value("Spacing") { c.spacing=v.parse().map_err(|_| "invalid Spacing")?; }
        if let Some(v)=s.value("SampleEvery") { c.sample_every=v.parse().map_err(|_| "invalid SampleEvery")?; }
        if let Some(v)=s.value("Mode") { c.continuous=parse_mode(v)?; }
        for (key,want) in [("Width",WIDTH as f64),("Height",HEIGHT as f64),("TickHz",TICK_HZ)] {
            if let Some(v)=s.value(key) {
                let v:f64=v.parse().map_err(|_|format!("invalid {key}"))?;
                if v!=want { return Err(format!("preset {key} must be {want}")); }
            }
        }
        c.validate()?; Ok(c)
    }
    pub fn encode(&self) -> String {
        format!("[Scenario]\nScene={}\nPerGroup={}\nSeed={}\nJitter={}\nSpacing={}\nMode={}\nSampleEvery={}\nWidth={WIDTH}\nHeight={HEIGHT}\nTickHz={TICK_HZ}\n",
            self.kind.name(),self.per_group,self.seed,self.jitter,self.spacing,
            if self.continuous { "continuous" } else { "discrete" },self.sample_every)
    }
}

fn parse_mode(v: &str) -> Result<bool,String> {
    match v { "continuous"=>Ok(true), "discrete"=>Ok(false), _=>Err("mode must be continuous or discrete".into()) }
}

#[derive(Clone, Debug)]
pub struct RunOptions {
    pub headless: bool,
    pub ticks: u64,
    pub csv: Option<PathBuf>,
    pub paused: bool,
    pub exit_after_ticks: Option<u64>,
    pub capture: Option<PathBuf>,
    pub exit_after_frames: Option<u64>,
}

pub fn parse_args(args: &[String]) -> Result<(SceneConfig,RunOptions),String> {
    let mut config = SceneConfig::default();
    for pair in args.windows(2) { if pair[0]=="--config" { config=SceneConfig::load(Path::new(&pair[1]))?; } }
    let mut run = RunOptions { headless:false,ticks:3600,csv:None,paused:false,exit_after_ticks:None,capture:None,exit_after_frames:None };
    let mut i=0;
    while i<args.len() {
        let option=&args[i];
        if option=="--headless" { run.headless=true; i+=1; continue; }
        if option=="--paused" { run.paused=true; i+=1; continue; }
        let value=args.get(i+1).ok_or_else(||format!("missing value for {option}"))?;
        match option.as_str() {
            "--config"=>{}, "--scene"=>config.kind=SceneKind::parse(value)?,
            "--per-group"=>config.per_group=value.parse().map_err(|_|"invalid per-group")?,
            "--seed"=>config.seed=value.parse().map_err(|_|"invalid seed")?,
            "--jitter"=>config.jitter=value.parse().map_err(|_|"invalid jitter")?,
            "--mode"=>config.continuous=parse_mode(value)?,
            "--sample-every"=>config.sample_every=value.parse().map_err(|_|"invalid sample interval")?,
            "--ticks"=>run.ticks=value.parse().map_err(|_|"invalid tick count")?,
            "--csv"=>run.csv=Some(PathBuf::from(value)),
            "--exit-after-ticks"=>run.exit_after_ticks=Some(value.parse().map_err(|_|"invalid exit tick")?),
            "--capture"=>run.capture=Some(PathBuf::from(value)),
            "--exit-after-frames"=>run.exit_after_frames=Some(value.parse().map_err(|_|"invalid exit frame")?),
            _=>return Err(format!("unknown option: {option}")),
        }
        i+=2;
    }
    config.validate()?; Ok((config,run))
}

/// The only scenario/command constructor; both runners use this exact function.
pub fn populate(world: &mut World, config: &SceneConfig) -> Result<(),String> {
    config.validate()?;
    let old:Vec<_>=world.query_filtered::<Entity,With<Unit>>().iter(world).collect();
    for e in old { world.despawn(e); }
    let mut runtime=Runtime::from_hz(TICK_HZ,config.seed);
    runtime.sample_every=config.sample_every;
    let step=runtime.fixed_step();
    world.insert_resource(SimEnv {
        nav:CostGrid { w:WIDTH,h:HEIGHT,cost:vec![1;(WIDTH*HEIGHT) as usize],..default() },
        ground:GroundField { w:WIDTH,h:HEIGHT,level:vec![0;(WIDTH*HEIGHT) as usize],..default() },
        use_crowd:config.continuous,runtime,..default()
    });
    world.insert_resource(Selection::default());
    let mut fixed=Time::<Fixed>::from_duration(step); fixed.advance_by(step); world.insert_resource(fixed);
    let mut random=SeededInput(config.seed);
    let side=(config.per_group as f32).sqrt().ceil() as usize;
    let mut requests=Vec::new();
    for (group,(start,dest)) in config.kind.routes().into_iter().enumerate() {
        let mut selected=Vec::with_capacity(config.per_group);
        for index in 0..config.per_group {
            let jitter_x=random.jitter()*(config.jitter/0.02);
            let jitter_y=random.jitter()*(config.jitter/0.02);
            let unit=Unit {
                idx:(group*config.per_group+index) as i32,
                cell_x:start.x as f32+(index%side) as f32*config.spacing-(side-1) as f32*config.spacing*0.5+jitter_x,
                cell_y:start.y as f32+(index/side) as f32*config.spacing-(side-1) as f32*config.spacing*0.5+jitter_y,
                ..default()
            };
            selected.push(world.spawn(unit).id());
        }
        requests.push((selected,dest));
    }
    let reports=play::issue_orders_at(world,&requests);
    if reports.iter().any(|r|r.accepted!=config.per_group) {return Err(format!("scenario command rejected: {reports:?}"));}
    for mut u in world.query::<&mut Unit>().iter_mut(world) { u.start_delay=0.0; }
    world.resource_mut::<Selection>().0.clear();
    Ok(())
}

pub fn schedule() -> Schedule {
    let mut s=Schedule::default();
    s.add_systems((world::update_sim,world::update_sim_crowd).chain()); s
}

#[derive(Clone, Debug, Default)]
pub struct Metrics {
    pub tick:u64, pub active:usize, pub moved:usize, pub legacy_stalled:usize,
    pub max_still_seconds:f32, pub overlap_end:f32, pub bad_centers:usize,
    pub total_us:u128, pub snapshot_us:u128, pub pressure_us:u128,
    pub velocity_us:u128, pub contact_us:u128, pub writeback_us:u128, pub digest:u64, pub timing_ready:bool,
    pub swept_penetration:f32, pub unresolved_contacts:usize, pub illegal_trajectories:usize,
    pub limited_trajectories:usize, pub initial_overlaps:usize, pub safety_stops:usize,
    pub collision_ready:bool,
    pub spatial:crate::spatial::Stats,
    pub reserved:usize,pub frozen_targets:usize,pub waiting_space:usize,pub waiting_entrance:usize,pub blocked_near:usize,
    pub planning_us:u128,pub formation_us:u128,
    pub entry_refinements:u64,
    pub arrived:Option<usize>,pub arriving:usize,pub waiting:[usize;4],pub replanning:usize,pub failed:usize,pub max_parked_drift:f32,
}
impl Metrics {
    pub fn arrival_text(&self)->String {self.arrived.map_or("not-ready".into(),|v|v.to_string())}
    pub fn waiting_text(&self)->String {if self.arrived.is_none(){return "not-ready".into();}
        format!("stopped:{}|space:{}|entrance:{}|path:{}|replan:{}",self.waiting[0],self.waiting[1],self.waiting[2],self.waiting[3],self.replanning)}
}

pub fn metrics(world:&mut World) -> Metrics {
    let mut q=world.query::<&Unit>(); let mut units:Vec<_>=q.iter(world).collect(); units.sort_by_key(|u|u.idx);
    let env=world.resource::<SimEnv>();
    let mut m=Metrics { tick:env.tick,..default() };
    let contacts=&env.runtime.contacts;
    m.spatial=env.runtime.spatial_stats.clone();
    m.reserved=env.reservations.claims.len();m.planning_us=env.reservations.planning_time.as_micros();
    m.entry_refinements=env.reservations.entry_refinements;
    m.collision_ready=env.use_crowd&&env.tick>0;
    m.swept_penetration=contacts.max_swept_penetration;m.unresolved_contacts=contacts.unresolved_pairs;
    m.illegal_trajectories=contacts.illegal_static_trajectories;m.limited_trajectories=contacts.limited_trajectories;
    m.initial_overlaps=contacts.initial_overlaps;
    m.arrived=(env.use_crowd&&units.iter().all(|u|u.parking.is_some()||u.goal.x<0))
        .then(||units.iter().filter(|u|u.arrival.settled()).count());
    m.safety_stops=contacts.reasons.iter().filter(|r|**r==crate::collision::Blocked::SafetyStop).count();
    let mut digest=0xcbf29ce484222325u64;
    for (i,u) in units.iter().enumerate() {
        m.active+=usize::from(matches!(crate::arrival::status(u),crate::arrival::CommandStatus::Accepted|crate::arrival::CommandStatus::Active));
        use crate::arrival::Motion;
        match u.arrival.motion {Motion::Arriving=>m.arriving+=1,Motion::Waiting(r)=>m.waiting[r as usize]+=1,
            Motion::Replanning=>m.replanning+=1,Motion::Failed=>m.failed+=1,_=>{}}
        m.max_parked_drift=m.max_parked_drift.max(u.arrival.max_drift);
        if let Some(t)=&u.parking {
            m.frozen_targets+=usize::from(t.frozen);m.waiting_space+=usize::from(!t.valid);
            m.waiting_entrance+=usize::from(t.blocked==Some(crate::formation::TargetBlock::Entrance));
            m.blocked_near+=usize::from(t.blocked==Some(crate::formation::TargetBlock::NoLocalPath));
        }
        m.moved+=usize::from(u.vx.hypot(u.vy)>0.02);
        m.legacy_stalled+=usize::from(u.stall_ticks>0);
        m.max_still_seconds=m.max_still_seconds.max(u.still_ticks as f32*env.runtime.dt());
        m.bad_centers+=usize::from(!env.nav.walkable(u.cell_x.round() as i32,u.cell_y.round() as i32));
        for v in [u.idx as u32,u.cell_x.to_bits(),u.cell_y.to_bits(),u.vx.to_bits(),u.vy.to_bits(),u.stall_ticks,u.still_ticks] {
            digest=(digest^v as u64).wrapping_mul(0x100000001b3);
        }
        for v in [u.arrival.motion.tag() as u64,u.arrival.command as u64,u.arrival.quiet_seconds.to_bits()] {
            digest=(digest^v).wrapping_mul(0x100000001b3);
        }
        if !m.collision_ready {for other in &units[i+1..] {
            m.overlap_end=m.overlap_end.max((env.crowd_params.radius_default()*2.0-(u.cell_x-other.cell_x).hypot(u.cell_y-other.cell_y)).max(0.0));
        }}
    }
    if m.collision_ready {m.overlap_end=contacts.max_penetration;}
    m.digest=digest;
    if let Some(sample)=env.runtime.last_sample.as_ref().filter(|s|s.tick==env.tick) {
        m.timing_ready=true;
        let t=&sample.times;
        m.total_us=t.total.as_micros();m.snapshot_us=t.snapshot.as_micros();m.pressure_us=t.pressure.as_micros();
        m.velocity_us=t.velocity.as_micros();m.contact_us=t.contact.as_micros();m.writeback_us=t.writeback.as_micros();
        m.formation_us=t.formation.as_micros();
    }
    m
}

pub fn write_report(world:&mut World,config:&SceneConfig,samples:&[Metrics],path:&Path) -> Result<(),String> {
    use std::fmt::Write;
    if let Some(parent)=path.parent().filter(|p|!p.as_os_str().is_empty()) { fs::create_dir_all(parent).map_err(|e|e.to_string())?; }
    let mut csv=String::from("tick,seconds,active,moving,arrived,waiting_reason,legacy_stalled,max_still_seconds,overlap_end,bad_centers,core_us,snapshot_us,pressure_us,velocity_us,contact_us,writeback_us,digest,collision_ready,swept_penetration,unresolved_contacts,illegal_trajectories,limited_trajectories,initial_overlaps,safety_stops,spatial_us,neighbor_peak,bucket_peak,neighbor_queries,neighbor_entries,pair_candidates,neighbor_truncated,dense_bucket_events,large_bounds,fallback_queries,reservation_count,frozen_targets,waiting_space,waiting_entrance,blocked_near_path,command_plan_us,formation_us,entry_refinements,arriving,stopped,replanning,failed,max_parked_drift\n");
    for m in samples {
        let timing=|v:u128|if m.timing_ready {v.to_string()} else {"not-ready".into()};
        let contacts=if m.collision_ready {format!("{:.6},{},{},{},{},{}",m.swept_penetration,
            m.unresolved_contacts,m.illegal_trajectories,m.limited_trajectories,m.initial_overlaps,m.safety_stops)}
            else {"not-ready,not-ready,not-ready,not-ready,not-ready,not-ready".into()};
        let s=&m.spatial;
        let spatial=if m.collision_ready {format!("{},{},{},{},{},{},{},{},{},{}",(s.build_time+s.query_time).as_micros(),s.peak_neighbors,
            s.peak_bucket,s.queries,s.visited_entries,s.candidate_pairs,s.truncated,s.dense_bucket_events,s.large_bounds,s.fallback_queries)}
            else {"not-ready,not-ready,not-ready,not-ready,not-ready,not-ready,not-ready,not-ready,not-ready,not-ready".into()};
        let formation=format!("{},{},{},{},{},{},{},{}",m.reserved,m.frozen_targets,m.waiting_space,m.waiting_entrance,m.blocked_near,m.planning_us,timing(m.formation_us),m.entry_refinements);
        let arrival=format!("{},{},{},{},{:.9}",m.arriving,m.waiting[0],m.replanning,m.failed,m.max_parked_drift);
        writeln!(csv,"{},{:.6},{},{},{},{},{},{:.6},{:.6},{},{},{},{},{},{},{},{:016x},{},{},{},{},{}",
            m.tick,m.tick as f64/TICK_HZ,m.active,m.moved,m.arrival_text(),m.waiting_text(),m.legacy_stalled,m.max_still_seconds,m.overlap_end,
            m.bad_centers,timing(m.total_us),timing(m.snapshot_us),timing(m.pressure_us),timing(m.velocity_us),timing(m.contact_us),timing(m.writeback_us),m.digest,
            m.collision_ready,contacts,spatial,formation,arrival).unwrap();
    }
    fs::write(path,csv).map_err(|e|e.to_string())?;
    fs::write(path.with_extension("scene.ini"),config.encode()).map_err(|e|e.to_string())?;
    let mut units:Vec<_>=world.query::<&Unit>().iter(world).cloned().collect();units.sort_by_key(|u|u.idx);
    let mut snapshot=String::from("id,group,command,x,y,vx,vy,goal_x,goal_y,still_ticks,stall_ticks,parking_x,parking_y,parking_valid,parking_frozen,target_block,motion,command_status,quiet_seconds,parked_drift\n");
    for u in units {
        let parking=u.parking.as_ref().map_or("none,none,false,false,none".into(),|t|format!("{:.9},{:.9},{},{},{:?}",t.position.x,t.position.y,t.valid,t.frozen,t.blocked));
        writeln!(snapshot,"{},{},{},{:.9},{:.9},{:.9},{:.9},{},{},{},{},{},{:?},{:?},{:.9},{:.9}",u.idx,
        u.idx as usize/config.per_group,u.command_id.map_or(0,|c|c.0),u.cell_x,u.cell_y,u.vx,u.vy,
        u.goal.x,u.goal.y,u.still_ticks,u.stall_ticks,parking,u.arrival.motion,crate::arrival::status(&u),u.arrival.quiet_seconds,u.arrival.max_drift).unwrap(); }
    fs::write(path.with_extension("units.csv"),snapshot).map_err(|e|e.to_string())?;
    Ok(())
}

pub fn headless(config:&SceneConfig,run:&RunOptions) -> Result<(),String> {
    let mut world=World::new(); populate(&mut world,config)?;
    let mut schedule=schedule();let mut samples=vec![metrics(&mut world)];
    let start=Instant::now();
    println!("scene={} mode={} groups={} per_group={} seed={} jitter={} hz={TICK_HZ} arrival={} waiting={}",
        config.kind.name(),config.mode(),config.kind.routes().len(),config.per_group,config.seed,config.jitter,samples[0].arrival_text(),samples[0].waiting_text());
    for _ in 0..run.ticks {
        schedule.run(&mut world);
        let tick=world.resource::<SimEnv>().tick;
        if tick%config.sample_every==0 || tick==run.ticks {
            let m=metrics(&mut world);
            println!("tick={} moving={} legacy_stalled={} overlap_end={:.4} arrived={} digest={:016x}",m.tick,m.moved,m.legacy_stalled,m.overlap_end,m.arrival_text(),m.digest);
            samples.push(m);
        }
    }
    if let Some(path)=&run.csv { write_report(&mut world,config,&samples,path)?; }
    let completed=world.resource::<SimEnv>().tick;let arrived=metrics(&mut world).arrival_text();
    println!("completed_ticks={} wall_ms={} arrived={}",completed,start.elapsed().as_millis(),arrived);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn configuration_roundtrips_and_rejects_changed_acceptance_geometry() {
        for kind in [SceneKind::Crossing,SceneKind::HeadOn,SceneKind::Single] {
            let c=SceneConfig {kind,seed:42,jitter:0.02,..default()};
            assert_eq!(SceneConfig::decode(&c.encode()).unwrap(),c);
            assert!(SceneConfig::decode(&c.encode().replace("TickHz=60","TickHz=30")).is_err());
            assert!(SceneConfig::decode(&c.encode().replace("Width=192","Width=oops")).is_err());
        }
    }
    #[test]
    fn presets_have_exact_centers_counts_and_simultaneous_orders() {
        for count in [64,256,1024] {
            let config=SceneConfig { per_group:count,..default() };let mut w=World::new();populate(&mut w,&config).unwrap();
            assert_eq!(w.resource::<SimEnv>().tick,0);
            let units:Vec<_>=w.query::<&Unit>().iter(&w).collect();assert_eq!(units.len(),4*count);
            for group in 0..4 {
                let g:Vec<_>=units.iter().filter(|u|u.idx as usize/count==group).collect();
                let start=config.kind.routes()[group].0;
                assert!((g.iter().map(|u|u.cell_x).sum::<f32>()/count as f32-start.x as f32).abs()<1e-3);
                assert!((g.iter().map(|u|u.cell_y).sum::<f32>()/count as f32-start.y as f32).abs()<1e-3);
                assert!(g.iter().all(|u|u.goal.x>=0&&u.command_id==g[0].command_id&&u.start_delay==0.0));
            }
        }
    }
    #[test]
    fn reset_restores_seed_commands_tick_and_positions() {
        let config=SceneConfig { seed:42,jitter:0.02,..default() };let mut w=World::new();populate(&mut w,&config).unwrap();
        let initial=metrics(&mut w).digest;let mut s=schedule();for _ in 0..120 {s.run(&mut w);}
        assert_ne!(metrics(&mut w).digest,initial);populate(&mut w,&config).unwrap();assert_eq!(metrics(&mut w).digest,initial);
        assert_eq!(w.resource::<SimEnv>().tick,0);
    }
    #[test]
    fn same_scenario_matches_direct_core_and_ecs() {
        let config=SceneConfig::default();let mut a=World::new();let mut b=World::new();
        populate(&mut a,&config).unwrap();populate(&mut b,&config).unwrap();let mut s=schedule();
        for _ in 0..120 {
            s.run(&mut a);
            let mut units:Vec<_>=b.query::<&Unit>().iter(&b).cloned().collect();
            crate::simulation::step_continuous(&mut b.resource_mut::<SimEnv>(),&mut units);
            let mut ordered:Vec<_>=b.query::<(Entity,&Unit)>().iter(&b).map(|(e,u)|(u.idx,e)).collect();ordered.sort_by_key(|x|x.0);
            for ((_,e),u) in ordered.into_iter().zip(units) {*b.get_mut::<Unit>(e).unwrap()=u;}
            assert_eq!(metrics(&mut a).digest,metrics(&mut b).digest);
        }
    }
}
