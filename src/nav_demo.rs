//! Four-colour view and controls over the same scenario and simulation as the CLI.

use bevy::{app::AppExit, prelude::*,render::view::screenshot::{save_to_disk,Screenshot,ScreenshotCaptured}};
use crate::{nav_scenario::{self, Metrics, RunOptions, SceneConfig, SceneKind}, world::{self, SimEnv, Unit}};

const SCALE:f32=5.0;
const COLORS:[Color;4]=[
    Color::srgb(1.0,0.27,0.25),Color::srgb(0.2,0.6,1.0),
    Color::srgb(0.2,0.88,0.45),Color::srgb(1.0,0.78,0.18),
];

#[derive(Resource)]
struct DemoState {
    config:SceneConfig, run:RunOptions, paused:bool, single_step:bool,
    latest:Metrics, samples:Vec<Metrics>, frames:u32, frame_seconds:f32, fps:f32,
    render_frames:u64,capture_requested:bool,capture_finished:bool,
    parked_since:Option<u64>,
}

#[derive(Resource,Clone)]
struct Graphics { circle:Handle<Mesh>, materials:Vec<Handle<ColorMaterial>> }
#[derive(Component)]
struct Hud;
#[derive(Component)]
struct GoalMarker;
#[derive(Component)]
struct NavBlock;

pub fn run(args:&[String]) -> Result<(),String> {
    let (config,options)=nav_scenario::parse_args(args)?;
    if options.headless { return nav_scenario::headless(&config,&options); }
    if let Some(parent)=options.capture.as_ref().and_then(|p|p.parent()).filter(|p|!p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent).map_err(|e|format!("cannot create capture output directory: {e}"))?;
    }
    let paused=options.paused;
    let mut app=App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window:Some(Window { title:"ra2engine - navigation demo".into(),resolution:(1200,960).into(),..default() }),
        ..default()
    }));
    app.insert_resource(ClearColor(Color::srgb(0.025,0.035,0.05)));
    app.insert_resource(DemoState { config,run:options,paused,single_step:false,latest:Metrics::default(),
        samples:Vec::new(),frames:0,frame_seconds:0.0,fps:0.0,render_frames:0,capture_requested:false,capture_finished:false,parked_since:None });
    app.add_systems(Startup,setup);
    add_fixed_systems(&mut app);
    app.add_systems(Update,(input,update_positions,update_hud,capture_frame).chain());
    app.run(); Ok(())
}

fn add_fixed_systems(app:&mut App) {
    app.add_systems(FixedUpdate,(world::update_sim,world::update_sim_crowd,observe_tick).chain().run_if(can_step));
}

/// Exports the application's own render target, without capturing the desktop.
fn capture_frame(world:&mut World) {
    let (capture,exit)={
        let mut s=world.resource_mut::<DemoState>();s.render_frames+=1;
        let capture=if s.render_frames>=30&&!s.capture_requested {
            let path=s.run.capture.clone();if path.is_some(){s.capture_requested=true;}path
        }else{None};
        let exit=s.run.exit_after_frames.is_some_and(|n|s.render_frames>=n)
            &&(s.run.capture.is_none()||s.capture_finished);
        (capture,exit)
    };
    if let Some(path)=capture {
        world.spawn(Screenshot::primary_window()).observe(save_to_disk(path))
            .observe(|_:On<ScreenshotCaptured>,mut state:ResMut<DemoState>|{state.capture_finished=true;println!("event=render-captured");});
    }
    if exit {save(world);world.write_message(AppExit::Success);}
}

fn position(x:f32,y:f32,z:f32)->Vec3 {
    Vec3::new((x-nav_scenario::WIDTH as f32*0.5)*SCALE,
        (nav_scenario::HEIGHT as f32*0.5-y)*SCALE-45.0,z)
}

fn setup(world:&mut World) {
    world.spawn(Camera2d);
    world.spawn((Sprite::from_color(Color::srgb(0.075,0.095,0.12),
        Vec2::new(nav_scenario::WIDTH as f32*SCALE,nav_scenario::HEIGHT as f32*SCALE)),
        Transform::from_xyz(0.0,-45.0,-2.0)));
    let radius=crate::crowd::Params::default().radius_default();
    let circle=world.resource_mut::<Assets<Mesh>>().add(Circle::new(radius*SCALE));
    let materials=COLORS.iter().map(|c|world.resource_mut::<Assets<ColorMaterial>>().add(*c)).collect();
    world.insert_resource(Graphics { circle,materials });
    world.spawn((Hud,Text::new("Preparing navigation scenario"),TextFont {font_size:16.0.into(),..default()},
        TextColor(Color::srgb(0.92,0.95,1.0)),
        Node { position_type:PositionType::Absolute,left:Val::Px(18.0),top:Val::Px(12.0),width:Val::Px(1164.0),..Node::DEFAULT }));
    reset(world);
}

fn reset(world:&mut World) {
    let config=world.resource::<DemoState>().config.clone();
    nav_scenario::populate(world,&config).expect("validated navigation scenario");
    let graphics=world.resource::<Graphics>().clone();
    let units:Vec<_>=world.query::<(Entity,&Unit)>().iter(world)
        .map(|(e,u)|(e,u.idx as usize/config.per_group,u.cell_x,u.cell_y)).collect();
    for (entity,group,x,y) in units {
        world.entity_mut(entity).insert((Mesh2d(graphics.circle.clone()),MeshMaterial2d(graphics.materials[group].clone()),
            Transform::from_translation(position(x,y,1.0))));
    }
    let markers:Vec<_>=world.query_filtered::<Entity,Or<(With<GoalMarker>,With<NavBlock>)>>().iter(world).collect();
    for e in markers { world.despawn(e); }
    let grid=world.resource::<SimEnv>().nav.clone();
    for y in 0..grid.h {let mut x=0;while x<grid.w {if grid.walkable(x,y) {x+=1;continue;}let first=x;
        while x<grid.w&&!grid.walkable(x,y) {x+=1;}
        world.spawn((NavBlock,Sprite::from_color(Color::srgb(0.3,0.34,0.4),Vec2::new((x-first) as f32*SCALE,SCALE)),
            Transform::from_translation(position((first+x-1) as f32*0.5,y as f32,-1.0))));
    }}
    for (group,(_,goal)) in config.kind.routes().iter().enumerate() {
        world.spawn((GoalMarker,Sprite::from_color(COLORS[group].with_alpha(0.25),Vec2::splat(9.0)),
            Transform::from_translation(position(goal.x as f32,goal.y as f32,0.0))));
    }
    let initial=nav_scenario::metrics(world);
    let mut state=world.resource_mut::<DemoState>();state.single_step=false;state.latest=initial.clone();state.samples=vec![initial];state.parked_since=None;
    println!("event=reset scene={} per_group={} seed={} jitter={} tick=0 digest={:016x}",
        config.kind.name(),config.per_group,config.seed,config.jitter,state.latest.digest);
}

fn can_step(state:Res<DemoState>)->bool { !state.paused||state.single_step }

fn observe_tick(world:&mut World) {
    let tick=world.resource::<SimEnv>().tick;
    let parked=nav_scenario::all_parked(world);
    {let mut s=world.resource_mut::<DemoState>();if parked {s.parked_since.get_or_insert(tick);}else{s.parked_since=None;}}
    let sample={let s=world.resource::<DemoState>();tick%s.config.sample_every==0||s.single_step};
    if sample {
        let m=nav_scenario::metrics(world);
        let mut state=world.resource_mut::<DemoState>();state.latest=m.clone();state.samples.push(m);
    }
    let (single,exit)={let s=world.resource::<DemoState>();(s.single_step,s.run.exit_after_ticks.is_some_and(|t|tick>=t)
        ||s.run.stop_after_parked.is_some_and(|hold|s.parked_since.is_some_and(|since|tick-since>=hold)))};
    if single { println!("event=single-step tick={tick} digest={:016x}",world.resource::<DemoState>().latest.digest); }
    world.resource_mut::<DemoState>().single_step=false;
    if exit {
        // A render frame can contain several fixed ticks. Stop at the requested one.
        world.resource_mut::<DemoState>().paused=true;
        save(world); world.write_message(AppExit::Success);
    }
}

fn save(world:&mut World) {
    let (config,mut samples,path)={let s=world.resource::<DemoState>();(s.config.clone(),s.samples.clone(),
        s.run.csv.clone().unwrap_or_else(||"_scratch/nav-demo.csv".into()))};
    let final_sample=nav_scenario::metrics(world);
    if samples.last().is_none_or(|s|s.tick!=final_sample.tick) { samples.push(final_sample); }
    match nav_scenario::write_report(world,&config,&samples,&path) {
        Ok(())=>{
            let m=nav_scenario::metrics(world);
            println!("event=saved tick={} digest={:016x}",m.tick,m.digest);
        }
        Err(error)=>eprintln!("cannot save navigation report: {error}"),
    }
}

fn input(world:&mut World) {
    let keys=world.resource::<ButtonInput<KeyCode>>();
    let (pause,step,restart,save_now,exit,cancel,stop,wake)=(keys.just_pressed(KeyCode::Space),keys.just_pressed(KeyCode::KeyN),
        keys.just_pressed(KeyCode::KeyR),keys.just_pressed(KeyCode::KeyS),keys.just_pressed(KeyCode::Escape),keys.just_pressed(KeyCode::KeyX),
        keys.just_pressed(KeyCode::KeyH),keys.just_pressed(KeyCode::KeyU));
    let count=if keys.just_pressed(KeyCode::Digit1) {Some(64)} else if keys.just_pressed(KeyCode::Digit2) {Some(256)}
        else if keys.just_pressed(KeyCode::Digit3) {Some(1024)} else {None};
    let kind=if keys.just_pressed(KeyCode::F1) {Some(SceneKind::Crossing)} else if keys.just_pressed(KeyCode::F2) {Some(SceneKind::HeadOn)}
        else if keys.just_pressed(KeyCode::F3) {Some(SceneKind::Single)} else if keys.just_pressed(KeyCode::F4) {Some(SceneKind::SingleObstacle)}
        else if keys.just_pressed(KeyCode::F5) {Some(SceneKind::SingleWidth)} else {None};
    let mode=keys.just_pressed(KeyCode::Tab);
    {
        let mut state=world.resource_mut::<DemoState>();
        if pause {state.paused=!state.paused;println!("event=pause paused={}",state.paused);}
        if step {state.paused=true;state.single_step=true;}
        if let Some(n)=count {state.config.per_group=n;}
        if let Some(k)=kind {state.config.kind=k;}
        if mode {state.config.continuous=!state.config.continuous;}
    }
    if restart||count.is_some()||kind.is_some()||mode {reset(world);}
    if cancel||stop||wake {
        world.resource_mut::<crate::play::Selection>().0=world.query_filtered::<Entity,With<Unit>>().iter(world).collect();
        let (n,action)=if cancel {(crate::play::cancel_selected(world),"cancel")}
            else if stop {(crate::play::stop_selected(world),"stop")}
            else {(crate::play::wake_selected(world,crate::arrival::WakeReason::Resume),"resume")};
        world.resource_mut::<crate::play::Selection>().0.clear();
        let latest=nav_scenario::metrics(world);world.resource_mut::<DemoState>().latest=latest;
        println!("event={action} units={n}");
    }
    if save_now {save(world);}
    if exit {world.write_message(AppExit::Success);}
}

fn update_positions(mut units:Query<(&Unit,&mut Transform)>) {
    for (unit,mut transform) in &mut units {transform.translation=position(unit.cell_x,unit.cell_y,1.0);}
}

fn update_hud(time:Res<Time>,env:Res<SimEnv>,mut state:ResMut<DemoState>,mut text:Query<&mut Text,With<Hud>>) {
    state.frames+=1;state.frame_seconds+=time.delta_secs();
    if state.frames>=60 {state.fps=state.frames as f32/state.frame_seconds.max(1e-4);state.frames=0;state.frame_seconds=0.0;}
    let m=&state.latest;
    let core=if m.timing_ready {format!("{:.3} ms",m.total_us as f64/1000.0)} else {"NOT READY".into()};
    let fps=if state.fps>0.0 {format!("{:.1} FPS",state.fps)} else {"warming up".into()};
    if let Ok(mut text)=text.single_mut() {
        text.0=format!("Navigation | {} | {} | {} | {} units | seed {} jitter {} | tick {} | {}\n\
            Space pause | N step | R reset | 1/2/3=64/256/1024 | F1 cross F2 head-on F3 single F4 wall F5 width | Tab mode | S save | H stop U resume X cancel | Esc close\n\
            A red  B blue  C green  D gold | commands active {} | moving {} | legacy stalls {} | max still {:.2}s\n\
            Arrived {} | approaching {} | stopped {} | replanning {} | failed {} | drift {:.4} | sample tick {} | overlap(end) {:.4} | core {}\n\
            Reserved {} | frozen {} | space wait {} | entrance wait {} | blocked local path {} | digest {:016x}",
            state.config.kind.name(),state.config.mode(),if state.paused {"PAUSED"} else {"RUNNING"},
            state.config.per_group*state.config.kind.routes().len(),state.config.seed,state.config.jitter,env.tick,fps,
            m.active,m.moved,m.legacy_stalled,m.max_still_seconds,m.arrival_text(),m.arriving,m.waiting[0],m.replanning,m.failed,m.max_parked_drift,m.tick,m.overlap_end,core,
            m.reserved,m.frozen_targets,m.waiting_space,m.waiting_entrance,m.blocked_near,m.digest);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn app()->App {
        let mut app=App::new();let (config,mut run)=nav_scenario::parse_args(&[]).unwrap();run.paused=true;
        app.insert_resource(Assets::<Mesh>::default());app.insert_resource(Assets::<ColorMaterial>::default());
        app.insert_resource(DemoState {config,run,paused:true,single_step:false,latest:Metrics::default(),samples:Vec::new(),
            frames:0,frame_seconds:0.0,fps:0.0,render_frames:0,capture_requested:false,capture_finished:false,parked_since:None});
        add_fixed_systems(&mut app);setup(app.world_mut());app
    }
    fn press(app:&mut App,key:KeyCode) {
        let mut keys=ButtonInput::default();keys.press(key);app.insert_resource(keys);input(app.world_mut());
    }
    #[test]
    fn real_control_system_pauses_steps_and_restores_original_state() {
        let mut app=app();let initial=nav_scenario::metrics(app.world_mut()).digest;
        app.world_mut().run_schedule(FixedUpdate);assert_eq!(app.world().resource::<SimEnv>().tick,0);
        press(&mut app,KeyCode::KeyN);app.world_mut().run_schedule(FixedUpdate);
        assert_eq!(app.world().resource::<SimEnv>().tick,1);
        app.world_mut().run_schedule(FixedUpdate);assert_eq!(app.world().resource::<SimEnv>().tick,1);
        press(&mut app,KeyCode::Space);app.world_mut().run_schedule(FixedUpdate);
        assert_eq!(app.world().resource::<SimEnv>().tick,2);
        press(&mut app,KeyCode::Space);app.world_mut().run_schedule(FixedUpdate);
        assert_eq!(app.world().resource::<SimEnv>().tick,2);
        press(&mut app,KeyCode::KeyR);assert_eq!(app.world().resource::<SimEnv>().tick,0);
        assert_eq!(nav_scenario::metrics(app.world_mut()).digest,initial);
    }
    #[test]
    fn real_control_system_switches_count_scene_and_legacy_mode() {
        let mut app=app();press(&mut app,KeyCode::Digit2);
        assert_eq!(app.world_mut().query::<&Unit>().iter(app.world()).count(),1024);
        press(&mut app,KeyCode::F2);
        assert_eq!(app.world_mut().query::<&Unit>().iter(app.world()).count(),512);
        press(&mut app,KeyCode::F3);assert_eq!(app.world_mut().query::<&Unit>().iter(app.world()).count(),256);
        press(&mut app,KeyCode::Tab);assert!(!app.world().resource::<SimEnv>().use_crowd);
        press(&mut app,KeyCode::Tab);assert!(app.world().resource::<SimEnv>().use_crowd);
        let reserved=app.world().resource::<SimEnv>().reservations.claims.len();
        press(&mut app,KeyCode::KeyH);assert!(app.world_mut().query::<&Unit>().iter(app.world()).all(|u|u.arrival.held()));
        assert_eq!(app.world().resource::<SimEnv>().reservations.claims.len(),reserved);
        press(&mut app,KeyCode::KeyU);assert!(app.world_mut().query::<&Unit>().iter(app.world()).all(|u|!u.arrival.held()));
        press(&mut app,KeyCode::KeyX);assert!(app.world().resource::<SimEnv>().reservations.claims.is_empty());
        assert!(app.world_mut().query::<&Unit>().iter(app.world()).all(|u|u.command_id.is_none()));
        press(&mut app,KeyCode::KeyR);assert!(!app.world().resource::<SimEnv>().reservations.claims.is_empty());
        press(&mut app,KeyCode::Digit1);press(&mut app,KeyCode::F1);
        assert_eq!(app.world_mut().query::<&Unit>().iter(app.world()).count(),256);
        assert_eq!(app.world().resource::<SimEnv>().tick,0);
    }
    #[test]
    fn bounded_window_run_stops_at_the_exact_fixed_tick() {
        let mut app=app();
        {
            let mut state=app.world_mut().resource_mut::<DemoState>();
            state.paused=false;state.run.exit_after_ticks=Some(5);
            state.run.csv=Some(std::env::temp_dir().join(format!("nav-demo-exit-{}.csv",std::process::id())));
        }
        for _ in 0..20 {app.world_mut().run_schedule(FixedUpdate);}
        assert_eq!(app.world().resource::<SimEnv>().tick,5);
    }
}
