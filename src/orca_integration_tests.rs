use bevy::prelude::*;
use crate::{nav::{Cell,CostGrid},world::{SimEnv,Unit},play::{self,Selection}};

fn world(grid:CostGrid)->World {
    let mut w=World::new();let env=SimEnv{nav:grid,..default()};
    let mut time=Time::<Fixed>::from_duration(env.runtime.fixed_step());time.advance_by(env.runtime.fixed_step());
    w.insert_resource(time);w.insert_resource(env);w.insert_resource(Selection::default());w
}
fn open()->CostGrid {CostGrid{w:48,h:32,cost:vec![1;48*32],..default()}}
fn run(w:&mut World,ticks:usize)->(usize,usize) {
    let mut schedule=crate::nav_scenario::schedule();let mut fallbacks=0;let mut infeasible=0;
    for _ in 0..ticks {schedule.run(w);let env=w.resource::<SimEnv>();
        let c=&env.runtime.contacts;assert!(c.max_penetration<=0.02&&c.max_swept_penetration<=0.02);
        assert_eq!((c.initial_overlaps,c.illegal_static_trajectories,c.unresolved_pairs),(0,0,0));
        assert_eq!((env.runtime.orca.invalid,env.runtime.orca.truncated),(0,0));
        fallbacks+=env.runtime.orca.contact_fallback;infeasible+=env.runtime.orca.infeasible;
    }(fallbacks,infeasible)
}
fn complete(w:&World,ids:&[Entity]) {for &id in ids {let u=w.get::<Unit>(id).unwrap();
    assert!(u.arrival.settled(),"id={} pos=({},{}), parking={:?}, motion={:?}",u.idx,u.cell_x,u.cell_y,u.parking,u.arrival.motion);}}

#[test]
fn actual_symmetric_head_on_and_two_teams_complete_without_overlap() {
    for team in [1,8] {
        let mut w=world(open());let mut a=Vec::new();let mut b=Vec::new();
        for i in 0..team {
            a.push(w.spawn(Unit{idx:i,cell_x:7.0,cell_y:10.0+i as f32*1.1,..default()}).id());
            b.push(w.spawn(Unit{idx:team+i,cell_x:38.0,cell_y:10.0+i as f32*1.1,..default()}).id());
        }
        let reports=play::issue_orders_at(&mut w,&[(a.clone(),Cell::new(38,14)),(b.clone(),Cell::new(7,14))]);
        assert!(reports.iter().all(|r|r.accepted==team as usize));
        let (fallbacks,infeasible)=run(&mut w,1800);complete(&w,&a);complete(&w,&b);
        println!("orca_head_on team={team} fallbacks={fallbacks} infeasible={infeasible}");
    }
}
#[test]
fn actual_wall_following_one_cell_corridor_and_parked_obstacle_complete() {
    // The disc is not replaced by a tile's bounding circle: width one stays open.
    let mut grid=open();for y in 0..32 {for x in 0..48 {if y!=12 {grid.cost[y*48+x]=0;}}}
    let mut w=world(grid);let a=w.spawn(Unit{idx:0,cell_x:4.0,cell_y:12.0,..default()}).id();
    w.resource_mut::<Selection>().0=vec![a];assert_eq!(play::issue_order_at(&mut w,Cell::new(42,12)),1);
    let report=run(&mut w,1500);complete(&w,&[a]);println!("orca_corridor {report:?}");
    let mut w=world(open());let parked=w.spawn(Unit{idx:0,cell_x:22.0,cell_y:16.0,..default()}).id();
    let a=w.spawn(Unit{idx:1,cell_x:5.0,cell_y:16.0,..default()}).id();
    w.resource_mut::<Selection>().0=vec![parked];assert_eq!(play::issue_order_at(&mut w,Cell::new(22,16)),1);
    run(&mut w,60);let fixed=w.get::<Unit>(parked).unwrap().clone();assert!(fixed.arrival.settled());
    w.resource_mut::<Selection>().0=vec![a];assert_eq!(play::issue_order_at(&mut w,Cell::new(40,16)),1);
    let report=run(&mut w,1500);complete(&w,&[a,parked]);
    let after=w.get::<Unit>(parked).unwrap();assert_eq!((after.cell_x,after.cell_y),(fixed.cell_x,fixed.cell_y));
    println!("orca_parked {report:?}");
}
#[test]
fn objective_smoothing_does_not_limit_emergency_solver_velocity() {
    use crate::crowd::*;
    let p=smooth_preferred(Some(Vec2::X*6.0),-Vec2::X*6.0,36.0,1.0/60.0,6.0);
    assert!((p.x-5.4).abs()<1e-6);
    let agents=[Agent{x:4.0,y:5.0,vx:6.0,..default()},Agent{x:5.0,y:5.0,settled:true,..default()}];
    let mut work=OrcaWorkspace::default();work.boundaries.refresh(&open());let mut report=OrcaReport::default();
    let v=work.velocity(0,&agents,&[0,1],p,6.0,1.0/60.0,&mut report);
    assert!(v.distance(Vec2::X*6.0)>0.6);assert_eq!(report.emergency,1);assert_eq!(report.full_responsibility,1);
    assert!(v.is_finite());
}
