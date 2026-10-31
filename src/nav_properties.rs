//! Independent small-graph reference and production movement regressions.
use crate::{nav::{self,Cell,CostGrid,FlowField,Footprint},world::{self,GroundField,SimEnv,Unit},play::{self,Selection}};
use bevy::prelude::*;
use std::sync::Arc;

fn grid(w:i32,h:i32,cost:Vec<u8>,cut:bool)->CostGrid {CostGrid{w,h,cost,cut_corners:cut}}

/// Floyd-Warshall over a separately constructed adjacency matrix.
fn reference(g:&CostGrid)->Vec<Vec<f64>> {
    let count=(g.w*g.h) as usize;let mut d=vec![vec![f64::INFINITY;count];count];
    let free=|x:i32,y:i32|x>=0&&y>=0&&x<g.w&&y<g.h&&g.cost[(y*g.w+x) as usize]>0;
    for i in 0..count {
        let x=i as i32%g.w;let y=i as i32/g.w;if !free(x,y){continue;}d[i][i]=0.0;
        for dy in -1i32..=1 {for dx in -1i32..=1 {
            if dx==0&&dy==0 {continue;}let (nx,ny)=(x+dx,y+dy);if !free(nx,ny){continue;}
            if dx!=0&&dy!=0 {
                let a=free(nx,y);let b=free(x,ny);
                if (g.cut_corners&&!(a||b))||(!g.cut_corners&&!(a&&b)){continue;}
            }
            let j=(ny*g.w+nx) as usize;
            d[i][j]=g.cost[j] as f64*if dx!=0&&dy!=0{2.0f64.sqrt()}else{1.0};
        }}
    }
    for k in 0..count {for i in 0..count {for j in 0..count {d[i][j]=d[i][j].min(d[i][k]+d[k][j]);}}}d
}

#[test]
fn weighted_astar_and_fields_match_independent_all_pairs_costs() {
    for cut in [false,true] {for seed in 0..24u64 {
        let mut state=seed+1;let cost=(0..20).map(|_|{state=state.wrapping_mul(6364136223846793005).wrapping_add(1);let r=(state>>32)%11;if r<2{0}else{(r-1) as u8}}).collect();
        let g=grid(5,4,cost,cut);let expected=reference(&g);
        let sources:Vec<_>=(0..20).filter(|i|g.cost[*i]>0).step_by(4).map(|i|Cell::new(i as i32%5,i as i32/5)).collect();
        let mut multi=FlowField::default();let ok=nav::compute_flow_field_cpu_multi(&g,&sources,&mut multi);
        assert_eq!(ok,!sources.is_empty());
        if ok {for from in 0..20 {
            let best=sources.iter().map(|s|expected[from][(s.y*5+s.x) as usize]).fold(f64::INFINITY,f64::min);
            let found=multi.dist_at(from as i32%5,from as i32/5) as f64;
            if best.is_finite(){assert!((found-best).abs()<1e-4*(1.0+best));}else{assert!(found>=1e29);}
        }}
        for goal in 0..20 {
            let target=Cell::new(goal%5,goal/5);let mut ff=FlowField::default();
            assert_eq!(nav::compute_flow_field_cpu(&g,target,&mut ff),g.cost[goal as usize]>0);
            for from in 0..20 {
                let start=Cell::new(from%5,from/5);let result=nav::find_path(&g,start,target);
                let distance=expected[from as usize][goal as usize];
                assert_eq!(result.found,distance.is_finite(),"seed={seed} cut={cut} {start:?}->{target:?}");
                if distance.is_finite() {
                    let cost:f64=result.cells.windows(2).map(|p|nav::step_cost(&g,p[0],p[1]).unwrap() as f64).sum();
                    assert!((cost-distance).abs()<1e-4*(1.0+distance));
                    assert!((ff.dist_at(start.x,start.y) as f64-distance).abs()<1e-4*(1.0+distance));
                    let mut reached=Cell::default();let mut path=Vec::new();
                    assert!(nav::path_to_nearest_source(&g,&ff,start,&mut reached,&mut path));assert_eq!(reached,target);
                    let flow_cost:f64=path.windows(2).map(|p|nav::step_cost(&g,p[0],p[1]).unwrap() as f64).sum();
                    assert!((flow_cost-distance).abs()<1e-4*(1.0+distance));
                } else if g.cost[goal as usize]>0 {assert!(ff.dist_at(start.x,start.y)>=1e29);}
            }
        }
    }}
}

#[test]
fn corner_rules_boundaries_dynamic_sides_and_footprints_are_shared() {
    let mut g=grid(2,2,vec![1,0,0,1],false);
    for cut in [false,true] {g.cut_corners=cut;assert_eq!(nav::step_cost(&g,Cell::new(0,0),Cell::new(1,1)),None);
        assert!(!nav::segment_allowed(&g,(0.0,0.0),(1.0,1.0)));}
    g.cost[1]=1;g.cut_corners=false;assert!(nav::step_cost(&g,Cell::new(0,0),Cell::new(1,1)).is_none());
    g.cut_corners=true;assert!(nav::step_cost(&g,Cell::new(0,0),Cell::new(1,1)).is_some());
    assert!(nav::step_cost_extra(&g,Cell::new(0,0),Cell::new(1,1),&|x,y|x==1&&y==0).is_none());
    assert!(nav::step_cost(&g,Cell::new(0,0),Cell::new(-1,0)).is_none());
    assert!(nav::step_cost(&g,Cell::new(i32::MIN,0),Cell::new(i32::MAX,0)).is_none());
    let raw=grid(4,3,vec![1,1,0,1,1,1,0,1,1,1,0,1],false);
    let wide=nav::erode_grid(&raw,&Footprint{w:2,h:1,..default()});
    assert!(wide.walkable(0,1));assert!(!wide.walkable(1,1));assert!(!wide.walkable(3,1));
    let mut ff=FlowField::default();assert!(nav::compute_flow_field_cpu(&wide,Cell::new(0,2),&mut ff));
    assert!(nav::find_path(&wide,Cell::new(0,0),Cell::new(0,2)).found);
}

#[test]
fn invalid_recomputations_clear_fields_and_failed_routes_clear_outputs() {
    let g=grid(3,1,vec![1,0,1],false);let mut ff=FlowField::default();
    assert!(nav::compute_flow_field_cpu(&g,Cell::new(0,0),&mut ff));
    assert!(!nav::compute_flow_field_cpu(&g,Cell::new(1,0),&mut ff));
    assert!(ff.sources.is_empty());assert!(ff.is_source.iter().all(|x|*x==0));assert!(ff.dir_x.iter().all(|x|*x==0));
    assert!(nav::compute_flow_field_cpu_multi(&g,&[Cell::new(-1,0),Cell::new(2,0),Cell::new(2,0)],&mut ff));assert_eq!(ff.sources,vec![Cell::new(2,0)]);
    let mut reached=Cell::new(123,123);let mut path=vec![Cell::new(123,123)];
    assert!(!nav::path_to_nearest_source(&g,&ff,Cell::new(0,0),&mut reached,&mut path));assert!(path.is_empty());assert_eq!(reached,Cell::new(-1,-1));
    assert!(!nav::compute_flow_field_cpu(&CostGrid::default(),Cell::new(0,0),&mut ff));assert_eq!(ff.w,0);assert!(ff.dist.is_empty());
    let partial=grid(2,2,vec![1],false);let mut partial_path=Vec::new();
    assert!(!nav::find_path_hierarchical(&partial,Cell::new(0,0),Cell::new(0,0),&mut partial_path,2));
}

#[test]
fn orders_reject_unreachable_and_invalid_units_and_do_not_double_count_selection() {
    let mut w=World::new();w.insert_resource(SimEnv{nav:grid(5,1,vec![1,1,0,1,1],false),..default()});
    let a=w.spawn(Unit{idx:0,cell_x:0.0,cell_y:0.0,..default()}).id();let b=w.spawn(Unit{idx:1,cell_x:4.0,cell_y:0.0,..default()}).id();
    let invalid=w.spawn(Unit{idx:2,cell_x:100.0,cell_y:0.0,..default()}).id();
    w.insert_resource(Selection(vec![a,a,b,invalid]));
    assert_eq!(play::issue_order_at(&mut w,Cell::new(4,0)),1);assert!(w.get::<Unit>(a).unwrap().command_id.is_none());
    assert!(w.get::<Unit>(b).unwrap().command_id.is_some());assert!(w.get::<Unit>(invalid).unwrap().command_id.is_none());
    assert_eq!(play::issue_order_at(&mut w,Cell::new(-1,0)),0);
    assert_eq!(play::nearest_usable(&grid(2,1,vec![0,0],false),Cell::new(0,0)),None);
}

#[test]
fn actual_legacy_step_and_continuous_integration_refuse_a_sealed_corner() {
    let g=grid(2,2,vec![1,0,0,1],false);
    let fake=FlowField{w:2,h:2,dist:vec![2.0,1.0,1.0,0.0],dir_x:vec![1,0,0,0],dir_y:vec![1,0,0,0],is_source:vec![0,0,0,1],sources:vec![Cell::new(1,1)]};
    let mut world=World::new();let mut t=Time::<Fixed>::from_hz(60.0);t.advance_by(std::time::Duration::from_secs_f64(1.0/60.0));world.insert_resource(t);
    world.insert_resource(SimEnv{nav:g.clone(),ground:GroundField{w:2,h:2,level:vec![0;4],..default()},use_crowd:false,..default()});
    let e=world.spawn(Unit{idx:0,goal:Cell::new(1,1),guide:Some(Arc::new(fake)),..default()}).id();let mut s=Schedule::default();s.add_systems(world::update_sim);
    for _ in 0..120{s.run(&mut world);}let u=world.get::<Unit>(e).unwrap();assert_eq!((u.cell_x,u.cell_y),(0.0,0.0));
    let p=crate::crowd::resolve_position(&g,(0.0,0.0),(1.0,1.0));
    assert!(p.0<0.06&&p.1<0.06);assert!(nav::segment_allowed(&g,(0.0,0.0),p));
    let g=grid(5,1,vec![1,1,0,1,1],false);let p=crate::crowd::resolve_position(&g,(0.0,0.0),(4.0,0.0));
    assert!(p.0<1.051&&p.1==0.0);assert!(nav::segment_allowed(&g,(0.0,0.0),p));
}

#[test]
fn production_continuous_unit_reaches_a_goal_around_a_wall_without_illegal_edges() {
    let mut cost=vec![1;35];for y in 0..4 {cost[y*7+3]=0;}
    let g=grid(7,5,cost,false);let goal=Cell::new(5,1);let mut ff=FlowField::default();
    assert!(nav::compute_flow_field_cpu(&g,goal,&mut ff));
    let mut env=SimEnv{nav:g.clone(),..default()};let mut units=vec![Unit{idx:0,cell_x:1.0,cell_y:1.0,goal,guide:Some(Arc::new(ff)),..default()}];
    for _ in 0..600 {
        let old=(units[0].cell_x,units[0].cell_y);crate::simulation::step_continuous(&mut env,&mut units);
        assert!(nav::segment_allowed(&g,old,(units[0].cell_x,units[0].cell_y)));
    }
    assert!((units[0].cell_x-5.0).hypot(units[0].cell_y-1.0)<0.7,"unit at {},{}",units[0].cell_x,units[0].cell_y);
}
