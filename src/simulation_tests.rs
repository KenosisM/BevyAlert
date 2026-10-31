use bevy::prelude::*;
use std::sync::Arc;

use crate::nav::{self, Cell, CostGrid, FlowField};
use crate::world::{self, SimEnv, Unit};

fn fixture() -> (SimEnv, Vec<Unit>) {
    let mut nav = CostGrid { w: 32, h: 24, cost: vec![1; 32 * 24], ..default() };
    for y in 0..18 {
        nav.cost[y * 32 + 15] = 0;
    }
    let mut field = FlowField::default();
    assert!(nav::compute_flow_field_cpu_multi(&nav, &[Cell::new(25, 12)], &mut field));
    let field = Arc::new(field);
    let units = (0..12).map(|idx| Unit {
        idx,
        cell_x: 4.0 + (idx % 3) as f32 * 1.1,
        cell_y: 7.0 + (idx / 3) as f32 * 1.1,
        goal: Cell::new(25, 12),
        guide: Some(field.clone()),
        ..default()
    }).collect();
    (SimEnv { nav, ..default() }, units)
}

#[test]
fn indexed_and_brute_force_formal_motion_match_through_contacts_and_bucket_updates() {
    let (mut fast,mut a)=fixture();let (mut reference,mut b)=fixture();
    reference.buffers.velocity_index.brute_force=true;
    reference.buffers.contacts.projection_index.brute_force=true;
    reference.buffers.contacts.trajectory_index.brute_force=true;
    for _ in 0..900 {
        crate::simulation::step_continuous(&mut fast,&mut a);
        crate::simulation::step_continuous(&mut reference,&mut b);
        for (x,y) in a.iter().zip(&b) {
            assert_eq!((x.cell_x.to_bits(),x.cell_y.to_bits(),x.vx.to_bits(),x.vy.to_bits(),x.stall_ticks),
                (y.cell_x.to_bits(),y.cell_y.to_bits(),y.vx.to_bits(),y.vy.to_bits(),y.stall_ticks));
        }
        assert_eq!(fast.runtime.contacts.reasons,reference.runtime.contacts.reasons);
        assert_eq!(fast.runtime.contacts.max_penetration,reference.runtime.contacts.max_penetration);
    }
    assert_eq!(fast.runtime.spatial_stats.truncated,0);
}

fn fingerprint(world: &mut World) -> u64 {
    let mut query = world.query::<&Unit>();
    let mut units: Vec<_> = query.iter(world).collect();
    units.sort_by_key(|u| u.idx);
    fingerprint_state(world.resource::<SimEnv>(), &units)
}

fn fingerprint_state(env: &SimEnv, units: &[&Unit]) -> u64 {
    let mut hash = 0xcbf29ce484222325u64;
    let mut feed = |n: u32| { hash = (hash ^ n as u64).wrapping_mul(0x100000001b3); };
    for u in units {
        for f in [u.cell_x, u.cell_y, u.vx, u.vy, u.dir_angle, u.dir_target,
            u.turret_angle, u.escape_side, u.leg_t] { feed(f.to_bits()); }
        for n in [u.idx as u32, u.still_ticks, u.stall_ticks, u.escape_ticks as u32,
            u.turret_idle as u32, u.goal.x as u32, u.goal.y as u32,
            u.cell_z as u32, u.ramp_type as u32, u.ground_dirty as u32] { feed(n); }
    }
    feed(env.tick as u32);
    feed(env.since_pressure);
    for p in &env.pressure.field { feed(p.to_bits()); }
    hash
}

#[test]
fn ecs_and_headless_match_each_tick_at_fixed_rates() {
    for hz in [30.0, 60.0, 120.0] {
        let (mut env, mut units) = crate::nav_headless::initial_state(42, hz);
        let (ecs_env, ecs_units) = crate::nav_headless::initial_state(42, hz);
        let mut world = World::new();
        world.insert_resource(ecs_env);
        for u in ecs_units.into_iter().rev() { world.spawn(u); }
        let mut schedule = Schedule::default();
        schedule.add_systems(world::update_sim_crowd);
        for tick in 1..=360 {
            crate::simulation::step_continuous(&mut env, &mut units);
            schedule.run(&mut world);
            let refs: Vec<_> = units.iter().collect();
            assert_eq!(fingerprint_state(&env, &refs), fingerprint(&mut world), "hz={hz} tick={tick}");
        }
    }
}

#[test]
fn seed_and_sampling_do_not_depend_on_wall_time() {
    let (mut env, mut units) = crate::nav_headless::initial_state(987, 60.0);
    let (mut sampled_env, mut sampled_units) = crate::nav_headless::initial_state(987, 60.0);
    sampled_env.runtime.sample_every = 7;
    let (_, different) = crate::nav_headless::initial_state(988, 60.0);
    assert_ne!(units[0].cell_x.to_bits(), different[0].cell_x.to_bits());
    for tick in 1..=210 {
        crate::simulation::step_continuous(&mut env, &mut units);
        crate::simulation::step_continuous(&mut sampled_env, &mut sampled_units);
        assert_eq!(fingerprint_state(&env, &units.iter().collect::<Vec<_>>()),
            fingerprint_state(&sampled_env, &sampled_units.iter().collect::<Vec<_>>()));
        if tick % 7 == 0 {
            let sample = sampled_env.runtime.last_sample.as_ref().unwrap();
            assert_eq!(sample.tick, tick);
            assert_eq!(sample.seed, 987);
            assert_eq!(sample.arrived, None);
            assert_eq!(sample.units.len(), units.len());
            for (s, u) in sample.units.iter().zip(&units) {
                assert_eq!(s.id, u.idx);
                assert_eq!(s.command, u.command_id);
                assert_eq!(s.status, crate::simulation::CommandStatus::Active);
                assert_eq!(s.position, (u.cell_x, u.cell_y));
                assert_eq!(s.velocity, (u.vx, u.vy));
            }
        }
    }
}

#[test]
fn movement_modes_cannot_write_the_same_tick() {
    for continuous in [false, true] {
        let make_world = || {
            let (mut env, units) = fixture();
            env.use_crowd = continuous;
            env.ground.w = 32;
            env.ground.h = 24;
            env.ground.level = vec![0; 32 * 24];
            let mut world = World::new();
            let mut time = Time::<Fixed>::from_duration(env.runtime.fixed_step());
            time.advance_by(env.runtime.fixed_step());
            world.insert_resource(time);
            world.insert_resource(env);
            for u in units { world.spawn(u); }
            world
        };
        let mut single_world = make_world();
        let mut combined_world = make_world();
        let mut single = Schedule::default();
        if continuous { single.add_systems(world::update_sim_crowd); }
        else { single.add_systems(world::update_sim); }
        let mut combined = Schedule::default();
        combined.add_systems((world::update_sim, world::update_sim_crowd).chain());
        for _ in 0..60 {
            single.run(&mut single_world);
            combined.run(&mut combined_world);
            assert_eq!(fingerprint(&mut single_world), fingerprint(&mut combined_world));
        }
        assert_eq!(combined_world.resource::<SimEnv>().tick, 60);
    }
}

#[test]
fn commands_share_an_id_and_reorders_allocate_a_new_one() {
    let (env, units) = fixture();
    let mut world = World::new();
    world.insert_resource(env);
    let entities: Vec<_> = units.into_iter().map(|u| world.spawn(u).id()).collect();
    world.insert_resource(crate::play::Selection(entities.clone()));
    assert_eq!(crate::play::issue_order_at(&mut world, Cell::new(24, 20)), 12);
    let id = world.get::<Unit>(entities[0]).unwrap().command_id.unwrap();
    assert!(entities.iter().all(|e| world.get::<Unit>(*e).unwrap().command_id == Some(id)));
    assert_eq!(crate::play::issue_order_at(&mut world, Cell::new(25, 21)), 12);
    let next = world.get::<Unit>(entities[0]).unwrap().command_id.unwrap();
    assert_ne!(id, next);
}

#[test]
#[should_panic(expected = "duplicate simulation unit ID")]
fn duplicate_unit_ids_are_rejected() {
    let (mut env, mut units) = crate::nav_headless::initial_state(0, 60.0);
    units[1].idx = units[0].idx;
    crate::simulation::step_continuous(&mut env, &mut units);
}
