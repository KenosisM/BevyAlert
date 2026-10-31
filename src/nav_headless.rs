//! Asset-free entry point. All movement is performed by simulation::step_continuous.

use crate::nav::{self, Cell, CostGrid, FlowField};
use crate::simulation::{self, Runtime};
use crate::world::{SimEnv, Unit};
use std::sync::Arc;

/// Local deterministic input generator; never reads a clock or global RNG.
pub struct SeededInput(pub u64);

impl SeededInput {
    pub fn jitter(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 40) as f32 / 16777216.0 - 0.5) * 0.02
    }
}

pub fn initial_state(seed: u64, hz: f64) -> (SimEnv, Vec<Unit>) {
    let nav = CostGrid { w: 32, h: 24, cost: vec![1; 32 * 24], ..Default::default() };
    let goal = Cell::new(25, 12);
    let mut field = FlowField::default();
    assert!(nav::compute_flow_field_cpu_multi(&nav, &[goal], &mut field));
    let field = Arc::new(field);
    let mut runtime = Runtime::from_hz(hz, seed);
    let command = runtime.allocate_command();
    let mut random = SeededInput(seed);
    let units = (0..12).map(|idx| Unit {
        idx,
        command_id: Some(command),
        cell_x: 4.0 + (idx % 3) as f32 * 1.1 + random.jitter(),
        cell_y: 7.0 + (idx / 3) as f32 * 1.1 + random.jitter(),
        goal,
        guide: Some(field.clone()),
        ..Default::default()
    }).collect();
    // Retained explicitly as the historical pre-reservation steering fixture.
    (SimEnv { nav, runtime, velocity_mode:crate::crowd::VelocityMode::Legacy, ..Default::default() }, units)
}

pub fn run(args: &[String]) -> Result<(), String> {
    let (mut ticks, mut seed, mut hz, mut sample_every) = (240u64, 0u64, 60.0f64, 60u64);
    if args.len() % 2 != 0 { return Err("usage: nav-headless [--ticks N] [--seed N] [--hz N] [--sample-every N]".into()); }
    for pair in args.chunks_exact(2) {
        match pair[0].as_str() {
            "--ticks" => ticks = pair[1].parse().map_err(|_| "invalid tick count")?,
            "--seed" => seed = pair[1].parse().map_err(|_| "invalid seed")?,
            "--hz" => hz = pair[1].parse().map_err(|_| "invalid simulation rate")?,
            "--sample-every" => sample_every = pair[1].parse().map_err(|_| "invalid sampling interval")?,
            other => return Err(format!("unknown option: {other}")),
        }
    }
    if !hz.is_finite() || !(1.0..=10000.0).contains(&hz) { return Err("simulation rate must be in 1..=10000 Hz".into()); }
    let (mut env, mut units) = initial_state(seed, hz);
    env.runtime.sample_every = sample_every;
    println!("simulation=continuous-legacy seed={seed} hz={hz} units={} arrival=not-ready", units.len());
    for _ in 0..ticks {
        simulation::step_continuous(&mut env, &mut units);
        if sample_every > 0 && env.tick % sample_every == 0 {
            let s = env.runtime.last_sample.as_ref().unwrap();
            println!("tick={} moved={} near_goal={} arrived=not-ready simulation_us={}",
                s.tick, s.moved, s.near_goal, s.times.total.as_micros());
        }
    }
    for u in &units {
        println!("unit={} command={} x={:.6} y={:.6} vx={:.6} vy={:.6} status=active",
            u.idx, u.command_id.unwrap().0, u.cell_x, u.cell_y, u.vx, u.vy);
    }
    println!("completed_ticks={} arrived=not-ready", env.tick);
    Ok(())
}
