// Copyright 2008 University of North Carolina at Chapel Hill
// SPDX-License-Identifier: Apache-2.0
//
// Adapted from RVO2 v2.0.2, Agent.cpp, commit
// c96a6ec8328492bae0721e59774026a119a60b4a. See docs/ORCA.md and
// licenses/RVO2-APACHE-2.0.txt. Authors: Jur van den Berg, Stephen J. Guy,
// Jamie Snape, Ming C. Lin, and Dinesh Manocha.
// Modified: Rust/f64, checked inputs, deterministic coincident centres,
// passive-neighbor responsibility, reusable buffers, explicit infeasibility.
// Licensed under the Apache License, Version 2.0; you may not use this file
// except in compliance with that License. Distributed on an AS IS BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

//! Pure velocity constraints. No ECS, position writes, terrain copy or post-solve steering.
use bevy::math::DVec2;

const PARALLEL_EPS:f64=1e-12;
pub const FEASIBILITY_TOLERANCE:f64=1e-8;
fn det(a:DVec2,b:DVec2)->f64 {a.x*b.y-a.y*b.x}
fn left(a:DVec2)->DVec2 {DVec2::new(-a.y,a.x)}
fn length(a:DVec2)->f64 {a.x.hypot(a.y)}
fn bounded(v:DVec2,r:f64)->DVec2 {let n=length(v);if n>r&&n>0.0 {v/n*r}else{v}}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum InputError {NonFinite,NonPositiveTime,InvalidRadius,ZeroNormal,DuplicateId,InvalidHardCount,MagnitudeOverflow}

/// Allowed velocities satisfy normal.dot(velocity) >= offset. Constructors normalize it.
#[derive(Clone,Copy,Debug)]
pub struct HalfPlane {pub normal:DVec2,pub offset:f64}
impl HalfPlane {
    pub fn new(normal:DVec2,offset:f64)->Result<Self,InputError> {
        if !normal.is_finite()||!offset.is_finite() {return Err(InputError::NonFinite);}
        let n=length(normal);if n==0.0 {return Err(InputError::ZeroNormal);}
        let offset=offset/n;
        if !n.is_finite()||!offset.is_finite()||!offset.powi(2).is_finite() {return Err(InputError::MagnitudeOverflow);}
        Ok(Self{normal:normal/n,offset})
    }
    pub fn through(normal:DVec2,point:DVec2)->Result<Self,InputError> {
        if !point.is_finite() {return Err(InputError::NonFinite);}
        Self::new(normal,normal.dot(point))
    }
    pub fn violation(self,v:DVec2)->f64 {(self.offset-self.normal.dot(v)).max(0.0)}
}

#[derive(Clone,Copy,Debug)]
pub struct Agent {pub id:u64,pub position:DVec2,pub velocity:DVec2,pub radius:f64}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Responsibility {Reciprocal,Full}
impl Responsibility {pub fn share(self)->f64 {match self{Self::Reciprocal=>0.5,Self::Full=>1.0}}}
#[derive(Clone,Copy,Debug)]
pub struct PairConstraint {pub plane:HalfPlane,pub overlapping:bool,pub short_horizon:bool,pub share:f64}

/// Full responsibility is mandatory for a neighbor that will not react, including
/// a parked/stopped owner. Coincident zero-relative-velocity pairs use opposite ID axes.
pub fn unit_constraint(me:Agent,other:Agent,responsibility:Responsibility,horizon:f64,dt:f64)->Result<PairConstraint,InputError> {
    if me.id==other.id {return Err(InputError::DuplicateId);}
    if !me.position.is_finite()||!other.position.is_finite()||!me.velocity.is_finite()||!other.velocity.is_finite()
        ||!me.radius.is_finite()||!other.radius.is_finite()||!horizon.is_finite()||!dt.is_finite() {return Err(InputError::NonFinite);}
    if me.radius<=0.0||other.radius<=0.0 {return Err(InputError::InvalidRadius);}
    if horizon<=0.0||dt<=0.0 {return Err(InputError::NonPositiveTime);}
    let p=other.position-me.position;let relative=me.velocity-other.velocity;let distance_sq=p.length_squared();
    let radius=me.radius+other.radius;let radius_sq=radius*radius;
    if !p.is_finite()||!relative.is_finite()||!distance_sq.is_finite()||!radius_sq.is_finite() {return Err(InputError::MagnitudeOverflow);}
    let short_horizon=distance_sq<=radius_sq;let inverse=1.0/if short_horizon{dt}else{horizon};
    let w=relative-p*inverse;let w_length=length(w);let dot=w.dot(p);
    if !inverse.is_finite()||!w.is_finite()||!w_length.is_finite()||!dot.is_finite() {return Err(InputError::MagnitudeOverflow);}
    let (direction,change)=if short_horizon||dot<0.0&&-dot>radius*w_length {
        // Equivalent to RVO2's squared cutoff test, without squaring the dot product.
        let normal=if w_length>0.0 {w/w_length}else if distance_sq>0.0 {-p/length(p)}
            else if me.id<other.id {-DVec2::X}else{DVec2::X};
        (DVec2::new(normal.y,-normal.x),normal*(radius*inverse-w_length))
    }else{
        let leg=(distance_sq-radius_sq).max(0.0).sqrt();
        let d=if det(p,w)>0.0 {DVec2::new(p.x*leg-p.y*radius,p.x*radius+p.y*leg)/distance_sq}
            else{-DVec2::new(p.x*leg+p.y*radius,-p.x*radius+p.y*leg)/distance_sq};
        let norm=length(d);if !d.is_finite()||!norm.is_finite()||norm==0.0 {return Err(InputError::MagnitudeOverflow);}
        let direction=d/norm;(direction,direction*relative.dot(direction)-relative)
    };
    let share=responsibility.share();let plane=HalfPlane::through(left(direction),me.velocity+change*share)?;
    Ok(PairConstraint{plane,overlapping:distance_sq<radius_sq,short_horizon,share})
}

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum SolveStatus {
    Feasible,
    Infeasible {first:usize,hard:bool},
    InvalidInput {index:Option<usize>,reason:InputError},
    NumericalFailure,
}
#[derive(Clone,Copy,Debug)]
pub struct Solution {pub velocity:DVec2,pub status:SolveStatus,pub violated:usize,pub hard_violated:usize,pub max_violation:f64}
impl Solution {
    fn invalid(index:Option<usize>,reason:InputError)->Self {Self{velocity:DVec2::ZERO,status:SolveStatus::InvalidInput{index,reason},violated:0,hard_violated:0,max_violation:0.0}}
}
#[derive(Clone,Copy,Debug)]
struct Line {point:DVec2,direction:DVec2}
fn violation(line:Line,v:DVec2)->f64 {det(line.direction,line.point-v)}

/// Reuse one workspace for sequential solves. Static/boundary planes may occupy
/// the first hard_count entries; the infeasibility relaxation never relaxes them.
#[derive(Default)]
pub struct Solver {lines:Vec<Line>,projected:Vec<Line>}
impl Solver {
    pub fn solve(&mut self,planes:&[HalfPlane],preferred:DVec2,max_speed:f64,hard_count:usize)->Solution {
        if !preferred.is_finite()||!max_speed.is_finite() {return Solution::invalid(None,InputError::NonFinite);}
        if max_speed<0.0 {return Solution::invalid(None,InputError::InvalidRadius);}
        if hard_count>planes.len() {return Solution::invalid(None,InputError::InvalidHardCount);}
        if !length(preferred).is_finite()||!(max_speed*max_speed).is_finite() {return Solution::invalid(None,InputError::MagnitudeOverflow);}
        self.lines.clear();
        for (i,plane) in planes.iter().enumerate() {
            let p=match HalfPlane::new(plane.normal,plane.offset) {Ok(p)=>p,Err(reason)=>return Solution::invalid(Some(i),reason)};
            self.lines.push(Line{point:p.normal*p.offset,direction:DVec2::new(p.normal.y,-p.normal.x)});
        }
        let mut velocity=bounded(preferred,max_speed);
        let failed=lp2(&self.lines,max_speed,preferred,false,&mut velocity);
        let prefix=velocity;
        if failed<self.lines.len()&&failed>=hard_count {lp3(&self.lines,hard_count,failed,max_speed,&mut velocity,&mut self.projected);}
        let tolerance=FEASIBILITY_TOLERANCE*(1.0+max_speed);
        // Numerical failure keeps a bounded prefix solution and reports failure;
        // no successful solution is rotated or clipped after solving.
        let numerical=!velocity.is_finite()||length(velocity)>max_speed+tolerance;
        if numerical {velocity=prefix;}
        if self.lines[..hard_count].iter().any(|l|violation(*l,velocity)>tolerance) && failed>=hard_count {velocity=prefix;}
        let mut result=Solution{velocity,status:SolveStatus::Feasible,violated:0,hard_violated:0,max_violation:0.0};
        for (i,line) in self.lines.iter().enumerate() {let error=violation(*line,velocity).max(0.0);
            result.max_violation=result.max_violation.max(error);
            if error>tolerance {result.violated+=1;result.hard_violated+=usize::from(i<hard_count);}
        }
        result.status=if numerical {SolveStatus::NumericalFailure}
            else if result.violated==0 {SolveStatus::Feasible}
            else if failed<self.lines.len() {SolveStatus::Infeasible{first:failed,hard:failed<hard_count}}
            else{SolveStatus::NumericalFailure};
        result
    }
}

fn lp1(lines:&[Line],index:usize,radius:f64,optimum:DVec2,directional:bool)->Option<DVec2> {
    let line=lines[index];let normal=left(line.direction);let offset=normal.dot(line.point);
    let discriminant=radius*radius-offset*offset;
    if discriminant<0.0||!discriminant.is_finite() {return None;}
    // Canonical line anchor avoids cancellation from a large tangential coordinate.
    let point=normal*offset;let root=discriminant.sqrt();let (mut lo,mut hi)=(-root,root);
    for previous in &lines[..index] {
        let denominator=det(line.direction,previous.direction);let numerator=det(previous.direction,point-previous.point);
        if denominator.abs()<=PARALLEL_EPS {if numerator<0.0 {return None;}continue;}
        let t=numerator/denominator;if denominator>=0.0 {hi=hi.min(t);}else{lo=lo.max(t);}
        if lo>hi {return None;}
    }
    let t=if directional {if optimum.dot(line.direction)>0.0 {hi}else{lo}}
        else{line.direction.dot(optimum-point).clamp(lo,hi)};
    let result=point+line.direction*t;result.is_finite().then_some(result)
}
fn lp2(lines:&[Line],radius:f64,optimum:DVec2,directional:bool,result:&mut DVec2)->usize {
    *result=if directional {optimum*radius}else{bounded(optimum,radius)};
    for i in 0..lines.len() {if violation(lines[i],*result)>0.0 {
        if let Some(candidate)=lp1(lines,i,radius,optimum,directional) {*result=candidate;}else{return i;}
    }}lines.len()
}
fn lp3(lines:&[Line],hard:usize,begin:usize,radius:f64,result:&mut DVec2,projected:&mut Vec<Line>) {
    let mut distance=0.0;
    for i in begin..lines.len() {if violation(lines[i],*result)>distance {
        projected.clear();projected.extend_from_slice(&lines[..hard]);
        for j in hard..i {
            let determinant=det(lines[i].direction,lines[j].direction);
            let point=if determinant.abs()<=PARALLEL_EPS {
                if lines[i].direction.dot(lines[j].direction)>0.0 {continue;}
                (lines[i].point+lines[j].point)*0.5
            }else{lines[i].point+lines[i].direction*(det(lines[j].direction,lines[i].point-lines[j].point)/determinant)};
            let delta=lines[j].direction-lines[i].direction;let n=length(delta);
            if n==0.0||!n.is_finite()||!point.is_finite() {continue;}
            projected.push(Line{point,direction:delta/n});
        }
        let previous=*result;
        if lp2(projected,radius,left(lines[i].direction),true,result)<projected.len() {*result=previous;}
        distance=violation(lines[i],*result);
    }}
}

#[cfg(test)]
#[path="orca_tests.rs"]
mod tests;
