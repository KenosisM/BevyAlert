// Copyright 2008 University of North Carolina at Chapel Hill
// SPDX-License-Identifier: Apache-2.0
// Static velocity obstacles adapted from RVO2 v2.0.2 Agent.cpp, commit
// c96a6ec8328492bae0721e59774026a119a60b4a. Authors: Jur van den Berg,
// Stephen J. Guy, Jamie Snape, Ming C. Lin, and Dinesh Manocha.
// Modified: Rust/f64, grid-derived contours, spatial queries, checked normals.
// Licensed under the Apache License, Version 2.0. See
// licenses/RVO2-APACHE-2.0.txt. Distributed AS IS, WITHOUT WARRANTIES OR CONDITIONS.

//! Obstacle polygons are a derived cache of CostGrid, including the map perimeter.
//! Solid is on the left of each directed edge. No independent terrain rules.
use std::collections::BTreeMap;
use bevy::math::DVec2;
use crate::{nav::CostGrid,orca::{HalfPlane,InputError},spatial::{Aabb,Index}};
fn det(a:DVec2,b:DVec2)->f64 {a.perp_dot(b)}
fn left(a:DVec2)->DVec2 {DVec2::new(-a.y,a.x)}
fn right(a:DVec2)->DVec2 {DVec2::new(a.y,-a.x)}
fn unit(v:DVec2,fallback:DVec2)->DVec2 {if v.length_squared()>1e-24 {v.normalize()}else{fallback}}

#[derive(Clone,Debug)]
struct Vertex {point:DVec2,next:usize,prev:usize,direction:DVec2,convex:bool}
#[derive(Default)]
pub struct Boundaries {signature:Vec<u8>,width:i32,height:i32,vertices:Vec<Vertex>,pub index:Index,
    candidates:Vec<usize>,ordered:Vec<(f64,usize)>}
impl Boundaries {
    pub fn refresh(&mut self,g:&CostGrid) {
        self.index.begin_tick();
        if self.width==g.w&&self.height==g.h&&self.signature.len()==g.cost.len()
            &&self.signature.iter().zip(&g.cost).all(|(a,b)|*a==u8::from(*b>0)) {return;}
        self.width=g.w;self.height=g.h;self.signature=g.cost.iter().map(|v|u8::from(*v>0)).collect();
        self.vertices.clear();
        if !g.valid() {self.index.rebuild(&[]);return;}
        type P=(i32,i32);
        let mut edges:Vec<(P,P)>=Vec::new();
        for y in 0..g.h {for x in 0..g.w {if g.walkable(x,y) {
            let (l,r,t,b)=(2*x-1,2*x+1,2*y-1,2*y+1);
            if !g.walkable(x,y-1) {edges.push(((r,t),(l,t)));}
            if !g.walkable(x+1,y) {edges.push(((r,b),(r,t)));}
            if !g.walkable(x,y+1) {edges.push(((l,b),(r,b)));}
            if !g.walkable(x-1,y) {edges.push(((l,t),(l,b)));}
        }}}
        let mut outgoing:BTreeMap<P,Vec<usize>>=BTreeMap::new();
        for (i,(a,_)) in edges.iter().enumerate() {outgoing.entry(*a).or_default().push(i);}
        let mut seen=vec![false;edges.len()];
        for first in 0..edges.len() {if seen[first] {continue;}
            let mut contour=Vec::new();let mut current=first;
            loop {
                assert!(!seen[current],"navigation boundary must form closed contours");
                seen[current]=true;let (a,b)=edges[current];contour.push(a);
                let dx=b.0-a.0;let dy=b.1-a.1;
                let next=*outgoing[&b].iter().max_by_key(|&&i| {
                    let c=edges[i].1;let ex=c.0-b.0;let ey=c.1-b.1;
                    // Keep diagonally touching solids separate: turn left first.
                    match (dx*ey-dy*ex,dx*ex+dy*ey) {(v,_) if v>0=>3,(0,v) if v>0=>2,(v,_) if v<0=>1,_=>0}
                }).unwrap();
                if next==first {break;}current=next;
            }
            let points:Vec<_>=(0..contour.len()).filter_map(|i| {
                let a=contour[(i+contour.len()-1)%contour.len()];let b=contour[i];let c=contour[(i+1)%contour.len()];
                ((b.0-a.0)*(c.1-b.1)!=(b.1-a.1)*(c.0-b.0)).then_some(DVec2::new(b.0 as f64*0.5,b.1 as f64*0.5))
            }).collect();
            self.add_polygon(&points);
        }
        self.reindex();
    }
    fn add_polygon(&mut self,p:&[DVec2]) {
        let base=self.vertices.len();let n=p.len();if n<2 {return;}
        for i in 0..n {let prev=(i+n-1)%n;let next=(i+1)%n;
            self.vertices.push(Vertex{point:p[i],next:base+next,prev:base+prev,
                direction:(p[next]-p[i]).normalize(),convex:n==2||det(p[i]-p[prev],p[next]-p[i])>=0.0});
        }
    }
    fn reindex(&mut self) {
        let bounds:Vec<_>=self.vertices.iter().map(|v|Aabb::swept(v.point.as_vec2(),self.vertices[v.next].point.as_vec2(),0.0)).collect();
        self.index.rebuild(&bounds);
    }
    /// Every visible edge in the reachable horizon; no obstacle or neighbor cap.
    pub fn constraints(&mut self,position:DVec2,velocity:DVec2,radius:f64,speed:f64,horizon:f64,
        out:&mut Vec<HalfPlane>)->Result<(),InputError> {
        if !position.is_finite()||!velocity.is_finite()||!radius.is_finite()||!speed.is_finite()||!horizon.is_finite() {return Err(InputError::NonFinite);}
        if radius<=0.0||speed<0.0 {return Err(InputError::InvalidRadius);}
        if horizon<=0.0 {return Err(InputError::NonPositiveTime);}
        let range=radius+speed.max(velocity.length())*horizon;
        self.index.query(Aabb::around(position.as_vec2(),(range as f32).next_up()),&mut self.candidates);self.ordered.clear();
        for &i in &self.candidates {
            let a=&self.vertices[i];let b=&self.vertices[a.next];
            if det(b.point-a.point,position-a.point)>=0.0 {continue;}
            let edge=b.point-a.point;let t=((position-a.point).dot(edge)/edge.length_squared()).clamp(0.0,1.0);
            let d=position.distance_squared(a.point+edge*t);
            if d<=range*range {self.ordered.push((d,i));}
        }
        self.ordered.sort_by(|a,b|a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for &(_,i) in &self.ordered {self.edge_constraint(i,position,velocity,radius,horizon,out)?;}
        Ok(())
    }
    fn edge_constraint(&self,index:usize,pos:DVec2,velocity:DVec2,r:f64,horizon:f64,out:&mut Vec<HalfPlane>)->Result<(),InputError> {
        let inv=1.0/horizon;let mut i=index;let mut j=self.vertices[i].next;
        let p=self.vertices[i].point-pos;let q=self.vertices[j].point-pos;
        // det(inv*p-line.point,line.direction) == n.dot(line.point-inv*p).
        if out.iter().any(|h|h.offset-h.normal.dot(inv*p)-inv*r>=-1e-8
            &&h.offset-h.normal.dot(inv*q)-inv*r>=-1e-8) {return Ok(());}
        let (d1,d2,r2)=(p.length_squared(),q.length_squared(),r*r);
        let edge=q-p;let s=-p.dot(edge)/edge.length_squared();let dl=(-p-s*edge).length_squared();
        let mut add=|direction:DVec2,point:DVec2|->Result<(),InputError> {out.push(HalfPlane::through(left(direction),point)?);Ok(())};
        if s<0.0&&d1<=r2 {if self.vertices[i].convex {add(unit(left(p),-self.vertices[i].direction),DVec2::ZERO)?;}return Ok(());}
        if s>1.0&&d2<=r2 {if self.vertices[j].convex&&det(q,self.vertices[j].direction)>=0.0 {add(unit(left(q),-self.vertices[i].direction),DVec2::ZERO)?;}return Ok(());}
        if s>=0.0&&s<1.0&&dl<=r2 {add(-self.vertices[i].direction,DVec2::ZERO)?;return Ok(());}
        let leg=|p:DVec2,d:f64,sign:f64| {let l=(d-r2).max(0.0).sqrt();DVec2::new(p.x*l-sign*p.y*r,sign*p.x*r+p.y*l)/d};
        let (mut l,mut rr);
        if s<0.0&&dl<=r2 {if !self.vertices[i].convex {return Ok(());}j=i;l=leg(p,d1,1.0);rr=leg(p,d1,-1.0);}
        else if s>1.0&&dl<=r2 {if !self.vertices[j].convex {return Ok(());}i=j;l=leg(q,d2,1.0);rr=leg(q,d2,-1.0);}
        else {l=if self.vertices[i].convex {leg(p,d1,1.0)}else{-self.vertices[i].direction};
            rr=if self.vertices[j].convex {leg(q,d2,-1.0)}else{self.vertices[i].direction};}
        let neighbor=&self.vertices[self.vertices[i].prev];let mut foreign_l=false;let mut foreign_r=false;
        if self.vertices[i].convex&&det(l,-neighbor.direction)>=0.0 {l=-neighbor.direction;foreign_l=true;}
        if self.vertices[j].convex&&det(rr,self.vertices[j].direction)<=0.0 {rr=self.vertices[j].direction;foreign_r=true;}
        let lc=(self.vertices[i].point-pos)*inv;let rc=(self.vertices[j].point-pos)*inv;let cv=rc-lc;
        let t=if i==j {0.5}else{(velocity-lc).dot(cv)/cv.length_squared()};
        let tl=(velocity-lc).dot(l);let tr=(velocity-rc).dot(rr);
        if t<0.0&&tl<0.0||i==j&&tl<0.0&&tr<0.0 {let w=unit(velocity-lc,unit(-lc,DVec2::X));add(right(w),lc+r*inv*w)?;return Ok(());}
        if t>1.0&&tr<0.0 {let w=unit(velocity-rc,unit(-rc,DVec2::X));add(right(w),rc+r*inv*w)?;return Ok(());}
        let dc=if t<0.0||t>1.0||i==j {f64::INFINITY}else{velocity.distance_squared(lc+t*cv)};
        let dl=if tl<0.0 {f64::INFINITY}else{velocity.distance_squared(lc+tl*l)};
        let dr=if tr<0.0 {f64::INFINITY}else{velocity.distance_squared(rc+tr*rr)};
        let choice=if dc<=dl&&dc<=dr {Some((-self.vertices[i].direction,lc))}
            else if dl<=dr {(!foreign_l).then_some((l,lc))}else{(!foreign_r).then_some((-rr,rc))};
        if let Some((direction,cutoff))=choice {add(direction,cutoff+r*inv*left(direction))?;}
        Ok(())
    }
    #[cfg(test)] pub fn from_polygons(polygons:&[Vec<DVec2>])->Self {
        let mut out=Self::default();for p in polygons {out.add_polygon(p);}out.reindex();out
    }
    #[cfg(test)] pub fn edge_count(&self)->usize {self.vertices.len()}
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn contours_preserve_diagonal_contacts_perimeter_and_grid_edits() {
        let mut g=CostGrid{w:8,h:8,cost:vec![1;64],..Default::default()};let mut b=Boundaries::default();
        b.refresh(&g);assert_eq!(b.edge_count(),4);
        g.cost[3*8+3]=0;g.cost[4*8+4]=0;b.refresh(&g);assert_eq!(b.edge_count(),12);
        assert_eq!(b.vertices.iter().filter(|v|v.convex).count(),8);
        g.cost[3*8+4]=0;b.refresh(&g);assert_eq!(b.edge_count(),10);
        g.cost[0]=2;b.refresh(&g);assert_eq!(b.edge_count(),10);
    }
    #[test]
    fn original_cpp_wall_corner_concavity_and_corridor_reference() {
        let mut count=0;
        for row in include_str!("../tests/data/orca-obstacles-rvo2-v2.0.2.csv").lines().filter(|l|!l.starts_with('#')&&!l.is_empty()) {
            let mut f=row.split(',');assert_eq!(f.next(),Some("obstacle"));
            let values:Vec<f64>=f.map(|v|v.parse().unwrap()).collect();let v=&values;
            let (id,horizon,pos,velocity,radius,cap,pref,n)=(v[0] as usize,v[1],DVec2::new(v[3],v[4]),DVec2::new(v[5],v[6]),v[7],v[8],DVec2::new(v[9],v[10]),v[11] as usize);
            let mut cursor=12;let mut polygons=Vec::new();
            for _ in 0..n {let k=v[cursor] as usize;cursor+=1;let mut p=Vec::new();for _ in 0..k {p.push(DVec2::new(v[cursor],v[cursor+1]));cursor+=2;}polygons.push(p);}
            let expected=DVec2::new(v[cursor+1],v[cursor+2]);let mut b=Boundaries::from_polygons(&polygons);let mut planes=Vec::new();
            b.constraints(pos,velocity,radius,cap,horizon,&mut planes).unwrap();
            let s=crate::orca::Solver::default().solve(&planes,pref,cap,planes.len());
            assert_eq!(s.status,crate::orca::SolveStatus::Feasible,"case {id}: {s:?}");
            assert!(s.velocity.distance(expected)<=2e-4*(1.0+cap),"case {id}: Rust {:?}, original {expected:?}, lines {} original {}",s.velocity,planes.len(),v[cursor]);
            assert!(planes.iter().all(|p|p.violation(s.velocity)<=crate::orca::FEASIBILITY_TOLERANCE*(1.0+cap)));
            count+=1;
        }
        assert_eq!(count,160);
    }
    #[test]
    fn grid_boundary_prediction_is_clear_before_contact_projection() {
        use bevy::math::Vec2;
        let mut g=CostGrid{w:40,h:30,cost:vec![1;1200],..Default::default()};
        for y in 5..25 {g.cost[y*40+20]=0;}
        let mut b=Boundaries::default();b.refresh(&g);
        for start in [Vec2::new(18.0,15.0),Vec2::new(18.0,4.0),Vec2::new(22.0,25.0),Vec2::new(0.1,10.0)] {
            for wanted in [Vec2::X*6.0,-Vec2::X*6.0,Vec2::new(4.0,4.0),Vec2::new(4.0,-4.0)] {
                let mut planes=Vec::new();b.constraints(start.as_dvec2(),wanted.as_dvec2(),0.45,6.0,2.0,&mut planes).unwrap();
                let s=crate::orca::Solver::default().solve(&planes,wanted.as_dvec2(),6.0,planes.len());assert_eq!(s.status,crate::orca::SolveStatus::Feasible);
                assert!(crate::collision::disc_segment_clear(&g,start,start+s.velocity.as_vec2()*1.9,0.45),"{start:?} {:?}",s.velocity);
            }
        }
    }
}
