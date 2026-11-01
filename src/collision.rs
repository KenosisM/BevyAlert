//! Conservative disc motion on the navigation grid. No cached copy of terrain geometry.
use bevy::math::{DVec2,Vec2};
use crate::{crowd::Agent, nav::{self, CostGrid}};
use crate::spatial::{Aabb,Index};

#[derive(Default,Debug)]
pub struct Workspace {
    pub projection_index:Index,pub trajectory_index:Index,
    bounds:Vec<Aabb>,pairs:Vec<(usize,usize)>,neighbors:Vec<usize>,before_projection:Vec<Vec2>,
    recovery:Vec<bool>,
}
impl Workspace {
    pub fn begin_tick(&mut self) {self.projection_index.begin_tick();self.trajectory_index.begin_tick();}
}

fn trajectory_pairs(old:&[Agent],next:&[Agent],w:&mut Workspace) {
    w.bounds.clear();w.bounds.extend(old.iter().zip(next.iter()).map(|(a,b)|Aabb::swept(pos(a),pos(b),a.radius+CONTACT_GAP)));
    w.trajectory_index.rebuild(&w.bounds);w.trajectory_index.pairs(&mut w.pairs);
}

/// Contact gap used to avoid repeated floating-point hits; radii are never reduced.
pub const CONTACT_GAP: f32 = 0.0001;
const EPS: f32 = 0.00001;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Blocked { #[default] None, StaticBoundary, InvalidStart, Contact, SafetyStop }

#[derive(Clone, Debug, Default)]
pub struct ContactReport {
    pub reasons: Vec<Blocked>,
    pub unresolved_pairs: usize,
    pub max_penetration: f32,
    pub limited_trajectories: usize,
    pub initial_overlaps: usize,
    pub max_swept_penetration: f32,
    pub illegal_static_trajectories: usize,
}

fn pos(a: &Agent) -> Vec2 { Vec2::new(a.x,a.y) }
fn set(a: &mut Agent,p: Vec2) { a.x=p.x; a.y=p.y; }
fn initially_overlapping(a:&Agent,b:&Agent)->bool {
    (pos(a).as_dvec2()-pos(b).as_dvec2()).length()+f64::from(EPS)<f64::from(a.radius)+f64::from(b.radius)
}

/// Exact circle/square clearance, including the outside of the map.
pub fn disc_clear(g: &CostGrid,p: Vec2,r: f32) -> bool {
    // Accepting an inset tolerance can leave a start that the next sweep rejects.
    let (px,py,rd)=(f64::from(p.x),f64::from(p.y),f64::from(r));
    if !g.valid() || !p.is_finite() || !r.is_finite() || r<0.0
        || px-rd < -0.5 || py-rd < -0.5
        || px+rd > g.w as f64-0.5 || py+rd > g.h as f64-0.5 {return false;}
    let (x0,x1)=(((p.x-r-0.5).floor() as i32).max(0),((p.x+r+0.5).ceil() as i32).min(g.w-1));
    let (y0,y1)=(((p.y-r-0.5).floor() as i32).max(0),((p.y+r+0.5).ceil() as i32).min(g.h-1));
    for y in y0..=y1 { for x in x0..=x1 {
        if g.walkable(x,y) {continue;}
        let near=p.clamp(Vec2::new(x as f32-0.5,y as f32-0.5),Vec2::new(x as f32+0.5,y as f32+0.5));
        if (p.as_dvec2()-near.as_dvec2()).length_squared()<(r as f64).powi(2) || (r==0.0 && p==near) {return false;}
    }}
    true
}

#[derive(Clone, Copy)]
struct Hit { t: f32, normal: Vec2 }

/// Earliest moving-circle contact: rectangle faces plus its four circular corners.
fn first_hit(g: &CostGrid,p: Vec2,d: Vec2,r: f32) -> Option<Hit> {
    if d.length_squared()<1e-16 {return None;}
    let mut hit: Option<Hit>=None;
    let mut add=|t:f32,n:Vec2| {
        if t>=-EPS && t<=1.0 && d.dot(n)<0.0 && hit.is_none_or(|h|t<h.t) {
            hit=Some(Hit{t:t.max(0.0),normal:n});
        }
    };
    for (q,v,lo,hi,n) in [(p.x,d.x,-0.5+r,g.w as f32-0.5-r,Vec2::X),
        (p.y,d.y,-0.5+r,g.h as f32-0.5-r,Vec2::Y)] {
        if v<0.0 {add((lo-q)/v,n);} else if v>0.0 {add((hi-q)/v,-n);}
    }
    let end=p+d;
    let low=p.min(end)-Vec2::splat(r+0.5);let high=p.max(end)+Vec2::splat(r+0.5);
    for y in (low.y.floor() as i32).max(0)..=(high.y.ceil() as i32).min(g.h-1) {
        for x in (low.x.floor() as i32).max(0)..=(high.x.ceil() as i32).min(g.w-1) {
            if g.walkable(x,y) {continue;}
            let lo=Vec2::new(x as f32-0.5,y as f32-0.5);let hi=lo+Vec2::ONE;
            if d.x!=0.0 {
                let (face,n)=if d.x>0.0 {(lo.x-r,-Vec2::X)}else{(hi.x+r,Vec2::X)};
                let t=(face-p.x)/d.x;let y=p.y+d.y*t;
                if y>=lo.y-EPS && y<=hi.y+EPS {add(t,n);}
            }
            if d.y!=0.0 {
                let (face,n)=if d.y>0.0 {(lo.y-r,-Vec2::Y)}else{(hi.y+r,Vec2::Y)};
                let t=(face-p.y)/d.y;let x=p.x+d.x*t;
                if x>=lo.x-EPS && x<=hi.x+EPS {add(t,n);}
            }
            if r>0.0 {for (cx,sx) in [(lo.x,-1.0),(hi.x,1.0)] {for (cy,sy) in [(lo.y,-1.0),(hi.y,1.0)] {
                let q=p-Vec2::new(cx,cy);
                if let Some(t)=circle_toi(q,d,r) {
                    let n=q+d*t;
                    if n.x*sx>=-EPS && n.y*sy>=-EPS {add(t,n.normalize_or_zero());}
                }
            }}}
        }
    }
    hit
}

/// Relative segment time of impact. Double precision avoids cancellation at long displacements.
fn circle_toi(q:Vec2,d:Vec2,r:f32)->Option<f32> {
    circle_toi_f64(q.as_dvec2(),d.as_dvec2(),r as f64)
}
fn circle_toi_f64(q:DVec2,d:DVec2,r:f64)->Option<f32> {
    let (qx,qy,dx,dy)=(q.x,q.y,d.x,d.y);
    let a=dx*dx+dy*dy;let b=qx*dx+qy*dy;let c=qx*qx+qy*qy-r*r;
    if a<1e-20 || b>=0.0 {return None;}
    if c<=0.0 {return Some(0.0);}
    let discriminant=b*b-a*c;
    if discriminant<0.0 {return None;}
    let t=c/(-b+discriminant.sqrt());
    (t<=1.0).then_some(t as f32)
}

fn clip(g:&CostGrid,from:Vec2,to:Vec2,r:f32)->Vec2 {
    let d=to-from;
    let t=first_hit(g,from,d,r).map_or(1.0,|h|(h.t-CONTACT_GAP/d.length().max(CONTACT_GAP)).max(0.0));
    let candidate=from+d*t;
    if disc_segment_clear(g,from,candidate,r) {return candidate;}
    // Navigation's stricter no-cut rule can also reject an otherwise clear circle segment.
    let (mut lo,mut hi)=(0.0,t);
    for _ in 0..24 {
        let m=(lo+hi)*0.5;let p=from+d*m;
        if disc_segment_clear(g,from,p,r) {lo=m;}else{hi=m;}
    }
    let candidate=from+d*(lo-CONTACT_GAP/d.length().max(CONTACT_GAP)).max(0.0);
    if disc_segment_clear(g,from,candidate,r) {candidate}else{from}
}

pub fn disc_segment_clear(g:&CostGrid,from:Vec2,to:Vec2,r:f32)->bool {
    disc_clear(g,from,r)&&disc_clear(g,to,r)
        &&nav::segment_allowed(g,(from.x,from.y),(to.x,to.y))
        &&first_hit(g,from,to-from,r).is_none_or(|h|h.t>=1.0-EPS)
}

pub fn segment_avoids_disc(from:Vec2,to:Vec2,center:Vec2,radius:f32)->bool {
    let q=from.as_dvec2()-center.as_dvec2();let d=to.as_dvec2()-from.as_dvec2();
    let t=(-q.dot(d)/d.length_squared().max(1e-20)).clamp(0.0,1.0);
    (q+d*t).length()>=radius as f64-EPS as f64
}

/// Slide on the first wall, then verify the resulting *whole* tick segment again.
/// An invalid initial position is reported and kept; this routine never searches for a teleport target.
pub fn move_disc(g:&CostGrid,from:Vec2,to:Vec2,r:f32)->(Vec2,Blocked) {
    if !disc_clear(g,from,r) {return (from,Blocked::InvalidStart);}
    if !to.is_finite() || !(to-from).length_squared().is_finite() {return (from,Blocked::SafetyStop);}
    let d=to-from;let mut end=clip(g,from,to,r);
    if let Some(h)=first_hit(g,from,d,r) {
        let rest=d*(1.0-h.t);let tangent=rest-h.normal*rest.dot(h.normal).min(0.0);
        end=clip(g,end,end+tangent,r);
        end=clip(g,from,end,r);
    }
    let reason=if end.distance_squared(to)>EPS*EPS {Blocked::StaticBoundary}else{Blocked::None};
    (end,reason)
}

/// Constrained contact projection. Every correction starts at a saved legal position.
/// A wall-blocked share is offered to the other body, then all pairs are revisited.
#[cfg(test)]
pub fn project_contacts(agents:&mut [Agent],g:&CostGrid,iterations:u32) {
    project_contacts_with_workspace(agents,g,iterations,None,&mut Workspace::default());
}

fn project_contacts_with_workspace(agents:&mut [Agent],g:&CostGrid,iterations:u32,initial:Option<&[Agent]>,w:&mut Workspace) {
    let max_radius=agents.iter().map(|a|a.radius).fold(0.0f32,f32::max);
    w.bounds.clear();w.bounds.extend(agents.iter().map(|a|Aabb::point(pos(a))));w.projection_index.rebuild(&w.bounds);
    for _ in 0..iterations {
        let mut changed=false;
        for i in 0..agents.len() {
            w.projection_index.query_radius(pos(&agents[i]),agents[i].radius+max_radius+CONTACT_GAP,&mut w.neighbors);
            let mut cursor=0;let mut after=i;
            while cursor<w.neighbors.len() {
            let j=w.neighbors[cursor];cursor+=1;if j<=after {continue;}after=j;
            if let Some(old)=initial {
                if !initially_overlapping(&old[i],&old[j]) {continue;}
                w.recovery[i]=true;w.recovery[j]=true;
            }
            let (pi,pj)=(pos(&agents[i]),pos(&agents[j]));let q=pj-pi;let dist=q.length();
            let overlap=agents[i].radius+agents[j].radius+CONTACT_GAP-dist;
            if overlap<=EPS {continue;}
            let n=if dist>EPS {q/dist}else{Vec2::from_angle(((i*73856093)^(j*19349663)) as f32*0.618034)};
            let (wi,wj)=match (agents[i].settled,agents[j].settled) {
                (true,false)=>(0.25,0.75),(false,true)=>(0.75,0.25),_=>(0.5,0.5),
            };
            let shift=overlap.min(0.5);
            let ni=move_disc(g,pi,pi-n*(shift*wi/(wi+wj)),agents[i].radius).0;
            let nj=move_disc(g,pj,pj+n*(shift*wj/(wi+wj)),agents[j].radius).0;
            let missing=(shift-(pi-ni).dot(n)-(nj-pj).dot(n)).max(0.0);
            let ni=move_disc(g,ni,ni-n*missing,agents[i].radius).0;
            let missing=(shift-(pi-ni).dot(n)-(nj-pj).dot(n)).max(0.0);
            let nj=move_disc(g,nj,nj+n*missing,agents[j].radius).0;
            changed |= ni.distance_squared(pi)+nj.distance_squared(pj)>EPS*EPS;
            set(&mut agents[i],ni);set(&mut agents[j],nj);
            if ni!=pi {w.projection_index.update(i,Aabb::point(ni));}
            if nj!=pj {w.projection_index.update(j,Aabb::point(nj));}
            if ni!=pi {
                w.projection_index.query_radius(ni,agents[i].radius+max_radius+CONTACT_GAP,&mut w.neighbors);
                cursor=0;
            }
        }}
        if !changed {break;}
    }
}

fn pair_toi(old:&[Agent],next:&[Agent],i:usize,j:usize)->Option<f32> {
    // Widen positions before subtracting: this tick and the next must measure
    // the same published endpoints without float cancellation at map scale.
    let q=pos(&old[j]).as_dvec2()-pos(&old[i]).as_dvec2();let r=old[i].radius as f64+old[j].radius as f64;
    // Initial penetrations require recovery, not an impossible claim of safe past motion.
    if q.length()+f64::from(EPS)<r {return None;}
    let d=(pos(&next[j]).as_dvec2()-pos(&old[j]).as_dvec2())-(pos(&next[i]).as_dvec2()-pos(&old[i]).as_dvec2());
    let t=(-q.dot(d)/d.length_squared().max(1e-20)).clamp(0.0,1.0);
    if (q+d*t).length()+f64::from(EPS)>=r {return None;}
    circle_toi_f64(q,d,r)
}

/// Complete the proposed tick before committing any position. All published trajectories
/// are the straight segments old->new. Pair limits are iterated because slowing a leader
/// can affect its followers. Rare unsatisfied constraints conservatively stop the tick.
#[cfg(test)]
pub fn finish_motion(old:&[Agent],next:&mut [Agent],g:&CostGrid)->ContactReport {
    let mut report=ContactReport::default();
    finish_motion_with_workspace(old,next,g,&mut Workspace::default(),&mut report);report
}

pub fn finish_motion_with_workspace(old:&[Agent],next:&mut [Agent],g:&CostGrid,w:&mut Workspace,report:&mut ContactReport) {
    assert_eq!(old.len(),next.len());
    let mut reasons=std::mem::take(&mut report.reasons);reasons.clear();reasons.resize(next.len(),Blocked::None);
    *report=ContactReport{reasons,..Default::default()};
    for i in 0..next.len() {
        let (p,reason)=move_disc(g,pos(&old[i]),pos(&next[i]),old[i].radius);
        set(&mut next[i],p);report.reasons[i]=reason;
    }
    w.before_projection.clear();w.before_projection.extend(next.iter().map(pos));
    w.recovery.clear();w.recovery.resize(next.len(),false);
    project_contacts_with_workspace(next,g,8,Some(old),w);
    for i in 0..next.len() {
        if pos(&next[i]).distance_squared(w.before_projection[i])>EPS*EPS && report.reasons[i]==Blocked::None {
            report.reasons[i]=Blocked::Contact;
        }
        if disc_clear(g,pos(&old[i]),old[i].radius) {let p=clip(g,pos(&old[i]),pos(&next[i]),old[i].radius);set(&mut next[i],p);}
    }
    let mut safe=false;
    for _ in 0..32 {
        // Tangential projection can change a trajectory's box. Rebuild before each
        // pass instead of assuming candidates from the original motion still cover it.
        trajectory_pairs(old,next,w);
        let mut limited=false;
        for &(i,j) in &w.pairs {
            if let Some(t)=pair_toi(old,next,i,j) {
                let di=pos(&next[i])-pos(&old[i]);let dj=pos(&next[j])-pos(&old[j]);
                let n=((pos(&old[j])-pos(&old[i]))+(dj-di)*t).normalize_or_zero();
                let q=pos(&old[j])-pos(&old[i]);
                // Do not demand a new gap where valid bodies already touch: that
                // can be infeasible against a wall and needlessly kill tangent motion.
                let target=q.dot(n).min(old[i].radius+old[j].radius+CONTACT_GAP);
                let correction=((q+dj-di).dot(n)-target).min(0.0);
                let fixed_i=old[i].settled||(!w.recovery[i]&&w.before_projection[i].distance_squared(pos(&old[i]))<1e-16);
                let fixed_j=old[j].settled||(!w.recovery[j]&&w.before_projection[j].distance_squared(pos(&old[j]))<1e-16);
                let (wi,wj)=match (fixed_i,fixed_j) {
                    (true,false)=>(0.0,1.0),(false,true)=>(1.0,0.0),_=>(0.5,0.5),
                };
                let bound=|k:usize,candidate:Vec2| {
                    let delta=candidate-pos(&old[k]);let cap=w.before_projection[k].distance(pos(&old[k]));
                    let candidate=if !w.recovery[k]&&delta.length()>cap {pos(&old[k])+delta.normalize_or_zero()*cap}else{candidate};
                    move_disc(g,pos(&old[k]),candidate,old[k].radius).0
                };
                let pi=bound(i,pos(&old[i])+di+n*(correction*wi));
                let pj=bound(j,pos(&old[j])+dj-n*(correction*wj));
                set(&mut next[i],pi);set(&mut next[j],pj);
                report.reasons[i]=Blocked::Contact;report.reasons[j]=Blocked::Contact;
                report.limited_trajectories+=1;limited=true;
            }
        }
        if !limited {safe=true;break;}
    }
    if !safe {
        // Holding an unresolved pair can affect its followers. Propagate only those
        // necessary holds; a distant, legal trajectory must not be globally stopped.
        trajectory_pairs(old,next,w);
        for _ in 0..next.len() {
            let mut changed=false;
            for &(i,j) in &w.pairs {
                if pair_toi(old,next,i,j).is_none() {continue;}
                for k in [i,j] {if pos(&next[k])!=pos(&old[k]) {
                    set(&mut next[k],pos(&old[k]));report.reasons[k]=Blocked::SafetyStop;changed=true;
                }}
            }
            if !changed {break;}
        }
    }
    for &(i,j) in &w.pairs {
        let q=pos(&old[j]).as_dvec2()-pos(&old[i]).as_dvec2();let r=old[i].radius as f64+old[j].radius as f64;
        if initially_overlapping(&old[i],&old[j]) {report.initial_overlaps+=1;}
        else {
            let d=(pos(&next[j]).as_dvec2()-pos(&old[j]).as_dvec2())-(pos(&next[i]).as_dvec2()-pos(&old[i]).as_dvec2());
            let t=(-q.dot(d)/d.length_squared().max(1e-20)).clamp(0.0,1.0);
            report.max_swept_penetration=report.max_swept_penetration.max((r-(q+d*t).length()).max(0.0) as f32);
        }
        let penetration=(next[i].radius+next[j].radius-pos(&next[i]).distance(pos(&next[j]))).max(0.0);
        report.max_penetration=report.max_penetration.max(penetration);
        if penetration>0.02 {
            report.unresolved_pairs+=1;
            report.reasons[i]=Blocked::SafetyStop;report.reasons[j]=Blocked::SafetyStop;
        }
    }
    for i in 0..next.len() {
        let (a,b,r)=(pos(&old[i]),pos(&next[i]),old[i].radius);
        if !disc_clear(g,a,r) || !disc_clear(g,b,r)
            || !nav::segment_allowed(g,(a.x,a.y),(b.x,b.y))
            || first_hit(g,a,b-a,r).is_some_and(|h|h.t<1.0-EPS) {report.illegal_static_trajectories+=1;}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn grid(w:i32,h:i32)->CostGrid {CostGrid{w,h,cost:vec![1;(w*h) as usize],..Default::default()}}
    fn agent(x:f32,y:f32)->Agent {Agent{x,y,..Default::default()}}
    #[test]
    fn small_inward_steps_cannot_accumulate_wall_overlap() {
        let mut g=grid(192,160);for y in 0..g.h {for x in 78..=90 {g.cost[(y*g.w+x) as usize]=0;}}
        let mut old=vec![Agent{x:77.04999542236328,y:70.0,radius:0.45,..Default::default()}];
        for _ in 0..96 {
            let mut next=old.clone();next[0].x+=0.000008;next[0].y+=0.125;
            let report=finish_motion(&old,&mut next,&g);
            assert!(f64::from(next[0].x)+f64::from(next[0].radius)<=77.5);
            assert_eq!(report.illegal_static_trajectories,0);assert_eq!(report.initial_overlaps,0);
            old=next;
        }
        assert!(old[0].y>70.5,"legal tangential progress was stopped");
    }
    #[test]
    fn large_coordinates_do_not_hide_next_tick_pair_penetration() {
        let g=grid(192,160);
        let a=Agent{x:97.9861068725586,y:74.94999694824219,radius:0.45,..Default::default()};
        let b=Agent{x:97.08616638183594,y:74.95943450927734,radius:0.45,..Default::default()};
        let mut old=vec![a.clone(),b.clone()];let n=(pos(&b)-pos(&a)).normalize();
        set(&mut old[1],pos(&b)+n*0.0001);let mut next=vec![a,b];
        let report=finish_motion(&old,&mut next,&g);
        let radius=f64::from(old[0].radius)+f64::from(old[1].radius);
        assert!(swept_distance(&old[0],&old[1],&next[0],&next[1])+f64::from(EPS)>=radius);
        assert_eq!(report.initial_overlaps,0);assert_eq!(report.illegal_static_trajectories,0);
        let mut held=next.clone();let held_report=finish_motion(&next,&mut held,&g);
        assert_eq!(held_report.initial_overlaps,0);assert_eq!(held_report.unresolved_pairs,0);
    }
    // Independent distance from a moving point to a fixed point, in f64.
    fn swept_distance(a:&Agent,b:&Agent,na:&Agent,nb:&Agent)->f64 {
        let q=(b.x as f64-a.x as f64,b.y as f64-a.y as f64);
        let d=((nb.x-b.x) as f64-(na.x-a.x) as f64,(nb.y-b.y) as f64-(na.y-a.y) as f64);
        let den=d.0*d.0+d.1*d.1;
        let t=if den>0.0 {(-(q.0*d.0+q.1*d.1)/den).clamp(0.0,1.0)}else{0.0};
        (q.0+t*d.0).hypot(q.1+t*d.1)
    }
    fn assert_static_trace(g:&CostGrid,a:Vec2,b:Vec2,r:f32) {
        // Dense independent circle/square checks supplement analytical high-speed wall assertions.
        for k in 0..=1000 {
            let p=a.lerp(b,k as f32/1000.0);
            assert!(p.x-r>=-0.50002&&p.y-r>=-0.50002);
            assert!(p.x+r<=g.w as f32-0.49998&&p.y+r<=g.h as f32-0.49998);
            for y in 0..g.h {for x in 0..g.w {
                if g.at(x,y)!=0 {continue;}
                let dx=((p.x as f64-x as f64).abs()-0.5).max(0.0);
                let dy=((p.y as f64-y as f64).abs()-0.5).max(0.0);
                assert!(dx.hypot(dy)>=r as f64-0.00002,"trace hits tile {x},{y} at {p:?}");
            }}
        }
    }
    #[test]
    fn high_speed_disc_cannot_cross_a_wall_and_can_slide() {
        let mut g=grid(20,16);for y in 0..16 {g.cost[y*20+10]=0;}
        let from=Vec2::new(3.0,3.0);let (end,why)=move_disc(&g,from,Vec2::new(17.0,9.0),0.45);
        assert_eq!(why,Blocked::StaticBoundary);assert!(end.x<=9.05001&&end.y>6.0);
        assert_static_trace(&g,from,end,0.45);
    }
    #[test]
    fn corners_edges_and_radius_clearance_are_checked_over_the_whole_segment() {
        let mut g=grid(12,12);g.cost[4*12+5]=0;g.cost[5*12+4]=0;
        let from=Vec2::new(3.0,3.0);let (end,_)=move_disc(&g,from,Vec2::new(8.0,8.0),0.45);
        assert!(end.distance(from)<2.0);assert_static_trace(&g,from,end,0.45);
        for target in [Vec2::new(-20.0,3.0),Vec2::new(50.0,3.0),Vec2::new(3.0,-20.0),Vec2::new(3.0,50.0)] {
            let (end,why)=move_disc(&g,from,target,0.45);assert_eq!(why,Blocked::StaticBoundary);
            assert_static_trace(&g,from,end,0.45);
        }
        assert!(!disc_clear(&g,Vec2::new(4.4,4.0),0.45));
    }
    #[test]
    fn projection_preserves_wall_clearance_and_separates_a_coincident_pair() {
        let mut g=grid(8,8);for y in 0..8 {g.cost[y*8+4]=0;}
        let mut a=vec![agent(3.04,3.0),agent(2.5,3.0)];
        project_contacts(&mut a,&g,16);
        assert!(a.iter().all(|a|disc_clear(&g,pos(a),a.radius)));
        assert!(pos(&a[0]).distance(pos(&a[1]))>=0.89998);
        let old=vec![agent(2.0,2.0),agent(2.0,2.0)];let mut next=old.clone();
        let report=finish_motion(&old,&mut next,&g);
        assert_eq!(report.unresolved_pairs,0);assert!(report.max_penetration<=0.00002);
        assert!(next.iter().all(|a|pos(a).is_finite()&&disc_clear(&g,pos(a),a.radius)));
    }
    #[test]
    fn tick_internal_head_on_and_crossing_trajectories_never_pass_through_each_other() {
        let g=grid(32,32);
        for (old,target) in [
            (vec![agent(4.0,8.0),agent(16.0,8.0)],vec![agent(20.0,8.0),agent(0.0,8.0)]),
            (vec![agent(4.0,10.0),agent(10.0,4.0)],vec![agent(16.0,10.0),agent(10.0,16.0)]),
            (vec![agent(4.0,8.0),agent(4.9,8.0)],vec![agent(5.0,8.0),agent(3.9,8.0)])] {
            let mut next=target;let report=finish_motion(&old,&mut next,&g);
            assert!(swept_distance(&old[0],&old[1],&next[0],&next[1])>=0.89998);
            assert_eq!(report.unresolved_pairs,0);assert!(report.limited_trajectories>0);
        }
    }
    #[test]
    fn touching_discs_keep_legal_opposite_tangential_motion() {
        let g=grid(20,20);let old=vec![agent(5.0,5.0),agent(5.9,5.0)];let mut next=old.clone();
        next[0].x+=0.1;next[0].y+=0.1;next[1].x-=0.1;next[1].y-=0.1;
        let report=finish_motion(&old,&mut next,&g);
        assert_eq!(report.unresolved_pairs,0);assert_eq!(report.illegal_static_trajectories,0);
        assert!(swept_distance(&old[0],&old[1],&next[0],&next[1])>=0.89998);
        assert!(next[0].y>5.05&&next[1].y<4.95,"safe tangent motion was unnecessarily stopped");
        for i in 0..2 {assert!(pos(&next[i]).distance(pos(&old[i]))<=0.02f32.sqrt()+EPS);}
    }
    #[test]
    fn a_touching_body_can_slide_between_a_fixed_body_and_the_map_edge() {
        let g=grid(8,8);let old=vec![agent(-0.05,4.0),Agent{settled:true,..agent(0.85,4.0)}];
        let mut next=old.clone();next[0].x+=0.1;next[0].y+=0.1;
        let report=finish_motion(&old,&mut next,&g);
        assert!(next[0].y>4.09);assert_eq!(pos(&next[1]),pos(&old[1]));
        assert_eq!(report.illegal_static_trajectories,0);assert_eq!(report.unresolved_pairs,0);
        assert!(swept_distance(&old[0],&old[1],&next[0],&next[1])>=0.89998);
    }
    #[test]
    fn seeded_proposals_check_followers_after_contact_limiting() {
        let g=grid(32,32);let mut rng=7u64;
        for _ in 0..128 {
            let old=vec![agent(10.0,10.0),agent(11.1,10.0),agent(12.2,10.0),agent(11.1,11.1)];
            let mut next=old.clone();
            for a in &mut next {
                rng=rng.wrapping_mul(6364136223846793005).wrapping_add(1);a.x+=((rng>>32) as u32 as f64/u32::MAX as f64*8.0-4.0) as f32;
                rng=rng.wrapping_mul(6364136223846793005).wrapping_add(1);a.y+=((rng>>32) as u32 as f64/u32::MAX as f64*8.0-4.0) as f32;
            }
            let intended=next.clone();let report=finish_motion(&old,&mut next,&g);assert_eq!(report.unresolved_pairs,0);
            for i in 0..old.len() {assert!(pos(&next[i]).distance(pos(&old[i]))<=pos(&intended[i]).distance(pos(&old[i]))+EPS);}
            assert!(report.max_swept_penetration<=0.00002);assert_eq!(report.illegal_static_trajectories,0);
            for i in 0..old.len() {for j in i+1..old.len() {
                assert!(swept_distance(&old[i],&old[j],&next[i],&next[j])>=0.89998);
            }}
        }
    }
    #[test]
    fn dense_contact_fallback_does_not_stop_a_disjoint_trajectory() {
        let g=grid(40,40);let mut rng=19u64;
        for _ in 0..32 {
            let mut old:Vec<_>=(0..16).map(|i|agent(10.0+(i%4) as f32*0.901,10.0+(i/4) as f32*0.901)).collect();
            old.push(agent(30.0,30.0));let mut next=old.clone();
            for a in &mut next[..16] {
                rng=rng.wrapping_mul(6364136223846793005).wrapping_add(1);a.x+=((rng>>32) as u32 as f64/u32::MAX as f64*0.2-0.1) as f32;
                rng=rng.wrapping_mul(6364136223846793005).wrapping_add(1);a.y+=((rng>>32) as u32 as f64/u32::MAX as f64*0.2-0.1) as f32;
            }
            next[16].x+=0.1;let expected=pos(&next[16]);let report=finish_motion(&old,&mut next,&g);
            assert_eq!(pos(&next[16]),expected);assert!(report.max_swept_penetration<=0.00002);assert_eq!(report.unresolved_pairs,0);
        }
    }
    #[test]
    fn invalid_start_and_infeasible_initial_contacts_are_reported_without_teleporting() {
        let g=grid(1,1);let old=vec![agent(0.0,0.0),agent(0.0,0.0)];let mut next=old.clone();
        let report=finish_motion(&old,&mut next,&g);assert_eq!(report.unresolved_pairs,1);
        assert!(report.reasons.iter().all(|x|*x==Blocked::SafetyStop));
        assert!(next.iter().all(|a|pos(a).is_finite()&&disc_clear(&g,pos(a),a.radius)));
        let p=Vec2::new(2.0,0.0);let (end,why)=move_disc(&g,p,Vec2::ZERO,0.45);
        assert_eq!(end,p);assert_eq!(why,Blocked::InvalidStart);
        let (end,why)=move_disc(&g,Vec2::ZERO,Vec2::splat(f32::MAX),0.45);
        assert_eq!(end,Vec2::ZERO);assert_eq!(why,Blocked::SafetyStop);
    }
    #[test]
    fn formal_simulation_uses_the_same_contact_pipeline() {
        use crate::{simulation,world::{SimEnv,Unit},nav::Cell};
        let mut env=SimEnv{nav:grid(20,20),..Default::default()};
        let mut units=vec![Unit{idx:0,cell_x:5.0,cell_y:10.0,goal:Cell::new(15,10),vx:6.0,..Default::default()},
            Unit{idx:1,cell_x:6.0,cell_y:10.0,goal:Cell::new(1,10),vx:-6.0,..Default::default()}];
        for _ in 0..180 {
            let old:Vec<_>=units.iter().map(|u|agent(u.cell_x,u.cell_y)).collect();
            simulation::step_continuous(&mut env,&mut units);
            let next:Vec<_>=units.iter().map(|u|agent(u.cell_x,u.cell_y)).collect();
            assert!(swept_distance(&old[0],&old[1],&next[0],&next[1])>=0.89998);
            assert_eq!(env.runtime.contacts.unresolved_pairs,0);
        }
    }
    #[test]
    fn formal_simulation_predicts_a_wall_and_retains_contact_fallback_diagnostics() {
        use crate::{simulation,world::{SimEnv,Unit},nav::Cell};
        let mut g=grid(20,20);for y in 0..20 {g.cost[y*20+8]=0;}
        let mut env=SimEnv{nav:g,..Default::default()};
        let mut units=vec![Unit{idx:0,cell_x:7.0,cell_y:10.0,goal:Cell::new(15,10),vx:6.0,..Default::default()}];
        let mut reported=false;
        for _ in 0..20 {
            simulation::step_continuous(&mut env,&mut units);
            reported |= env.runtime.orca.static_constraints>0;
            assert_eq!(env.runtime.contacts.illegal_static_trajectories,0);
            assert_eq!(env.runtime.orca.contact_fallback,0);
        }
        assert!(reported);assert!(units[0].cell_x<7.05001);assert_eq!(units[0].goal,Cell::new(15,10));
    }
}
