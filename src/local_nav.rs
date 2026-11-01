//! Near parking routes reuse nav's graph solver and collision's physical predicates.
//! A half-spacing roadmap represents the free lanes between fractional parking slots.
use std::collections::{BTreeMap,BTreeSet};
use bevy::math::Vec2;
use crate::{collision,formation::{Body,Target,PARKING_SPACING},nav::{self,Cell,CostGrid},spatial::{Aabb,Index}};

pub struct Query<'a> {
    pub grid:&'a CostGrid,pub bodies:&'a [Body],pub index:&'a mut Index,pub neighbors:&'a mut Vec<usize>,
    pub owner:i32,pub radius:f32,pub max_radius:f32,
}
impl Query<'_> {
    pub fn clear(&mut self,a:Vec2,b:Vec2,ignore_owner:bool)->bool {
        if !collision::disc_segment_clear(self.grid,a,b,self.radius) {return false;}
        self.index.query(Aabb::swept(a,b,self.radius+self.max_radius),self.neighbors);
        !self.neighbors.iter().any(|i|{
            let body=&self.bodies[*i];
            !(ignore_owner&&body.id==self.owner)&&!collision::segment_avoids_disc(a,b,body.position,self.radius+body.radius)
        })
    }
}
type Key=(u64,i32,i32,u32,u32,u32,u32);
struct FineGrid {tick:u64,grid:CostGrid,origin:Vec2}
struct Pending {grid:CostGrid,origin:Vec2,step:f32,search:nav::PathSearch,original:bool,seen:u64}
pub enum RouteResult {Ready(Vec<Vec2>),Pending,NoPath}
#[derive(Default)]
pub struct Routes {grids:BTreeMap<Key,FineGrid>,scratch:CostGrid,pending:BTreeMap<nav::SearchKey,Pending>,pub budget:nav::ReplanBudget}
impl Routes {
    pub fn begin_tick(&mut self,tick:u64) {self.budget.begin(tick);self.pending.retain(|_,p|p.seen.saturating_add(1)>=tick);}
    pub fn prune(&mut self,active:&BTreeSet<u64>) {self.grids.retain(|key,_|active.contains(&key.0));self.pending.retain(|key,_|active.contains(&key.command));}
    pub fn path(&mut self,target:&Target,from:Vec2,goal:Vec2,command:u64,tick:u64,q:&mut Query<'_>)->RouteResult {
        self.begin_tick(tick);
        let request=nav::SearchKey{owner:q.owner,command,x:goal.x.to_bits(),y:goal.y.to_bits(),class:0};
        if !q.clear(goal,goal,true) {self.pending.remove(&request);self.budget.finish(request,false,30);return RouteResult::NoPath;}
        if let Some(p)=self.pending.get_mut(&request) {p.seen=tick;}
        let permit=self.budget.request(request,tick);if permit==0 {return RouteResult::Pending;}
        let step=PARKING_SPACING*0.5;
        if !self.pending.contains_key(&request) {
        let key=(command,target.center.x,target.center.y,target.phase.x.to_bits(),target.phase.y.to_bits(),target.area_radius.to_bits(),target.radius.to_bits());
        if self.grids.get(&key).is_none_or(|g|g.tick!=tick) {
            let anchor=Vec2::new(target.center.x as f32,target.center.y as f32)+target.phase*PARKING_SPACING;
            let pad=target.area_radius+crate::formation::LOCAL_APPROACH_MARGIN+PARKING_SPACING*2.0;
            let lo=((Vec2::new(target.center.x as f32,target.center.y as f32)-Vec2::splat(pad)).max(Vec2::splat(-0.5))-anchor)/step;
            let hi=((Vec2::new(target.center.x as f32,target.center.y as f32)+Vec2::splat(pad)).min(Vec2::new(q.grid.w as f32-0.5,q.grid.h as f32-0.5))-anchor)/step;
            let low=lo.floor();let high=hi.ceil();let w=(high.x-low.x) as i32+1;let h=(high.y-low.y) as i32+1;
            let origin=anchor+low*step;
            let cached=self.grids.entry(key).or_insert_with(||FineGrid{tick,grid:CostGrid::default(),origin});
            cached.tick=tick;cached.origin=origin;cached.grid.w=w;cached.grid.h=h;cached.grid.cut_corners=true;
            cached.grid.cost.resize((w*h) as usize,0);
            for y in 0..h {for x in 0..w {
                let p=origin+Vec2::new(x as f32,y as f32)*step;
                cached.grid.cost[(y*w+x) as usize]=if q.clear(p,p,false){q.grid.at(p.x.round() as i32,p.y.round() as i32) as u8}else{0};
            }}
        }
        let cached=self.grids.get(&key).unwrap();let origin=cached.origin;
        self.scratch.w=cached.grid.w;self.scratch.h=cached.grid.h;self.scratch.cut_corners=true;
        self.scratch.cost.clear();self.scratch.cost.extend_from_slice(&cached.grid.cost);
        // The shared mask contains every stationary body. Only this requester's own
        // previous footprint can be opened, using the full physical predicate again.
        if q.bodies.binary_search_by_key(&q.owner,|b|b.id).is_ok() {
            let center=(from-origin)/step;let span=((q.radius+q.max_radius)/step).ceil() as i32+1;
            for y in (center.y.round() as i32-span).max(0)..=(center.y.round() as i32+span).min(self.scratch.h-1) {
                for x in (center.x.round() as i32-span).max(0)..=(center.x.round() as i32+span).min(self.scratch.w-1) {
                    let p=origin+Vec2::new(x as f32,y as f32)*step;
                    self.scratch.cost[(y*self.scratch.w+x) as usize]=if q.clear(p,p,true){q.grid.at(p.x.round() as i32,p.y.round() as i32) as u8}else{0};
                }
            }
        }
        let search=make_search(&self.scratch,origin,step,from,goal,q);
        self.pending.insert(request,Pending{grid:self.scratch.clone(),origin,step,search,original:false,seen:tick});
        }
        let p=self.pending.get_mut(&request).unwrap();let origin=p.origin;let step=p.step;let before=p.search.expanded;
        let status=p.search.advance(&p.grid,permit,&|_,_|false,&mut |a,b|q.clear(origin+Vec2::new(a.x as f32,a.y as f32)*step,origin+Vec2::new(b.x as f32,b.y as f32)*step,true).then_some(0.0));
        self.budget.record(p.search.expanded-before);
        if status==nav::SearchStatus::Pending {return RouteResult::Pending;}
        if status==nav::SearchStatus::Found {
            let mut path:Vec<_>=p.search.cells.iter().map(|c|origin+Vec2::new(c.x as f32,c.y as f32)*step).collect();path.push(goal);
            let legal=path.windows(2).all(|edge|q.clear(edge[0],edge[1],true));
            if legal {if let Some(first)=path.iter().position(|p|q.clear(from,*p,true)) {
                self.pending.remove(&request);self.budget.finish(request,true,0);return RouteResult::Ready(path.into_iter().skip(first).collect());
            }}
            self.pending.remove(&request);self.budget.finish(request,false,0);return RouteResult::Pending;
        }
        if !p.original {
            // Preserve original terrain centres for a narrow corridor which does
            // not lie on the parking lattice, using the same sliced A* queue.
            let original=q.grid.clone();let search=make_search(&original,Vec2::ZERO,1.0,from,goal,q);
            *p=Pending{grid:original,origin:Vec2::ZERO,step:1.0,search,original:true,seen:tick};return RouteResult::Pending;
        }
        self.pending.remove(&request);self.budget.finish(request,false,30);RouteResult::NoPath
    }
}

fn endpoints(g:&CostGrid,origin:Vec2,step:f32,p:Vec2,q:&mut Query<'_>)->Vec<Cell> {
    let c=(p-origin)/step;let span=((q.radius+q.max_radius+step)/step).ceil() as i32;
    let mut out=Vec::new();
    for y in c.y.round() as i32-span..=c.y.round() as i32+span {for x in c.x.round() as i32-span..=c.x.round() as i32+span {
        if !g.walkable(x,y) {continue;}let point=origin+Vec2::new(x as f32,y as f32)*step;
        if q.clear(p,point,true) {out.push(Cell::new(x,y));}
    }}
    out.sort_by(|a,b|{
        let distance=|c:&Cell|(origin+Vec2::new(c.x as f32,c.y as f32)*step-p).length_squared();
        distance(a).total_cmp(&distance(b)).then(a.x.cmp(&b.x)).then(a.y.cmp(&b.y))
    });out
}
fn make_search(g:&CostGrid,origin:Vec2,step:f32,from:Vec2,goal:Vec2,q:&mut Query<'_>)->nav::PathSearch {
    let starts=endpoints(g,origin,step,from,q);let ends=endpoints(g,origin,step,goal,q);
    nav::PathSearch::multi(g,&starts,&ends)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::formation::{self,Reservations};
    fn target(g:&CostGrid,from:Vec2,goal:Cell)->Target {
        let body=Body{id:0,position:from,radius:0.45,departing:true,idle:false};
        formation::plan(g,&formation::components(g),goal,&[body],&[body],&BTreeSet::from([0]),&mut Reservations::default(),None)
            .unwrap().targets.remove(0).1
    }
    fn complete(routes:&mut Routes,t:&Target,from:Vec2,goal:Vec2,q:&mut Query<'_>)->Vec<Vec2> {
        for tick in 0..2000 {match routes.path(t,from,goal,1,tick,q) {
            RouteResult::Ready(path)=>return path,RouteResult::NoPath=>return Vec::new(),RouteResult::Pending=>assert!(routes.budget.expanded<=nav::ReplanBudget::LIMIT),
        }}panic!("near path queue did not complete")
    }
    #[test]
    fn fractional_parking_routes_avoid_stationary_bodies_and_open_only_the_requester_mask() {
        let g=CostGrid{w:24,h:18,cost:vec![1;24*18],..Default::default()};let from=Vec2::new(9.25,8.55);let goal=Vec2::new(12.55,8.55);
        let mut t=target(&g,from,Cell::new(12,8));t.position=goal;t.phase=Vec2::splat(0.5);
        let mut bodies=vec![Body{id:0,position:from,radius:0.45,departing:false,idle:false},
            Body{id:1,position:Vec2::new(10.35,8.55),radius:0.45,departing:false,idle:true}];
        let mut index=Index::default();let mut scratch=Vec::new();index.rebuild(&bodies.iter().map(|b|Aabb::point(b.position)).collect::<Vec<_>>());
        let mut routes=Routes::default();
        {
            let mut q=Query{grid:&g,bodies:&bodies,index:&mut index,neighbors:&mut scratch,owner:0,radius:0.45,max_radius:0.45};
            assert!(!q.clear(from,goal,true));let path=complete(&mut routes,&t,from,goal,&mut q);assert!(!path.is_empty());
            let mut previous=from;for p in path {assert!(q.clear(previous,p,true));previous=p;}
            assert_eq!(previous,goal);
        }
        bodies[1].position=goal;index.rebuild(&bodies.iter().map(|b|Aabb::point(b.position)).collect::<Vec<_>>());
        let mut q=Query{grid:&g,bodies:&bodies,index:&mut index,neighbors:&mut scratch,owner:0,radius:0.45,max_radius:0.45};
        assert!(matches!(routes.path(&t,from,goal,1,2001,&mut q),RouteResult::NoPath));
    }
    #[test]
    fn original_centres_preserve_a_narrow_terrain_turn_not_on_the_parking_lattice() {
        let mut g=CostGrid{w:24,h:20,cost:vec![0;24*20],..Default::default()};
        for x in 18..=20 {g.cost[12*24+x]=1;}for y in 12..=14 {g.cost[y*24+20]=1;}
        let from=Vec2::new(18.0,12.0);let goal=Vec2::new(20.0,14.0);let t=target(&g,from,Cell::new(20,14));
        let mut index=Index::default();index.rebuild(&[]);let mut neighbors=Vec::new();
        let mut q=Query{grid:&g,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:0,radius:0.45,max_radius:0.45};
        let path=complete(&mut Routes::default(),&t,from,goal,&mut q);assert!(!path.is_empty());assert!(path.contains(&Vec2::new(20.0,12.0)));
        let mut previous=from;for p in path {assert!(q.clear(previous,p,true));previous=p;}assert_eq!(previous,goal);
    }
    #[test]
    fn disconnected_fractional_endpoint_sets_do_not_repeat_exhaustive_searches() {
        let mut g=CostGrid{w:24,h:18,cost:vec![1;24*18],..Default::default()};
        for y in 0..18 {g.cost[y*24+12]=0;g.cost[y*24+13]=0;}
        let mut index=Index::default();index.rebuild(&[]);let mut neighbors=Vec::new();
        let mut q=Query{grid:&g,bodies:&[],index:&mut index,neighbors:&mut neighbors,owner:0,radius:0.45,max_radius:0.45};
        let mut path=make_search(&g,Vec2::ZERO,1.0,Vec2::new(8.1,8.1),Vec2::new(17.1,8.1),&mut q);
        while path.status==nav::SearchStatus::Pending {path.advance(&g,64,&|_,_|false,&mut |a,b|q.clear(Vec2::new(a.x as f32,a.y as f32),Vec2::new(b.x as f32,b.y as f32),true).then_some(0.0));}
        assert_eq!(path.status,nav::SearchStatus::NoPath);
        assert!(index.stats.queries<64*g.cost.len(),"queries={}",index.stats.queries);
    }
}
