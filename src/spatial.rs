//! Reusable spatial buckets. Hash iteration never determines simulation order:
//! query IDs and contact pairs are sorted and deduplicated before use.
use std::{collections::HashMap,time::{Duration,Instant}};
use bevy::math::Vec2;

const MAX_CELLS: u64=4096;
const DENSE_BUCKET: usize=32;

#[derive(Clone,Copy,Debug)]
pub struct Aabb {pub min:Vec2,pub max:Vec2}
impl Aabb {
    pub fn point(p:Vec2)->Self {Self{min:p,max:p}}
    pub fn around(p:Vec2,r:f32)->Self {Self{min:p-Vec2::splat(r),max:p+Vec2::splat(r)}}
    pub fn swept(a:Vec2,b:Vec2,r:f32)->Self {Self{min:a.min(b)-Vec2::splat(r),max:a.max(b)+Vec2::splat(r)}}
    fn valid(self)->bool {self.min.is_finite()&&self.max.is_finite()&&self.min.cmple(self.max).all()}
    pub fn intersects(self,other:Self)->bool {
        !self.valid()||!other.valid()||(self.min.cmple(other.max).all()&&other.min.cmple(self.max).all())
    }
}

#[derive(Clone,Debug,Default)]
pub struct Stats {
    pub builds:usize,pub queries:usize,pub visited_entries:usize,pub returned:usize,
    pub peak_neighbors:usize,pub peak_bucket:usize,pub dense_bucket_events:usize,
    pub large_bounds:usize,pub fallback_queries:usize,pub candidate_pairs:usize,
    pub truncated:usize,pub build_time:Duration,pub query_time:Duration,
}
impl Stats {
    pub fn merge(&mut self,s:&Self) {
        self.builds+=s.builds;self.queries+=s.queries;self.visited_entries+=s.visited_entries;
        self.returned+=s.returned;self.peak_neighbors=self.peak_neighbors.max(s.peak_neighbors);
        self.peak_bucket=self.peak_bucket.max(s.peak_bucket);self.dense_bucket_events+=s.dense_bucket_events;
        self.large_bounds=self.large_bounds.max(s.large_bounds);self.fallback_queries+=s.fallback_queries;
        self.candidate_pairs+=s.candidate_pairs;self.truncated+=s.truncated;
        self.build_time+=s.build_time;self.query_time+=s.query_time;
    }
}

/// The current steering horizon is one actual fixed tick; longer predictive solvers
/// can use the same function with their own horizon. The minimum preserves steering reach.
pub fn neighbor_range(speed:f32,radius:f32,max_speed:f32,max_radius:f32,horizon:f32,minimum:f32)->f32 {
    minimum.max(radius+max_radius+(speed.abs()+max_speed.abs())*horizon.max(0.0))
}

#[derive(Debug)]
pub struct Index {
    cell_size:f32,buckets:HashMap<(i32,i32),Vec<usize>>,bounds:Vec<Aabb>,large:Vec<usize>,
    pub stats:Stats,
    #[cfg(test)] pub brute_force:bool,
}
impl Default for Index {
    fn default()->Self {Self::new(2.2)}
}
impl Index {
    pub fn new(cell_size:f32)->Self {
        assert!(cell_size.is_finite()&&cell_size>0.0);
        Self{cell_size,buckets:HashMap::new(),bounds:Vec::new(),large:Vec::new(),stats:Stats::default(),
            #[cfg(test)] brute_force:false}
    }
    pub fn begin_tick(&mut self) {self.stats=Stats::default();}
    fn cells(&self,b:Aabb)->Option<(i32,i32,i32,i32)> {
        if !b.valid() {return None;}
        let (x0,x1)=((b.min.x/self.cell_size).floor() as i32,(b.max.x/self.cell_size).floor() as i32);
        let (y0,y1)=((b.min.y/self.cell_size).floor() as i32,(b.max.y/self.cell_size).floor() as i32);
        let count=(x1 as i64-x0 as i64+1) as u64;
        let count=count.checked_mul((y1 as i64-y0 as i64+1) as u64)?;
        (count<=MAX_CELLS).then_some((x0,x1,y0,y1))
    }
    fn insert(&mut self,id:usize,b:Aabb) {
        if let Some((x0,x1,y0,y1))=self.cells(b) {
            for y in y0..=y1 {for x in x0..=x1 {
                let bucket=self.buckets.entry((x,y)).or_default();bucket.push(id);
                self.stats.peak_bucket=self.stats.peak_bucket.max(bucket.len());
                if bucket.len()==DENSE_BUCKET+1 {self.stats.dense_bucket_events+=1;}
            }}
        }else{self.large.push(id);self.stats.large_bounds=self.stats.large_bounds.max(self.large.len());}
    }
    pub fn rebuild(&mut self,bounds:&[Aabb]) {
        let start=Instant::now();self.stats.builds+=1;
        for bucket in self.buckets.values_mut() {bucket.clear();}
        self.bounds.clear();self.bounds.extend_from_slice(bounds);self.large.clear();
        for (id,b) in bounds.iter().copied().enumerate() {self.insert(id,b);}
        self.stats.build_time+=start.elapsed();
    }
    /// Used by contact projection after each changed body, preserving the original
    /// lexicographic pair order even when a correction creates a new neighbor.
    pub fn update(&mut self,id:usize,b:Aabb) {
        let start=Instant::now();let old=self.bounds[id];
        if let Some((x0,x1,y0,y1))=self.cells(old) {
            for y in y0..=y1 {for x in x0..=x1 {
                if let Some(bucket)=self.buckets.get_mut(&(x,y)) {bucket.retain(|v|*v!=id);}
            }}
        }else{self.large.retain(|v|*v!=id);}
        self.bounds[id]=b;self.insert(id,b);self.stats.build_time+=start.elapsed();
    }
    pub fn query(&mut self,area:Aabb,out:&mut Vec<usize>) {
        let start=Instant::now();self.stats.queries+=1;out.clear();
        #[cfg(test)] if self.brute_force {
            for (i,b) in self.bounds.iter().enumerate() {self.stats.visited_entries+=1;if b.intersects(area){out.push(i);}}
            self.stats.returned+=out.len();self.stats.query_time+=start.elapsed();return;
        }
        if let Some((x0,x1,y0,y1))=self.cells(area) {
            for y in y0..=y1 {for x in x0..=x1 {
                if let Some(bucket)=self.buckets.get(&(x,y)) {
                    self.stats.visited_entries+=bucket.len();
                    out.extend(bucket.iter().copied().filter(|i|self.bounds[*i].intersects(area)));
                }
            }}
            out.extend(self.large.iter().copied().filter(|i|self.bounds[*i].intersects(area)));
            out.sort_unstable();out.dedup();
        }else{
            self.stats.fallback_queries+=1;
            for (id,b) in self.bounds.iter().enumerate() {self.stats.visited_entries+=1;if b.intersects(area){out.push(id);}}
        }
        self.stats.returned+=out.len();self.stats.query_time+=start.elapsed();
    }
    /// Point-index radius query. No neighbor cap: imminent threats cannot be truncated.
    pub fn query_radius(&mut self,p:Vec2,r:f32,out:&mut Vec<usize>) {
        self.query(Aabb::around(p,r),out);
        out.retain(|id|!self.bounds[*id].valid()||self.bounds[*id].min.distance_squared(p)<=r*r);
        self.stats.peak_neighbors=self.stats.peak_neighbors.max(out.len());
    }
    pub fn pairs(&mut self,out:&mut Vec<(usize,usize)>) {
        let start=Instant::now();out.clear();
        #[cfg(test)] if self.brute_force {
            for i in 0..self.bounds.len() {for j in i+1..self.bounds.len() {
                if self.bounds[i].intersects(self.bounds[j]) {out.push((i,j));}
            }}
            self.stats.candidate_pairs+=out.len();self.stats.query_time+=start.elapsed();return;
        }
        for bucket in self.buckets.values() {
            for a in 0..bucket.len() {for b in a+1..bucket.len() {
                let (i,j)=(bucket[a].min(bucket[b]),bucket[a].max(bucket[b]));
                if i!=j&&self.bounds[i].intersects(self.bounds[j]) {out.push((i,j));}
            }}
        }
        for &i in &self.large {for j in 0..self.bounds.len() {
            if i!=j&&self.bounds[i].intersects(self.bounds[j]) {out.push((i.min(j),i.max(j)));}
        }}
        out.sort_unstable();out.dedup();self.stats.candidate_pairs+=out.len();self.stats.query_time+=start.elapsed();
    }
    #[cfg(test)] pub fn capacities(&self)->(usize,usize,usize) {
        (self.bounds.capacity(),self.large.capacity(),self.buckets.values().map(Vec::capacity).sum())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn queries_and_swept_pairs_match_brute_force_across_boundaries_and_dense_buckets() {
        for count in [1,17,256,1024] {
            let mut bounds=Vec::new();
            for i in 0..count {
                let p=Vec2::new((i%31) as f32*0.37-4.4,(i/31) as f32*0.29-2.2);
                bounds.push(Aabb::swept(p,p+Vec2::new((i%3) as f32-1.0,(i%5) as f32*0.4),0.45));
            }
            let mut index=Index::default();index.rebuild(&bounds);let mut got=Vec::new();let mut expected=Vec::new();
            for k in 0..64 {
                let area=Aabb::around(Vec2::new(k as f32*0.19-4.4,k as f32*0.08-2.2),k as f32*0.07);
                index.query(area,&mut got);
                let expected:Vec<_>=bounds.iter().enumerate().filter_map(|(i,b)|b.intersects(area).then_some(i)).collect();
                assert_eq!(got,expected);
            }
            let mut pairs=Vec::new();index.pairs(&mut pairs);
            for i in 0..count {for j in i+1..count {if bounds[i].intersects(bounds[j]) {expected.push((i,j));}}}
            assert_eq!(pairs,expected);
        }
    }
    #[test]
    fn dynamic_updates_large_ranges_and_buffer_reuse_do_not_drop_objects() {
        let mut bounds=vec![Aabb::point(Vec2::new(-2.2,-2.2));256];let mut index=Index::default();
        index.rebuild(&bounds);let capacity=index.capacities();index.rebuild(&bounds);assert_eq!(capacity,index.capacities());
        let mut out=Vec::new();index.query_radius(Vec2::new(-2.2,-2.2),0.01,&mut out);assert_eq!(out.len(),256);
        assert!(index.stats.peak_bucket>=256&&index.stats.dense_bucket_events>0);assert_eq!(index.stats.truncated,0);
        bounds[0]=Aabb::point(Vec2::new(100.0,100.0));index.update(0,bounds[0]);
        index.query_radius(Vec2::new(100.0,100.0),0.01,&mut out);assert_eq!(out,vec![0]);
        index.query(Aabb::around(Vec2::ZERO,1000.0),&mut out);assert_eq!(out.len(),256);assert!(index.stats.fallback_queries>0);
        bounds[1]=Aabb::swept(Vec2::splat(-1000.0),Vec2::splat(1000.0),0.45);index.rebuild(&bounds);
        let mut pairs=Vec::new();index.pairs(&mut pairs);
        assert!(pairs.contains(&(0,1)));assert!(index.stats.large_bounds>0);
        assert!(neighbor_range(60.0,0.45,60.0,0.45,0.5,2.2)>60.0);
    }
    #[test]
    fn profile_sparse_and_dense_256_and_1024_queries() {
        for n in [256,1024] {for dense in [false,true] {
            let bounds:Vec<_>=(0..n).map(|i|Aabb::around(if dense {Vec2::ZERO}else{Vec2::new((i%32) as f32*10.0,(i/32) as f32*10.0)},0.45)).collect();
            let mut index=Index::default();let mut neighbors=Vec::new();let mut pairs=Vec::new();
            let start=Instant::now();index.rebuild(&bounds);
            for b in &bounds {index.query(*b,&mut neighbors);assert_eq!(neighbors.len(),if dense {n}else{1});}
            index.pairs(&mut pairs);assert_eq!(pairs.len(),if dense {n*(n-1)/2}else{0});
            println!("spatial_profile count={n} dense={dense} elapsed_us={} build_us={} query_us={} queries={} entries={} pairs={} peak_bucket={} truncated={}",
                start.elapsed().as_micros(),index.stats.build_time.as_micros(),index.stats.query_time.as_micros(),index.stats.queries,index.stats.visited_entries,
                pairs.len(),index.stats.peak_bucket,index.stats.truncated);
        }}
    }
}
