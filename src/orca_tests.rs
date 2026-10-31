use super::*;

fn plane(x:f64,y:f64,offset:f64)->HalfPlane {HalfPlane::new(DVec2::new(x,y),offset).unwrap()}
fn agent(id:u64,p:DVec2,v:DVec2,r:f64)->Agent {Agent{id,position:p,velocity:v,radius:r}}
fn assert_feasible(s:Solution,p:&[HalfPlane],r:f64) {
    assert_eq!(s.status,SolveStatus::Feasible,"{s:?}");assert!(s.velocity.is_finite());
    assert!(length(s.velocity)<=r+FEASIBILITY_TOLERANCE*(1.0+r));
    for h in p {assert!(h.violation(s.velocity)<=FEASIBILITY_TOLERANCE*(1.0+r));}
    assert_eq!(s.violated,0);
}
fn separation(a:Agent,b:Agent,va:DVec2,vb:DVec2,horizon:f64)->f64 {
    let p=b.position-a.position;let v=vb-va;
    let time=if v.length_squared()>0.0 {(-p.dot(v)/v.length_squared()).clamp(0.0,horizon)}else{0.0};
    (p+v*time).length()
}

#[test]
fn analytic_disk_corner_parallel_and_tangent_solutions() {
    let mut solver=Solver::default();
    let s=solver.solve(&[],DVec2::new(3.0,4.0),2.0,0);assert_feasible(s,&[],2.0);
    assert!((s.velocity-DVec2::new(1.2,1.6)).length()<1e-12);
    let h=[plane(1.0,0.0,0.5),plane(0.0,1.0,0.25),plane(2.0,0.0,0.6)];
    let s=solver.solve(&h,DVec2::ZERO,1.0,0);assert_feasible(s,&h,1.0);assert!((s.velocity-DVec2::new(0.5,0.25)).length()<1e-12);
    let h=[plane(1.0,0.0,1.0),plane(1.0,1e-13,1.0)];
    let s=solver.solve(&h,DVec2::ZERO,1.0,0);assert_feasible(s,&h,1.0);assert!((s.velocity-DVec2::X).length()<1e-10);
    let h=[plane(0.0,1.0,0.5)];let s=solver.solve(&h,DVec2::new(5.0,0.0),2.0,0);assert_feasible(s,&h,2.0);
    assert!((s.velocity-DVec2::new(3.75f64.sqrt(),0.5)).length()<1e-12);
    let h=[plane(1.0,0.0,0.0)];assert_feasible(solver.solve(&h,DVec2::ZERO,0.0,1),&h,0.0);
}

#[test]
fn infeasible_sets_report_residuals_and_preserve_feasible_hard_boundaries() {
    let mut solver=Solver::default();let h=[plane(1.0,0.0,0.8),plane(-1.0,0.0,0.8)];
    let s=solver.solve(&h,DVec2::ZERO,1.0,0);assert!(matches!(s.status,SolveStatus::Infeasible{hard:false,..}));
    assert!(s.velocity.is_finite());assert!(length(s.velocity)<=1.0+1e-10);assert!(s.violated>0);assert!(s.max_violation>=0.8-1e-9);
    let h=[plane(1.0,0.0,0.0),plane(-1.0,0.0,1.0)];let s=solver.solve(&h,DVec2::ZERO,2.0,1);
    assert!(matches!(s.status,SolveStatus::Infeasible{hard:false,..}));assert_eq!(s.hard_violated,0);assert!(s.velocity.x>=-1e-8);
    let h=[plane(1.0,0.0,2.0)];let s=solver.solve(&h,DVec2::ZERO,1.0,1);
    assert!(matches!(s.status,SolveStatus::Infeasible{hard:true,..}));assert_eq!(s.hard_violated,1);assert!(s.velocity.is_finite());
}

#[test]
fn analytic_symmetric_head_on_uses_opposite_tangent_velocities() {
    let a=agent(0,DVec2::ZERO,DVec2::X,0.5);let b=agent(1,DVec2::new(4.0,0.0),-DVec2::X,0.5);
    let pa=unit_constraint(a,b,Responsibility::Reciprocal,2.0,0.1).unwrap();
    let pb=unit_constraint(b,a,Responsibility::Reciprocal,2.0,0.1).unwrap();let mut solver=Solver::default();
    let va=solver.solve(&[pa.plane],a.velocity,1.0,0);let vb=solver.solve(&[pb.plane],b.velocity,1.0,0);
    assert_feasible(va,&[pa.plane],1.0);assert_feasible(vb,&[pb.plane],1.0);
    let expected=DVec2::new(15.0/16.0,-15.0f64.sqrt()/16.0);
    assert!((va.velocity-expected).length()<1e-12);assert!((va.velocity+vb.velocity).length()<1e-12);
    assert!(separation(a,b,va.velocity,vb.velocity,2.0)>=1.0-1e-12);
}

#[test]
fn passive_neighbor_requires_full_responsibility_not_half() {
    let a=agent(0,DVec2::ZERO,DVec2::X*2.0,0.5);let b=agent(1,DVec2::new(4.0,0.0),DVec2::ZERO,0.5);
    let half=unit_constraint(a,b,Responsibility::Reciprocal,2.0,0.1).unwrap();
    let full=unit_constraint(a,b,Responsibility::Full,2.0,0.1).unwrap();let mut solver=Solver::default();
    let vh=solver.solve(&[half.plane],a.velocity,2.0,0);let vf=solver.solve(&[full.plane],a.velocity,2.0,0);
    assert_feasible(vh,&[half.plane],2.0);assert_feasible(vf,&[full.plane],2.0);
    assert_eq!(full.share,1.0);assert_eq!(half.share,0.5);
    assert!(separation(a,b,vh.velocity,DVec2::ZERO,2.0)<0.6,"half responsibility silently assumed a reaction");
    assert!(separation(a,b,vf.velocity,DVec2::ZERO,2.0)>=1.0-1e-12);
}

#[test]
fn zero_relative_velocity_and_different_radii_use_the_sum_of_radii() {
    let a=agent(0,DVec2::ZERO,DVec2::ZERO,0.2);let b=agent(1,DVec2::new(4.0,0.0),DVec2::ZERO,0.8);
    let h=unit_constraint(a,b,Responsibility::Reciprocal,2.0,0.1).unwrap();let mut solver=Solver::default();
    let s=solver.solve(&[h.plane],DVec2::X*2.0,2.0,0);assert_feasible(s,&[h.plane],2.0);assert!((s.velocity.x-0.75).abs()<1e-12);
    let h=unit_constraint(a,b,Responsibility::Full,2.0,0.1).unwrap();let s=solver.solve(&[h.plane],DVec2::X*2.0,2.0,0);
    assert_feasible(s,&[h.plane],2.0);assert!((s.velocity.x-1.5).abs()<1e-12);
    for i in 0..24 {let r1=0.1+i as f64*0.017;let r2=0.2+i as f64*0.023;
        let a=agent(0,DVec2::ZERO,DVec2::X*2.5,r1);let b=agent(1,DVec2::new(5.0,0.0),DVec2::ZERO,r2);
        let h=unit_constraint(a,b,Responsibility::Full,2.0,0.1).unwrap();let s=solver.solve(&[h.plane],a.velocity,2.5,0);
        assert_feasible(s,&[h.plane],2.5);assert!(separation(a,b,s.velocity,DVec2::ZERO,2.0)>=r1+r2-1e-10);
    }
}

#[test]
fn coincident_centres_and_zero_cutoff_vectors_have_finite_opposite_recovery() {
    let a=agent(4,DVec2::ZERO,DVec2::ZERO,0.45);let b=agent(7,DVec2::ZERO,DVec2::ZERO,0.45);
    let pa=unit_constraint(a,b,Responsibility::Reciprocal,2.0,1.0/60.0).unwrap();
    let pb=unit_constraint(b,a,Responsibility::Reciprocal,2.0,1.0/60.0).unwrap();
    assert!(pa.overlapping&&pa.short_horizon);assert!((pa.plane.normal+pb.plane.normal).length()<1e-12);
    let mut solver=Solver::default();let limited=solver.solve(&[pa.plane],DVec2::ZERO,6.0,0);
    assert!(matches!(limited.status,SolveStatus::Infeasible{..}));assert!(limited.velocity.is_finite());
    let va=solver.solve(&[pa.plane],DVec2::ZERO,60.0,0);let vb=solver.solve(&[pb.plane],DVec2::ZERO,60.0,0);
    assert_feasible(va,&[pa.plane],60.0);assert_feasible(vb,&[pb.plane],60.0);
    assert!((va.velocity+vb.velocity).length()<1e-10);assert!((va.velocity-vb.velocity).length()/60.0>=0.9-1e-12);
    let a=agent(0,DVec2::ZERO,DVec2::X,0.3);let b=agent(1,DVec2::new(0.2,0.0),-DVec2::X,0.4);
    let h=unit_constraint(a,b,Responsibility::Reciprocal,2.0,0.1).unwrap();let s=solver.solve(&[h.plane],DVec2::X,10.0,0);
    assert_feasible(s,&[h.plane],10.0);assert!((s.velocity.x+2.5).abs()<1e-12);
}

#[test]
fn invalid_and_degenerate_inputs_are_explicit_and_never_emit_nan_velocity() {
    let mut solver=Solver::default();
    for s in [solver.solve(&[],DVec2::new(f64::NAN,0.0),1.0,0),solver.solve(&[],DVec2::ZERO,-1.0,0),
        solver.solve(&[HalfPlane{normal:DVec2::ZERO,offset:1.0}],DVec2::ZERO,1.0,0),
        solver.solve(&[],DVec2::ZERO,1.0,1),solver.solve(&[],DVec2::ZERO,1e308,0)] {
        assert!(matches!(s.status,SolveStatus::InvalidInput{..}));assert!(s.velocity.is_finite());assert!(s.max_violation.is_finite());
    }
    let a=agent(0,DVec2::ZERO,DVec2::ZERO,0.45);let mut b=agent(1,DVec2::X,DVec2::ZERO,0.45);
    assert_eq!(unit_constraint(a,b,Responsibility::Reciprocal,0.0,0.1).unwrap_err(),InputError::NonPositiveTime);
    assert_eq!(unit_constraint(a,a,Responsibility::Reciprocal,2.0,0.1).unwrap_err(),InputError::DuplicateId);
    b.radius=-0.1;assert_eq!(unit_constraint(a,b,Responsibility::Reciprocal,2.0,0.1).unwrap_err(),InputError::InvalidRadius);
    assert!(HalfPlane::new(DVec2::ZERO,0.0).is_err());
}

// Independent geometry reference: enumerate the preferred point, line projections,
// line/line vertices and line/circle intersections; no incremental clipping or LP3.
fn geometric_minimum(planes:&[HalfPlane],preferred:DVec2,r:f64)->Option<DVec2> {
    let mut candidates=vec![DVec2::ZERO,preferred];let d=preferred.length();if d>0.0 {candidates.push(preferred/d*r);}
    for (i,h) in planes.iter().enumerate() {
        candidates.push(preferred+h.normal*(h.offset-h.normal.dot(preferred)));
        let remaining=r*r-h.offset*h.offset;if remaining>=0.0 {
            let tangent=DVec2::new(-h.normal.y,h.normal.x)*remaining.sqrt();let base=h.normal*h.offset;
            candidates.push(base+tangent);candidates.push(base-tangent);
        }
        for other in &planes[..i] {let determinant=h.normal.x*other.normal.y-h.normal.y*other.normal.x;
            if determinant.abs()>1e-14 {candidates.push(DVec2::new((h.offset*other.normal.y-h.normal.y*other.offset)/determinant,
                (h.normal.x*other.offset-h.offset*other.normal.x)/determinant));}
        }
    }
    candidates.into_iter().filter(|p|p.is_finite()&&p.length()<=r+1e-9&&planes.iter().all(|h|h.normal.dot(*p)>=h.offset-1e-9))
        .min_by(|a,b|a.distance_squared(preferred).total_cmp(&b.distance_squared(preferred)))
}
#[test]
fn incremental_solver_matches_independent_geometric_enumeration_and_input_permutations() {
    let mut seed=91u64;let mut random=||{seed=seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);(seed>>11) as f64/9007199254740992.0};
    let mut solver=Solver::default();
    for k in 0..512 {
        let r=0.2+random()*4.0;let witness=DVec2::new((random()-0.5)*r*0.6,(random()-0.5)*r*0.6);
        let preferred=DVec2::new((random()-0.5)*r*4.0,(random()-0.5)*r*4.0);let mut h=Vec::new();
        for _ in 0..(2+k%13) {let theta=random()*std::f64::consts::TAU;let n=DVec2::new(theta.cos(),theta.sin());h.push(HalfPlane::new(n,n.dot(witness)-(0.02+random()*0.7)*r).unwrap());}
        let expected=geometric_minimum(&h,preferred,r).expect("witness makes the domain nonempty");
        let s=solver.solve(&h,preferred,r,0);assert_feasible(s,&h,r);assert!(s.velocity.distance(expected)<2e-7,"case {k}: {:?} vs {expected:?}",s.velocity);
        h.reverse();let other=solver.solve(&h,preferred,r,0);assert_feasible(other,&h,r);assert!(s.velocity.distance(other.velocity)<2e-7);
    }
}

#[test]
fn fixed_original_cpp_reference_cases_match_and_keep_the_original_nan_counterexample() {
    let mut solver=Solver::default();let (mut lp_count,mut pair_count,mut nonfinite)=(0,0,0);
    for row in include_str!("../tests/data/orca-rvo2-v2.0.2.csv").lines().filter(|s|!s.starts_with('#')&&!s.is_empty()) {
        let fields:Vec<_>=row.split(',').collect();let num=|i:usize|fields[i].parse::<f64>().unwrap();
        if fields[0]=="lp" {
            let (hard,r,pref,n)=(num(2) as usize,num(3),DVec2::new(num(4),num(5)),num(6) as usize);
            let h:Vec<_>=(0..n).map(|i|plane(num(7+3*i),num(8+3*i),num(9+3*i))).collect();let end=7+3*n;
            let expected=DVec2::new(num(end+1),num(end+2));let s=solver.solve(&h,pref,r,hard);
            if num(end) as usize==n {assert_feasible(s,&h,r);}else{assert!(matches!(s.status,SolveStatus::Infeasible{..}));}
            assert!(s.velocity.distance(expected)<2e-4*(1.0+r),"original LP case {}: {:?} != {expected:?}",fields[1],s.velocity);lp_count+=1;
        }else{
            assert_eq!(fields[0],"pair");let (tau,dt)=(num(2),num(3));
            let a=agent(0,DVec2::new(num(4),num(5)),DVec2::new(num(6),num(7)),num(8));
            let b=agent(1,DVec2::new(num(9),num(10)),DVec2::new(num(11),num(12)),num(13));let cap=num(14);
            let pa=unit_constraint(a,b,Responsibility::Reciprocal,tau,dt).unwrap();let pb=unit_constraint(b,a,Responsibility::Reciprocal,tau,dt).unwrap();
            let va=solver.solve(&[pa.plane],DVec2::new(num(15),num(16)),cap,0);let vb=solver.solve(&[pb.plane],DVec2::new(num(17),num(18)),cap,0);
            assert!(va.velocity.is_finite()&&vb.velocity.is_finite());
            if fields[19..].iter().any(|s|s.parse::<f64>().ok().is_none_or(|v|!v.is_finite())) {
                assert_eq!(fields[1],"135");assert!(pa.overlapping&&pa.short_horizon);
                assert_feasible(va,&[pa.plane],cap);assert_feasible(vb,&[pb.plane],cap);
                assert!((b.position+vb.velocity*dt-a.position-va.velocity*dt).length()>=a.radius+b.radius-1e-10);nonfinite+=1;
            }else{
                let tolerance=2e-4*(1.0+cap);
                assert!(pa.plane.normal.distance(DVec2::new(num(19),num(20)))<2e-5);
                assert!((pa.plane.offset-num(21)).abs()<tolerance);
                assert!(va.velocity.distance(DVec2::new(num(22),num(23)))<tolerance,"original pair {} A",fields[1]);
                assert!(vb.velocity.distance(DVec2::new(num(24),num(25)))<tolerance,"original pair {} B",fields[1]);
                if va.status==SolveStatus::Feasible {assert_feasible(va,&[pa.plane],cap);}
            }
            pair_count+=1;
        }
    }
    assert_eq!((lp_count,pair_count,nonfinite),(133,131,1));
    println!("orca_original_reference lp={lp_count} pairs={pair_count} retained_original_nonfinite={nonfinite}");
}
