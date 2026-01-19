use recast_common::Error;
use std::f32;

pub const DT_MAX_PATTERN_DIVS: usize = 32;
pub const DT_MAX_PATTERN_RINGS: usize = 4;

#[derive(Debug, Clone)]
pub struct DtObstacleCircle {
    pub p: [f32; 3],    // Position of the obstacle
    pub vel: [f32; 3],  // Velocity of the obstacle
    pub dvel: [f32; 3], // Desired velocity of the obstacle
    pub rad: f32,       // Radius of the obstacle
    pub dp: [f32; 3],   // Use for side selection during sampling
    pub np: [f32; 3],   // Use for side selection during sampling
}

impl Default for DtObstacleCircle {
    fn default() -> Self {
        Self {
            p: [0.0; 3],
            vel: [0.0; 3],
            dvel: [0.0; 3],
            rad: 0.0,
            dp: [0.0; 3],
            np: [0.0; 3],
        }
    }
}

#[derive(Debug, Clone)]
pub struct DtObstacleSegment {
    pub p: [f32; 3], // Start point of the obstacle segment
    pub q: [f32; 3], // End point of the obstacle segment
    pub touch: bool,
}

impl Default for DtObstacleSegment {
    fn default() -> Self {
        Self {
            p: [0.0; 3],
            q: [0.0; 3],
            touch: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct DtObstacleAvoidanceParams {
    pub vel_bias: f32,       // Velocity bias
    pub weight_des_vel: f32, // Weight for desired velocity
    pub weight_cur_vel: f32, // Weight for current velocity
    pub weight_side: f32,    // Weight for side preference
    pub weight_toi: f32,     // Weight for time of impact
    pub horiz_time: f32,     // Horizon time
    pub grid_size: u8,       // Grid size for grid-based sampling
    pub adaptive_divs: u8,   // Adaptive divisions for adaptive sampling
    pub adaptive_rings: u8,  // Adaptive rings for adaptive sampling
    pub adaptive_depth: u8,  // Adaptive depth for adaptive sampling
}

impl Default for DtObstacleAvoidanceParams {
    fn default() -> Self {
        Self {
            vel_bias: 0.4,
            weight_des_vel: 2.0,
            weight_cur_vel: 0.75,
            weight_side: 0.75,
            weight_toi: 2.5,
            horiz_time: 2.5,
            grid_size: 33,
            adaptive_divs: 7,
            adaptive_rings: 2,
            adaptive_depth: 5,
        }
    }
}

#[derive(Debug)]
pub struct DtObstacleAvoidanceDebugData {
    n_samples: usize,
    max_samples: usize,
    vel: Vec<f32>,
    ssize: Vec<f32>,
    pen: Vec<f32>,
    vpen: Vec<f32>,
    vcpen: Vec<f32>,
    spen: Vec<f32>,
    tpen: Vec<f32>,
}

impl Default for DtObstacleAvoidanceDebugData {
    fn default() -> Self {
        Self::new()
    }
}

impl DtObstacleAvoidanceDebugData {
    pub fn new() -> Self {
        Self {
            n_samples: 0,
            max_samples: 0,
            vel: Vec::new(),
            ssize: Vec::new(),
            pen: Vec::new(),
            vpen: Vec::new(),
            vcpen: Vec::new(),
            spen: Vec::new(),
            tpen: Vec::new(),
        }
    }

    pub fn init(&mut self, max_samples: usize) -> bool {
        self.max_samples = max_samples;
        self.vel = vec![0.0; max_samples * 3];
        self.ssize = vec![0.0; max_samples];
        self.pen = vec![0.0; max_samples];
        self.vpen = vec![0.0; max_samples];
        self.vcpen = vec![0.0; max_samples];
        self.spen = vec![0.0; max_samples];
        self.tpen = vec![0.0; max_samples];
        true
    }

    pub fn reset(&mut self) {
        self.n_samples = 0;
    }

    #[allow(clippy::too_many_arguments)]
    pub fn add_sample(
        &mut self,
        vel: &[f32; 3],
        ssize: f32,
        pen: f32,
        vpen: f32,
        vcpen: f32,
        spen: f32,
        tpen: f32,
    ) {
        if self.n_samples >= self.max_samples {
            return;
        }

        let idx = self.n_samples;
        self.vel[idx * 3] = vel[0];
        self.vel[idx * 3 + 1] = vel[1];
        self.vel[idx * 3 + 2] = vel[2];
        self.ssize[idx] = ssize;
        self.pen[idx] = pen;
        self.vpen[idx] = vpen;
        self.vcpen[idx] = vcpen;
        self.spen[idx] = spen;
        self.tpen[idx] = tpen;
        self.n_samples += 1;
    }

    pub fn normalize_samples(&mut self) {
        normalize_array(&mut self.pen, self.n_samples);
        normalize_array(&mut self.vpen, self.n_samples);
        normalize_array(&mut self.vcpen, self.n_samples);
        normalize_array(&mut self.spen, self.n_samples);
        normalize_array(&mut self.tpen, self.n_samples);
    }

    pub fn get_sample_count(&self) -> usize {
        self.n_samples
    }

    pub fn get_sample_velocity(&self, i: usize) -> Option<&[f32]> {
        if i >= self.n_samples {
            return None;
        }
        Some(&self.vel[i * 3..i * 3 + 3])
    }

    pub fn get_sample_size(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.ssize[i])
    }

    pub fn get_sample_penalty(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.pen[i])
    }

    pub fn get_sample_desired_velocity_penalty(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.vpen[i])
    }

    pub fn get_sample_current_velocity_penalty(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.vcpen[i])
    }

    pub fn get_sample_preferred_side_penalty(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.spen[i])
    }

    pub fn get_sample_collision_time_penalty(&self, i: usize) -> Option<f32> {
        if i >= self.n_samples {
            return None;
        }
        Some(self.tpen[i])
    }
}

pub struct DtObstacleAvoidanceQuery {
    params: DtObstacleAvoidanceParams,
    inv_horiz_time: f32,
    vmax: f32,
    inv_vmax: f32,

    max_circles: usize,
    circles: Vec<DtObstacleCircle>,
    n_circles: usize,

    max_segments: usize,
    segments: Vec<DtObstacleSegment>,
    n_segments: usize,
}

impl Default for DtObstacleAvoidanceQuery {
    fn default() -> Self {
        Self::new()
    }
}

impl DtObstacleAvoidanceQuery {
    pub fn new() -> Self {
        Self {
            params: DtObstacleAvoidanceParams::default(),
            inv_horiz_time: 0.0,
            vmax: 0.0,
            inv_vmax: 0.0,
            max_circles: 0,
            circles: Vec::new(),
            n_circles: 0,
            max_segments: 0,
            segments: Vec::new(),
            n_segments: 0,
        }
    }

    pub fn init(&mut self, max_circles: usize, max_segments: usize) -> bool {
        self.max_circles = max_circles;
        self.max_segments = max_segments;
        self.circles = vec![DtObstacleCircle::default(); max_circles];
        self.segments = vec![DtObstacleSegment::default(); max_segments];
        true
    }

    pub fn reset(&mut self) {
        self.n_circles = 0;
        self.n_segments = 0;
    }

    pub fn add_circle(&mut self, pos: &[f32; 3], rad: f32, vel: &[f32; 3], dvel: &[f32; 3]) {
        if self.n_circles >= self.max_circles {
            return;
        }

        let circle = &mut self.circles[self.n_circles];
        circle.p = *pos;
        circle.rad = rad;
        circle.vel = *vel;
        circle.dvel = *dvel;
        self.n_circles += 1;
    }

    pub fn add_segment(&mut self, p: &[f32; 3], q: &[f32; 3]) {
        if self.n_segments >= self.max_segments {
            return;
        }

        let segment = &mut self.segments[self.n_segments];
        segment.p = *p;
        segment.q = *q;
        segment.touch = false;
        self.n_segments += 1;
    }

    #[allow(clippy::too_many_arguments)]
    pub fn sample_velocity_grid(
        &mut self,
        pos: &[f32; 3],
        rad: f32,
        vmax: f32,
        vel: &[f32; 3],
        dvel: &[f32; 3],
        nvel: &mut [f32; 3],
        params: &DtObstacleAvoidanceParams,
        mut debug: Option<&mut DtObstacleAvoidanceDebugData>,
    ) -> Result<usize, Error> {
        self.prepare(pos, dvel);
        self.params = *params;
        self.inv_horiz_time = 1.0 / params.horiz_time;
        self.vmax = vmax;
        self.inv_vmax = if vmax > 0.0 { 1.0 / vmax } else { f32::MAX };

        if let Some(ref mut debug_data) = debug {
            debug_data.reset();
        }

        let cvx = dvel[0] * params.vel_bias;
        let cvz = dvel[2] * params.vel_bias;
        let cs = vmax * 2.0 / (params.grid_size as f32 - 1.0);
        let half = (params.grid_size as f32 - 1.0) * cs * 0.5;

        let mut min_penalty = f32::MAX;
        let mut ns = 0;

        for y in 0..params.grid_size {
            for x in 0..params.grid_size {
                let vcand = [
                    cvx + cs * (x as f32) - half,
                    0.0,
                    cvz + cs * (y as f32) - half,
                ];

                if sqr(vcand[0]) + sqr(vcand[2]) > sqr(vmax + cs / 2.0) {
                    continue;
                }

                let penalty = self.process_sample(
                    &vcand,
                    cs,
                    pos,
                    rad,
                    vel,
                    dvel,
                    min_penalty,
                    debug.as_deref_mut(),
                );
                ns += 1;
                if penalty < min_penalty {
                    min_penalty = penalty;
                    *nvel = vcand;
                }
            }
        }

        Ok(ns)
    }

    #[allow(clippy::too_many_arguments)]
    pub fn sample_velocity_adaptive(
        &mut self,
        pos: &[f32; 3],
        rad: f32,
        vmax: f32,
        vel: &[f32; 3],
        dvel: &[f32; 3],
        nvel: &mut [f32; 3],
        params: &DtObstacleAvoidanceParams,
        mut debug: Option<&mut DtObstacleAvoidanceDebugData>,
    ) -> Result<usize, Error> {
        self.prepare(pos, dvel);
        self.params = *params;
        self.inv_horiz_time = 1.0 / params.horiz_time;
        self.vmax = vmax;
        self.inv_vmax = if vmax > 0.0 { 1.0 / vmax } else { f32::MAX };

        if let Some(ref mut debug_data) = debug {
            debug_data.reset();
        }

        let cvx = dvel[0] * params.vel_bias;
        let cvz = dvel[2] * params.vel_bias;

        let mut min_penalty = f32::MAX;
        let mut ns = 0;

        // Sample center point
        let mut vcand = [cvx, 0.0, cvz];
        let cs = vmax * 4.0 / (params.adaptive_depth as f32);

        let penalty = self.process_sample(
            &vcand,
            cs,
            pos,
            rad,
            vel,
            dvel,
            min_penalty,
            debug.as_deref_mut(),
        );
        ns += 1;
        if penalty < min_penalty {
            min_penalty = penalty;
            *nvel = vcand;
        }

        // Sample adaptive pattern
        for ring in 1..=params.adaptive_rings as usize {
            let mut divs = 1;
            for _level in 0..params.adaptive_depth {
                let da = std::f32::consts::PI * 2.0 / divs as f32;
                let ca = (ring as f32) * vmax / (params.adaptive_rings as f32);

                for i in 0..divs {
                    let a = i as f32 * da;
                    vcand = [cvx + ca * a.cos(), 0.0, cvz + ca * a.sin()];

                    if sqr(vcand[0]) + sqr(vcand[2]) > sqr(vmax) {
                        continue;
                    }

                    let penalty = self.process_sample(
                        &vcand,
                        cs,
                        pos,
                        rad,
                        vel,
                        dvel,
                        min_penalty,
                        debug.as_deref_mut(),
                    );
                    ns += 1;
                    if penalty < min_penalty {
                        min_penalty = penalty;
                        *nvel = vcand;
                    }
                }

                divs = std::cmp::min(divs * 2, params.adaptive_divs as usize);
            }
        }

        Ok(ns)
    }

    pub fn get_obstacle_circle_count(&self) -> usize {
        self.n_circles
    }

    pub fn get_obstacle_circle(&self, i: usize) -> Option<&DtObstacleCircle> {
        if i >= self.n_circles {
            None
        } else {
            Some(&self.circles[i])
        }
    }

    pub fn get_obstacle_segment_count(&self) -> usize {
        self.n_segments
    }

    pub fn get_obstacle_segment(&self, i: usize) -> Option<&DtObstacleSegment> {
        if i >= self.n_segments {
            None
        } else {
            Some(&self.segments[i])
        }
    }

    fn prepare(&mut self, pos: &[f32; 3], dvel: &[f32; 3]) {
        // Prepare obstacles
        for i in 0..self.n_circles {
            let cir = &mut self.circles[i];

            // Side
            let pa = [cir.p[0] - pos[0], cir.p[2] - pos[2]];
            let pb = [
                cir.p[0] + cir.vel[0] - pos[0] - dvel[0],
                cir.p[2] + cir.vel[2] - pos[2] - dvel[2],
            ];
            let orig = [0.0, 0.0];
            let (_side, t) = intersect_ray_circle(&orig, &pb, &pa, cir.rad);

            cir.dp = [pa[0] + pb[0] * t, 0.0, pa[1] + pb[1] * t];
            cir.np = normalize_2d(&cir.dp);
        }

        for i in 0..self.n_segments {
            let seg = &mut self.segments[i];
            let p0 = [seg.p[0] - pos[0], seg.p[2] - pos[2]];
            let p1 = [seg.q[0] - pos[0], seg.q[2] - pos[2]];
            seg.touch = distance_pt_seg_sqr_2d(&[0.0, 0.0], &p0, &p1).1 < sqr(0.01);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn process_sample(
        &self,
        vcand: &[f32; 3],
        cs: f32,
        pos: &[f32; 3],
        rad: f32,
        vel: &[f32; 3],
        dvel: &[f32; 3],
        _min_penalty: f32,
        debug: Option<&mut DtObstacleAvoidanceDebugData>,
    ) -> f32 {
        // Velocity penalties
        let vpen = self.params.weight_des_vel * (sqr(vcand[0] - dvel[0]) + sqr(vcand[2] - dvel[2]));
        let vcpen = self.params.weight_cur_vel * (sqr(vcand[0] - vel[0]) + sqr(vcand[2] - vel[2]));

        let mut side = 0.0;
        let mut toi = 0.0;

        let vn = (sqr(vcand[0]) + sqr(vcand[2])).sqrt();
        if vn > 0.01 {
            let ivn = 1.0 / vn;
            let n = [vcand[0] * ivn, 0.0, vcand[2] * ivn];

            // Find minimum time of impact and side
            let mut tmin = self.params.horiz_time;
            let mut tside = 0.0;

            for i in 0..self.n_circles {
                let cir = &self.circles[i];
                let vab = [vcand[0] - cir.vel[0], 0.0, vcand[2] - cir.vel[2]];
                let vn_ab = vab[0] * n[0] + vab[2] * n[2];

                if vn_ab > 0.0 {
                    let sab = sqr(cir.np[0] - n[0]) + sqr(cir.np[2] - n[2]);
                    if sab < sqr(0.01) {
                        tside += (cir.dp[0] * n[0] + cir.dp[2] * n[2]).max(0.0) / vn_ab;
                    }
                }

                let (intersect, t) = intersect_ray_circle(
                    &[0.0, 0.0],
                    &[vab[0], vab[2]],
                    &[cir.dp[0], cir.dp[2]],
                    cir.rad + rad,
                );
                if intersect && t < tmin && t > 0.0 {
                    tmin = t;
                }
            }

            for i in 0..self.n_segments {
                let seg = &self.segments[i];
                if seg.touch {
                    continue;
                }

                let sp = [seg.p[0] - pos[0], seg.p[2] - pos[2]];
                let sq = [seg.q[0] - pos[0], seg.q[2] - pos[2]];
                let (intersect, t) =
                    intersect_ray_segment_circle(&[0.0, 0.0], &[vcand[0], vcand[2]], &sp, &sq, rad);
                if intersect && t < tmin && t > 0.0 {
                    tmin = t;
                }
            }

            side = tside * self.params.weight_side;
            toi = (self.params.horiz_time - tmin) * self.inv_horiz_time * self.params.weight_toi;
        }

        // Normalize penalties
        let ivmax = self.inv_vmax;
        let penalty = vpen * ivmax + vcpen * ivmax + side + toi;

        if let Some(debug_data) = debug {
            debug_data.add_sample(vcand, cs, penalty, vpen * ivmax, vcpen * ivmax, side, toi);
        }

        penalty
    }
}

// Helper functions
fn sqr(x: f32) -> f32 {
    x * x
}

fn normalize_2d(v: &[f32; 3]) -> [f32; 3] {
    let len = (v[0] * v[0] + v[2] * v[2]).sqrt();
    if len > 0.0001 {
        [v[0] / len, 0.0, v[2] / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

fn normalize_array(arr: &mut [f32], n: usize) {
    if n == 0 {
        return;
    }

    let mut max_val: f32 = 0.0;
    for val in arr.iter().take(n) {
        max_val = max_val.max(val.abs());
    }

    if max_val > 0.0001 {
        let inv_max = 1.0 / max_val;
        for val in arr.iter_mut().take(n) {
            *val *= inv_max;
        }
    }
}

fn intersect_ray_circle(
    orig: &[f32; 2],
    dir: &[f32; 2],
    center: &[f32; 2],
    rad: f32,
) -> (bool, f32) {
    let dx = center[0] - orig[0];
    let dz = center[1] - orig[1];
    let a = dir[0] * dir[0] + dir[1] * dir[1];
    let b = 2.0 * (dir[0] * dx + dir[1] * dz);
    let c = dx * dx + dz * dz - rad * rad;
    let discr = b * b - 4.0 * a * c;

    if discr < 0.0 {
        return (false, 0.0);
    }

    let sqrt_discr = discr.sqrt();
    let t1 = (-b - sqrt_discr) / (2.0 * a);
    let t2 = (-b + sqrt_discr) / (2.0 * a);

    if t1 >= 0.0 {
        (true, t1)
    } else if t2 >= 0.0 {
        (true, t2)
    } else {
        (false, 0.0)
    }
}

fn intersect_ray_segment_circle(
    orig: &[f32; 2],
    dir: &[f32; 2],
    p: &[f32; 2],
    q: &[f32; 2],
    rad: f32,
) -> (bool, f32) {
    let d = [q[0] - p[0], q[1] - p[1]];
    let f = [p[0] - orig[0], p[1] - orig[1]];

    let a = d[0] * d[0] + d[1] * d[1];
    let b = 2.0 * (f[0] * d[0] + f[1] * d[1]);
    let c = f[0] * f[0] + f[1] * f[1] - rad * rad;

    let discr = b * b - 4.0 * a * c;
    if discr < 0.0 {
        return (false, 0.0);
    }

    let sqrt_discr = discr.sqrt();
    let t1 = (-b - sqrt_discr) / (2.0 * a);
    let t2 = (-b + sqrt_discr) / (2.0 * a);

    for &t in &[t1, t2] {
        if (0.0..=1.0).contains(&t) {
            let int_point = [p[0] + t * d[0], p[1] + t * d[1]];
            let ray_t = if dir[0].abs() > 0.001 {
                (int_point[0] - orig[0]) / dir[0]
            } else {
                (int_point[1] - orig[1]) / dir[1]
            };

            if ray_t >= 0.0 {
                return (true, ray_t);
            }
        }
    }

    (false, 0.0)
}

fn distance_pt_seg_sqr_2d(pt: &[f32; 2], p: &[f32; 2], q: &[f32; 2]) -> (f32, f32) {
    let pqx = q[0] - p[0];
    let pqz = q[1] - p[1];
    let dx = pt[0] - p[0];
    let dz = pt[1] - p[1];
    let d = pqx * pqx + pqz * pqz;
    let mut t = pqx * dx + pqz * dz;

    if d > 0.0 {
        t /= d;
    }

    t = t.clamp(0.0, 1.0);

    let dx = p[0] + t * pqx - pt[0];
    let dz = p[1] + t * pqz - pt[1];
    (dx * dx + dz * dz, t)
}
