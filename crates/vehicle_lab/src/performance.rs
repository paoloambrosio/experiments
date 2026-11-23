use std::time::{Duration, Instant};

use bevy::{
    prelude::*,
    window::{PresentMode, WindowResolution},
    winit::WinitSettings,
};

use crate::elastic_experiment::TICKS_PER_SECOND;

#[derive(Clone, Copy, Default)]
pub struct WindowOptions {
    pub no_vsync: bool,
    pub benchmark: bool,
}

impl WindowOptions {
    pub fn plugin(self, title: &str) -> WindowPlugin {
        WindowPlugin {
            primary_window: Some(Window {
                title: title.into(),
                resolution: WindowResolution::new(1120, 720),
                present_mode: if self.no_vsync || self.benchmark {
                    PresentMode::AutoNoVsync
                } else {
                    PresentMode::default()
                },
                ..default()
            }),
            ..default()
        }
    }
}

const WARMUP: Duration = Duration::from_secs(2);
const IDLE_SAMPLE: Duration = Duration::from_secs(1);
const TIMEOUT: Duration = Duration::from_secs(120);
pub const BENCHMARK_PRESSURE_PRESET: usize = 2;

#[derive(Clone, Copy)]
enum Phase {
    Warmup,
    Preparation,
    Idle,
    Replay,
}

#[derive(Resource)]
pub struct Benchmark {
    phase: Phase,
    phase_start: Instant,
    last_frame: Instant,
    fixed_start: Instant,
    fixed_clock_before: Duration,
    frame_times: Vec<Duration>,
    fixed_cpu: Duration,
    fixed_sim: Duration,
    exit: Option<AppExit>,
}

impl Benchmark {
    pub fn install(app: &mut App, scenario: &str) {
        println!("Benchmark | {scenario} | 1120x720 logical pixels | no VSync");
        let now = Instant::now();
        app.insert_resource(Self {
            phase: Phase::Warmup,
            phase_start: now,
            last_frame: now,
            fixed_start: now,
            fixed_clock_before: Duration::ZERO,
            frame_times: Vec::new(),
            fixed_cpu: Duration::ZERO,
            fixed_sim: Duration::ZERO,
            exit: None,
        })
        .insert_resource(WinitSettings::continuous())
        .add_systems(Startup, pause_for_warmup)
        .add_systems(
            RunFixedMainLoop,
            (
                begin_fixed.in_set(RunFixedMainLoopSystems::BeforeFixedMainLoop),
                end_fixed.in_set(RunFixedMainLoopSystems::AfterFixedMainLoop),
            ),
        )
        .add_systems(Last, finish);
    }

    /// Records a frame and returns true when the lab should launch the replay.
    pub fn observe(&mut self, ready: bool, complete: bool, error: Option<&str>) -> bool {
        let now = Instant::now();
        if matches!(self.phase, Phase::Warmup) {
            self.last_frame = now;
            return false;
        }

        self.frame_times.push(now - self.last_frame);
        self.last_frame = now;
        let elapsed = now - self.phase_start;
        if let Some(error) = error {
            eprintln!("Benchmark failed: {error}");
            self.exit = Some(AppExit::error());
        } else if elapsed >= TIMEOUT {
            self.report("Timed-out phase", elapsed);
            eprintln!("Benchmark phase timed out after {} s", TIMEOUT.as_secs());
            self.exit = Some(AppExit::error());
        } else {
            match self.phase {
                Phase::Preparation if ready => {
                    self.report("Preparation", elapsed);
                    self.start(Phase::Idle, now);
                }
                Phase::Idle if elapsed >= IDLE_SAMPLE => {
                    self.report("Ready/idle", elapsed);
                    self.start(Phase::Replay, now);
                    return true;
                }
                Phase::Replay if complete => {
                    self.report("Replay", elapsed);
                    self.exit = Some(AppExit::Success);
                }
                _ => {}
            }
        }
        false
    }

    fn start(&mut self, phase: Phase, now: Instant) {
        self.phase = phase;
        self.phase_start = now;
        self.last_frame = now;
        self.frame_times.clear();
        self.fixed_cpu = Duration::ZERO;
        self.fixed_sim = Duration::ZERO;
    }

    fn report(&mut self, label: &str, elapsed: Duration) {
        self.frame_times.sort_unstable();
        let count = self.frame_times.len();
        let p95 = self.frame_times[(count * 95).div_ceil(100) - 1];
        let max = self.frame_times[count - 1];
        let seconds = elapsed.as_secs_f64();
        let sim = self.fixed_sim.as_secs_f64();
        let tick_ms = self.fixed_cpu.as_secs_f64() * 1000.0 / (sim * f64::from(TICKS_PER_SECOND));
        println!(
            "{label:15} {:7.1} FPS | p95 {:6.2} ms | max {:7.2} ms | fixed {:5.2} ms/tick | clock {:5.3}x real time | {count} frames",
            count as f64 / seconds,
            p95.as_secs_f64() * 1000.0,
            max.as_secs_f64() * 1000.0,
            tick_ms,
            sim / seconds,
        );
    }
}

fn pause_for_warmup(mut time: ResMut<Time<Virtual>>, mut benchmark: ResMut<Benchmark>) {
    time.pause();
    benchmark.start(Phase::Warmup, Instant::now());
}

fn begin_fixed(mut benchmark: ResMut<Benchmark>, clock: Res<Time<Fixed>>) {
    benchmark.fixed_clock_before = clock.elapsed();
    benchmark.fixed_start = Instant::now();
}

fn end_fixed(mut benchmark: ResMut<Benchmark>, clock: Res<Time<Fixed>>) {
    let sim = clock.elapsed() - benchmark.fixed_clock_before;
    if !sim.is_zero() {
        let elapsed = benchmark.fixed_start.elapsed();
        benchmark.fixed_cpu += elapsed;
        benchmark.fixed_sim += sim;
    }
}

pub fn finish(
    mut benchmark: ResMut<Benchmark>,
    mut time: ResMut<Time<Virtual>>,
    mut exit: MessageWriter<AppExit>,
) {
    let now = Instant::now();
    if matches!(benchmark.phase, Phase::Warmup) && now - benchmark.phase_start >= WARMUP {
        benchmark.start(Phase::Preparation, now);
        time.unpause();
    }
    if let Some(result) = benchmark.exit.take() {
        exit.write(result);
    }
}
