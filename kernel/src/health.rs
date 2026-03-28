//! Health monitoring types for partition liveness and schedule integrity.

/// Overall health status of the system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    /// All partitions running within deadlines.
    Ok,
    /// One or more non-critical violations detected.
    Degraded,
    /// A critical violation requiring immediate action.
    Critical,
}

/// Action the kernel should take in response to a health violation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthAction {
    /// Log the violation but take no corrective action.
    Log,
    /// Restart the partition identified by its index.
    Restart(u8),
    /// Halt the system.
    Halt,
}

/// Types of health violations the monitor can detect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthViolation {
    /// A major frame exceeded its deadline.
    ScheduleOverrun,
    /// Partition at the given index has not been scheduled for too long.
    PartitionStall(u8),
    /// System tick drift exceeded the configured threshold.
    TickDrift,
    /// A partition missed its heartbeat deadline.
    HeartbeatTimeout,
}

/// Configuration for the system health monitor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemHealthConfig {
    /// Maximum ticks allowed for one major frame before an overrun is raised.
    pub major_frame_deadline_ticks: u32,
    /// Number of major frames a partition may go unscheduled before a stall.
    pub partition_liveness_frames: u16,
    /// Maximum acceptable tick drift in parts-per-million.
    pub tick_drift_ppm: u16,
    /// Action to take when a violation is detected.
    pub on_violation: HealthAction,
}

impl Default for SystemHealthConfig {
    fn default() -> Self {
        Self {
            major_frame_deadline_ticks: 1000,
            partition_liveness_frames: 3,
            tick_drift_ppm: 100,
            on_violation: HealthAction::Log,
        }
    }
}

impl SystemHealthConfig {
    /// Validate configuration values. Returns `Err` with a description on failure.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.major_frame_deadline_ticks == 0 {
            return Err("major_frame_deadline_ticks must be non-zero");
        }
        if self.partition_liveness_frames == 0 {
            return Err("partition_liveness_frames must be non-zero");
        }
        if self.tick_drift_ppm == 0 {
            return Err("tick_drift_ppm must be non-zero");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- HealthStatus construction ---

    #[test]
    fn health_status_variants() {
        let ok = HealthStatus::Ok;
        let degraded = HealthStatus::Degraded;
        let critical = HealthStatus::Critical;
        assert_eq!(ok, HealthStatus::Ok);
        assert_eq!(degraded, HealthStatus::Degraded);
        assert_eq!(critical, HealthStatus::Critical);
        assert_ne!(ok, degraded);
        assert_ne!(degraded, critical);
    }

    #[test]
    fn health_status_copy_and_debug() {
        let s = HealthStatus::Degraded;
        let s2 = s;
        assert_eq!(s, s2);
        assert_eq!(format!("{:?}", HealthStatus::Ok), "Ok");
    }

    // --- HealthAction construction ---

    #[test]
    fn health_action_variants() {
        assert_eq!(HealthAction::Log, HealthAction::Log);
        assert_eq!(HealthAction::Halt, HealthAction::Halt);
        assert_ne!(HealthAction::Halt, HealthAction::Log);
        let a = HealthAction::Restart(3);
        assert_eq!(a, HealthAction::Restart(3));
        assert_ne!(a, HealthAction::Restart(0));
        assert_eq!(format!("{:?}", HealthAction::Restart(1)), "Restart(1)");
    }

    // --- HealthViolation construction ---

    #[test]
    fn violation_variants() {
        assert_eq!(
            HealthViolation::ScheduleOverrun,
            HealthViolation::ScheduleOverrun
        );
        let v = HealthViolation::PartitionStall(2);
        assert_eq!(v, HealthViolation::PartitionStall(2));
        assert_ne!(v, HealthViolation::PartitionStall(5));
        assert_eq!(HealthViolation::TickDrift, HealthViolation::TickDrift);
        assert_ne!(
            HealthViolation::HeartbeatTimeout,
            HealthViolation::TickDrift
        );
        assert_eq!(
            format!("{:?}", HealthViolation::PartitionStall(7)),
            "PartitionStall(7)"
        );
    }

    // --- SystemHealthConfig default ---

    #[test]
    fn config_default_values() {
        let cfg = SystemHealthConfig::default();
        assert_eq!(cfg.major_frame_deadline_ticks, 1000);
        assert_eq!(cfg.partition_liveness_frames, 3);
        assert_eq!(cfg.tick_drift_ppm, 100);
        assert_eq!(cfg.on_violation, HealthAction::Log);
    }

    #[test]
    fn config_default_validates() {
        assert!(SystemHealthConfig::default().validate().is_ok());
    }

    // --- SystemHealthConfig validation ---

    #[test]
    fn config_zero_deadline_rejected() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(
            cfg.validate(),
            Err("major_frame_deadline_ticks must be non-zero")
        );
    }

    #[test]
    fn config_zero_liveness_rejected() {
        let cfg = SystemHealthConfig {
            partition_liveness_frames: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(
            cfg.validate(),
            Err("partition_liveness_frames must be non-zero")
        );
    }

    #[test]
    fn config_zero_drift_rejected() {
        let cfg = SystemHealthConfig {
            tick_drift_ppm: 0,
            ..SystemHealthConfig::default()
        };
        assert_eq!(cfg.validate(), Err("tick_drift_ppm must be non-zero"));
    }

    #[test]
    fn config_custom_valid() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 500,
            partition_liveness_frames: 10,
            tick_drift_ppm: 50,
            on_violation: HealthAction::Halt,
        };
        assert!(cfg.validate().is_ok());
        assert_eq!(cfg.on_violation, HealthAction::Halt);
    }

    #[test]
    fn config_copy_debug_restart() {
        let cfg = SystemHealthConfig::default();
        let cfg2 = cfg;
        assert_eq!(cfg, cfg2);
        let s = format!("{:?}", cfg);
        assert!(s.contains("SystemHealthConfig"));

        let cfg3 = SystemHealthConfig {
            major_frame_deadline_ticks: 200,
            partition_liveness_frames: 1,
            tick_drift_ppm: 500,
            on_violation: HealthAction::Restart(2),
        };
        assert!(cfg3.validate().is_ok());
        assert_eq!(cfg3.on_violation, HealthAction::Restart(2));
    }

    #[test]
    fn config_all_zero_reports_deadline_first() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: 0,
            partition_liveness_frames: 0,
            tick_drift_ppm: 0,
            on_violation: HealthAction::Log,
        };
        assert_eq!(
            cfg.validate(),
            Err("major_frame_deadline_ticks must be non-zero")
        );
    }

    #[test]
    fn config_boundary_values() {
        let cfg = SystemHealthConfig {
            major_frame_deadline_ticks: u32::MAX,
            partition_liveness_frames: u16::MAX,
            tick_drift_ppm: u16::MAX,
            on_violation: HealthAction::Log,
        };
        assert!(cfg.validate().is_ok());
    }
}
