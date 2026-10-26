#![no_std]
#![no_main]

use defmt_rtt as _;

mod panic_capture;

pub mod pac {
    pub use embassy_stm32::Peripherals;
    pub use embassy_stm32::pac::Interrupt as interrupt;
    pub use embassy_stm32::pac::*;
}

mod ankyra;
mod app_types;
mod board_config;
mod board_init;
mod commissioning_backend;
mod firmware_ctx;
mod heartbeat;
mod helpers;
mod interrupts;
mod pulse_train;
mod state;
mod step_dispatch;
mod stepper_pins;
mod tmc_control;
mod tmc_diagnostics;
mod tmc_dispatch;
mod tmc_mux;
mod transport;
mod trigger_domain;
mod usb_task;
#[cfg(feature = "trace")]
mod usb_trace;

#[rtic::app(device = crate::pac, peripherals = true, dispatchers = [UART4, UART5, USART3])]
mod app {

    use crate::ankyra::constants::CLOCK_FREQ;
    use embassy_stm32::gpio::{Input, Level, Output, Speed};
    use embassy_stm32::spi::Spi;
    use embassy_stm32::wdg::IndependentWatchdog;
    use embassy_stm32::{bind_interrupts, peripherals};
    use foci_klipper::state::State;
    use rtic_monotonics::fugit::ExtU64;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;

    use crate::board_config::TMC_CHANNEL_COUNT;
    use crate::helpers::lifecycle::*;
    use crate::helpers::stepper::*;
    use crate::helpers::trsync::*;
    use crate::state::tmc::*;
    use crate::step_dispatch::{StepDispatchOutcome, run_step_dispatch};
    use crate::tmc_control::board::{
        TmcEnablePins, apply_live_maintenance_arm, channel_needs_safe_state,
        commissioning_request_blocked, enable_pin, ouroboros_backend, refresh_velocity_scale,
        stepper_state, tmc_driver, tmc_state,
    };
    use crate::trigger_domain::{TRIGGER_DOMAIN, TriggerDomain, with_trigger_domain};
    use foci_firmware::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlots;
    use foci_firmware::context::stepper::{
        StepperArmCheckBoard, StepperStopCheckBoard, StepperTimerStateBoard,
    };
    use foci_firmware::context::stepper_reconcile::drain_stepper_reconciliation;
    use foci_firmware::state::lifecycle::*;
    use foci_firmware::state::stepper::{NO_STEPPER_CHANNEL, STEPPER_DEADLINE_MISS_CHANNEL};
    use foci_firmware::state::tmc::{TMC_DISABLE_SIGNAL, TMC_QUEUE_FLUSH};
    use foci_firmware::state::trsync::*;
    use foci_firmware::tmc_control::TmcControlRequestTail;
    use foci_firmware::tmc_control::request::TmcPreRequestDisposition;
    use foci_firmware::tmc_control::{BoardAction, BoardActions, StepperAction};

    bind_interrupts!(struct Irqs {
        DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
    });

    stm32_tim2_monotonic!(Mono, 1_000_000);

    static USB_PUMP_COUNTERS: foci_firmware::usb_pump::UsbPumpCounters =
        foci_firmware::usb_pump::UsbPumpCounters::new();
    static USB_PUMP_EMITTER: crate::usb_task::Emitter = crate::usb_task::Emitter::new();

    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_ITERS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_MAX_GAP_US: core::sync::atomic::AtomicU32 =
        core::sync::atomic::AtomicU32::new(0);
    #[cfg(feature = "usb-p1-probe")]
    static P1_PROBE_LAST_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

    use foci_firmware::diagnostic::StepperEventReason;
    use foci_firmware::tmc::{TmcRequest, TmcRequestKind};

    use crate::board_config::TMC_CONFIG;
    use crate::helpers::stepper_diag::{UNKNOWN_QUEUE_LEN, record_stepper_event};

    fn lower_all_enable_pins(enable_pins: &mut TmcEnablePins) {
        for pin in enable_pins {
            pin.set_low();
        }
    }

    fn apply_runtime_stepper_actions(channel: u8, actions: BoardActions) {
        for action in actions {
            match action {
                BoardAction::Stepper(StepperAction::StopNow) => {
                    stepper_stop_from_p1(channel);
                }
                BoardAction::Stepper(StepperAction::None)
                | BoardAction::Stepper(StepperAction::ArmCheck { .. })
                | BoardAction::Stepper(StepperAction::ApplyPostAction(_))
                | BoardAction::Shutdown(_)
                | BoardAction::SignalEmergencyDisable => {}
            }
        }
    }

    fn quiesce_all_channels(
        drivers: &mut [foci_firmware::tmc_control::ChannelDriver; 2],
        enable_pins: &mut TmcEnablePins,
    ) {
        let actions0 = drivers[0].runtime.prepare_shutdown_quiesce();
        apply_runtime_stepper_actions(0, actions0);
        tmc_state(0)
            .commission_in_flight
            .store(false, core::sync::atomic::Ordering::Relaxed);
        let actions1 = drivers[1].runtime.prepare_shutdown_quiesce();
        apply_runtime_stepper_actions(1, actions1);
        tmc_state(1)
            .commission_in_flight
            .store(false, core::sync::atomic::Ordering::Relaxed);
        lower_all_enable_pins(enable_pins);
    }

    fn emit_invalid_channel_diagnostic(channel: u8) {
        ::ankyra::klipper_reply_from!(
            &mut crate::_ankyra_config::Sender,
            foci_firmware::ankyra::replies::FociDiagnostic,
            fault: u8 = foci_firmware::diagnostic::FaultClass::InvalidChannel as u8,
            operation: u8 = foci_firmware::diagnostic::operation::TMC_CONTROL,
            oid: u8 = 255,
            channel: u8 = channel,
            data0: u32 = 0,
            data1: u32 = 0,
        );
    }

    #[shared]
    struct Shared {
        app_state: State,
    }

    /// Persistent Integral candidate slots carried across `tmc_control` task calls.
    ///
    /// Retains the Integral authority a `breakaway_seeded` campaign hands off to
    /// `integral_resume`, so the production build runs the full autotune
    /// hand-off exactly as the trace build does.
    type IntegralCandidatesTaskLocal = IntegralRetentionSlots<{ TMC_CHANNEL_COUNT as usize }>;

    #[local]
    struct Local {
        tmc_mux: crate::tmc_mux::TmcSpiMux,
        tmc_enable: TmcEnablePins,
        tmc_rx: Receiver<'static, TmcControlEvent, TMC_CHANNEL_SIZE>,
        tmc_status0: Input<'static>,
        wdg: IndependentWatchdog<'static, peripherals::IWDG1>,
        integral_candidates: IntegralCandidatesTaskLocal,
        adc: embassy_stm32::adc::Adc<'static, crate::board_config::ExternalAdcInstance>,
        adc_internal:
            embassy_stm32::adc::Adc<'static, crate::board_config::InternalTempAdcInstance>,
        temp_channel: embassy_stm32::adc::Vbat,
        th0_pin: embassy_stm32::Peri<'static, peripherals::PC4>,
        th1_pin: embassy_stm32::Peri<'static, peripherals::PC5>,
        mcu_temp_calibration: foci_firmware::mcu_temp::FactoryCalibration,
    }

    #[init(local = [
        tmc_channel: rtic_sync::channel::Channel<TmcControlEvent, TMC_CHANNEL_SIZE> =
            rtic_sync::channel::Channel::new(),
        uid_str: [u8; 24] = [0; 24],
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        crate::board_init::paint_stack();
        foci_firmware::diagnostic::recover_last_boot_breakaway_breadcrumb();
        defmt::info!("FOCI Ouroboros starting");

        if let Some(record) = crate::panic_capture::recover() {
            defmt::error!(
                "recovered last-boot fault: kind={} line={} pc={:#010x}",
                record.kind,
                record.line,
                record.pc
            );
            foci_firmware::diagnostic::set_last_panic(record);
        }

        // SAFETY: init runs exactly once, before any other task starts.
        unsafe {
            crate::state::stepper::init_step_queues();
        }

        let p = embassy_stm32::init(crate::board_init::rcc_config());
        defmt::info!("SYSCLK: 520 MHz, USB: 48 MHz");
        let mut core = ctx.core;
        crate::board_init::enable_instruction_cache(&mut core);
        crate::board_init::verify_dma_interrupt_priority();
        crate::board_init::disable_debug_clock_during_sleep();

        {
            let rsr = embassy_stm32::pac::RCC.rsr().read();
            defmt::info!(
                "reset cause: LPWRRSTF={} WWDG1RSTF={} IWDG1RSTF={} SFTRSTF={} PORRSTF={} PINRSTF={} BORRSTF={}",
                rsr.lpwrrstf(),
                rsr.wwdg1rstf(),
                rsr.iwdg1rstf(),
                rsr.sftrstf(),
                rsr.porrstf(),
                rsr.pinrstf(),
                rsr.borrstf(),
            );
            embassy_stm32::pac::RCC.rsr().modify(|w| w.set_rmvf(true));
        }

        crate::board_init::init_tim5_clock();
        #[cfg(feature = "trace")]
        crate::usb_trace::note_trace_boot();
        crate::board_init::init_stepdir_gpio();
        defmt::info!("TIM5 enabled for Klipper clock @ 260 MHz, STEP/DIR GPIO configured");

        Mono::start(260_000_000);

        let tmc_enable = [
            Output::new(p.PD6, Level::Low, Speed::Low),
            Output::new(p.PD1, Level::Low, Speed::Low),
        ];

        let tmc_status0 = crate::board_init::init_tmc_status_pins(p.PE2, p.PA2);
        crate::board_init::init_tim15_status_capture();

        let mut wdg = IndependentWatchdog::new(p.IWDG1, 10_000_000);
        wdg.unleash();
        defmt::info!("IWDG started: 10s timeout");

        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleHigh,
            phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
        };
        spi_config.frequency = embassy_stm32::time::Hertz(8_000_000);

        let spi2 = Spi::new(
            p.SPI2, p.PB13, p.PB15, p.PB14, p.DMA1_CH0, p.DMA1_CH1, Irqs, spi_config,
        );

        let spi_cs0 = Output::new(p.PD0, Level::High, Speed::VeryHigh);
        let spi_cs1 = Output::new(p.PD2, Level::High, Speed::VeryHigh);
        let tmc_mux = crate::tmc_mux::TmcSpiMux::new(spi2, spi_cs0, spi_cs1);

        let (tmc_tx, tmc_rx) = ctx.local.tmc_channel.split();
        critical_section::with(|cs| {
            TMC_CONTROL_SENDER.borrow(cs).replace(Some(tmc_tx));
        });

        defmt::info!("SPI2 configured: 8 MHz, Mode 3, DMA");

        crate::board_init::init_usb_pins(p.PA11, p.PA12);
        crate::board_init::enable_usb_clock();

        let uid = crate::board_init::read_stm32_uid(ctx.local.uid_str);
        #[cfg(feature = "trace")]
        crate::usb_trace::set_trace_serial(uid);

        // SAFETY: Called exactly once, before any other klipper-usb function.
        // base_addr is the USB_OTG_HS peripheral base on STM32H723.
        unsafe {
            foci_usb_stm32::usb_init(&foci_usb_stm32::UsbConfig {
                base_addr: 0x4004_0000,
                irq_num: 77,
                trdt: 6,
                vbus_detection: true,
                double_buffer_tx: true,
                vid: 0x1D50,
                pid: 0x614E,
                manufacturer: "foci",
                product: crate::board_config::USB_PRODUCT,
                serial: uid,
            });
        }

        crate::board_init::enable_dwt_cyccnt();

        // SAFETY: init runs once before any task starts. No concurrent access.
        unsafe {
            core::ptr::addr_of_mut!(TRIGGER_DOMAIN)
                .cast::<TriggerDomain>()
                .write(TriggerDomain::new());
        }

        // TH0 (PC4) / TH1 (PC5) share ADC1; the internal temperature sensor
        // is only wired to ADC3 on this chip (see board_config.rs's
        // ExternalAdcInstance/InternalTempAdcInstance docs), so two separate
        // Adc instances are constructed.
        // embassy-stm32's Adc::new() leaves ADC_CFGR.RES at its hardware
        // reset default, which real Klipper's stm32h7_adc.c explicitly
        // overrides to 12-bit for ADC1/ADC2 (the H723's v4 IP resets to a
        // wider mode); ADC_MAX and every downstream Steinhart-Hart/
        // calibration computation in this feature assume 12-bit counts.
        let adc = embassy_stm32::adc::Adc::new_with_config(
            p.ADC1,
            embassy_stm32::adc::AdcConfig {
                resolution: Some(embassy_stm32::adc::Resolution::BITS12),
                ..Default::default()
            },
        );
        // ADC3's RES field is 2 bits wide at CFGR[4:3] (reset value 00 =
        // 12-bit), unlike ADC1/ADC2's 3-bit field at CFGR[4:2]. Applying
        // Resolution::BITS12's bit pattern (built for the 3-bit field) to
        // ADC3 lands as RES=01 (10-bit), so retain ADC3's reset resolution.
        let adc_internal = embassy_stm32::adc::Adc::new(p.ADC3);
        // embassy-stm32 0.6.0 maps `Temperature` to ADC3 channel 18 for
        // every stm32h7 part, but the H723's internal temperature sensor is
        // wired to channel 17 (18 is unused on this variant). `Vbat`'s
        // channel constant happens to be 17 in this crate version, so its
        // marker reaches the right channel; `enable_temperature()` still
        // asserts CCR.VSENSEEN to route the real VSENSE signal onto it.
        adc_internal.enable_temperature();
        let temp_channel = embassy_stm32::adc::Vbat;
        let th0_pin = p.PC4;
        let th1_pin = p.PC5;
        let mcu_temp_calibration = crate::board_config::read_mcu_temp_calibration();

        stepper_reconcile::spawn().expect("stepper reconciliation worker must spawn exactly once");
        trigger_drain_task::spawn().expect("trigger drain worker must spawn exactly once");
        tmc_control::spawn().expect("tmc_control task must spawn exactly once");
        usb_task::spawn().expect("usb_task must spawn exactly once");
        stats::spawn().expect("stats task must spawn exactly once");
        watchdog::spawn().expect("watchdog task must spawn exactly once");
        analog_sampling::spawn().expect("analog_sampling task must spawn exactly once");

        defmt::info!("FOCI Ouroboros initialized");

        (
            Shared {
                app_state: State::new(),
            },
            Local {
                tmc_mux,
                tmc_enable,
                tmc_rx,
                tmc_status0,
                wdg,
                integral_candidates: IntegralCandidatesTaskLocal::new(),
                adc,
                adc_internal,
                temp_channel,
                th0_pin,
                th1_pin,
                mcu_temp_calibration,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Mask interrupts so wfi wakes on a pending IRQ but defers the
            // handler until we re-enable: the bracketed interval is pure sleep,
            // not sleep plus the work burst that runs before idle (the
            // lowest-priority context) can resume. Recording the lap inside the
            // masked region keeps measure-and-record atomic against the P1
            // stats reader, so each stats window accounts its own sleep even
            // when a single sleep spans most of the window. Mirrors Klipper
            // sched.c's irq_disable / irq_wait / irq_enable around its sleep.
            cortex_m::interrupt::free(|_| {
                let sleep_start = read_tim5();
                rtic::export::wfi();
                let slept = read_tim5().wrapping_sub(sleep_start);
                foci_firmware::cpu_load::record_idle_lap(slept);
            });
        }
    }

    use crate::ankyra::replies::{emit_phase_reply, emit_terminal_reply};

    /// TMC4671 control task. Runs at priority 1 (preempted by protocol tasks).
    ///
    /// On startup, reads the TMC4671 chip identification register to verify
    /// SPI communication. Then processes register read/write requests from
    /// the protocol handler via the TMC request channel.
    ///
    /// The 1ms loop also advances the emulated stepper, converting queued
    /// step commands into position target updates. This must run on a timer
    /// rather than only on USB receive — Klipper sends queue_step commands
    /// ahead of time and expects the MCU to process them at the scheduled
    /// clock values, even when no USB traffic is arriving.
    #[task(
        priority = 1,
        shared = [app_state],
        local = [
            tmc_mux,
            tmc_enable,
            tmc_status0,
            tmc_rx,
            integral_candidates,
            drivers: [foci_firmware::tmc_control::ChannelDriver; 2] = [
                foci_firmware::tmc_control::ChannelDriver::new(0),
                foci_firmware::tmc_control::ChannelDriver::new(1),
            ]
        ]
    )]
    async fn tmc_control(mut ctx: tmc_control::Context) {
        use crate::tmc_control::board::{
            ChannelDevice, STATUS_CRITICAL_MASK, device_or_invalid, drive_ctx,
            enable_status_capture_ch1, enable_status_exti_ch0, park_shutdown, status_levels,
        };
        use core::sync::atomic::Ordering::{AcqRel, Acquire, Relaxed, Release};
        use embassy_futures::select::{Either, select};
        use foci_firmware::tmc_control::scheduler::{
            ChannelMode, ChannelScheduler, MaintenanceArm, UnitClass,
        };
        use foci_firmware::tmc_control::{
            AuditResult, CommissionNext, CommissionWaitRequestDisposition, UnitOutcome,
            bootstrap_tmc_safety,
        };

        use crate::tmc_control::wake::{
            HomingPreparationProgress, HomingReceive, HomingWait, HomingWakeEvent, HomingWakePolicy,
        };

        let mux = ctx.local.tmc_mux;
        let enable_pins = ctx.local.tmc_enable;
        let status_pin0 = ctx.local.tmc_status0;
        let rx = ctx.local.tmc_rx;
        let integral_candidates = ctx.local.integral_candidates;
        let drivers = ctx.local.drivers;

        let tmc_control_config = foci_firmware::tmc_control::TmcControlConfig::new(
            CLOCK_FREQ,
            TMC_CONFIG.pwm_freq_hz(),
            TMC_CONFIG.current_scale_ma_per_lsb,
            TMC_CONFIG,
        );
        let now_us = || Mono::now().duration_since_epoch().to_micros();
        let mut scheduler = ChannelScheduler::<2>::new(now_us(), [1_000, 1_000]);
        let mut homing_wake = HomingWakePolicy::new();
        let mut group_all_ready = true;
        let mut group_retry = false;
        let mut shutdown_handled = false;

        macro_rules! shutdown {
            ($msg:expr) => {
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        ::ankyra::klipper_shutdown_from!(
                            &mut crate::_ankyra_config::Sender,
                            $msg,
                            read_tim5(),
                        );
                    }
                });
            };
        }

        macro_rules! shutdown_reason {
            ($reason:expr) => {
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            $reason,
                            read_tim5(),
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            };
        }

        macro_rules! per_channel {
            (|$ch:ident, $idx:ident| $body:block) => {{
                {
                    let $ch: u8 = 0;
                    let $idx: usize = 0;
                    $body
                }
                {
                    let $ch: u8 = 1;
                    let $idx: usize = 1;
                    $body
                }
            }};
        }

        per_channel!(|ch, _idx| {
            match device_or_invalid(mux, ch) {
                ChannelDevice::Device(device) => {
                    let mut tmc = tmc_driver(device);
                    let mut backend = ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                    if let Err(failure) =
                        bootstrap_tmc_safety(&mut backend, ch, STATUS_CRITICAL_MASK).await
                    {
                        shutdown_reason!(failure.shutdown_reason(ch));
                    }
                }
                ChannelDevice::Invalid => {
                    shutdown!("Invalid TMC channel");
                }
            }
        });

        'control_loop: loop {
            #[cfg(feature = "usb-p1-probe")]
            {
                let now = Mono::now().ticks() as u32;
                let last = P1_PROBE_LAST_US.swap(now, Relaxed);
                if last != 0 {
                    P1_PROBE_MAX_GAP_US.fetch_max(now.wrapping_sub(last), Relaxed);
                }
                P1_PROBE_ITERS.fetch_add(1, Relaxed);
            }

            let levels = status_levels(status_pin0);
            tmc_state(0).status_pin_asserted.store(levels[0], Relaxed);
            tmc_state(1).status_pin_asserted.store(levels[1], Relaxed);

            per_channel!(|ch, idx| {
                if drivers[idx].runtime.restore_cancel_pending() {
                    match device_or_invalid(mux, ch) {
                        ChannelDevice::Device(device) => {
                            let mut tmc = tmc_driver(device);
                            let mut backend =
                                ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                            let _ = drivers[idx]
                                .runtime
                                .drive_restore_cancel(
                                    &mut backend,
                                    tmc_state(ch),
                                    stepper_state(ch),
                                    foci_firmware::autotune::CommissionError::Shutdown,
                                )
                                .await;
                        }
                        ChannelDevice::Invalid => {
                            shutdown!("Invalid TMC channel");
                            continue 'control_loop;
                        }
                    }
                }
            });

            let emergency_pending = TMC_DISABLE_SIGNAL.signaled()
                || FAULT_ACTIVE.load(Relaxed)
                || levels[0]
                || levels[1];
            per_channel!(|ch, idx| {
                if !emergency_pending && tmc_state(ch).cancel_requested.load(Relaxed) {
                    match device_or_invalid(mux, ch) {
                        ChannelDevice::Device(device) => {
                            let mut tmc = tmc_driver(device);
                            let mut backend =
                                ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                            if let Some(
                                foci_firmware::commission_dispatch::CommissionOutcome::Terminal {
                                    reply,
                                    actions,
                                },
                            ) = drivers[idx]
                                .runtime
                                .drive_operator_cancel(
                                    &mut backend,
                                    tmc_state(ch),
                                    stepper_state(ch),
                                    TMC_CONFIG.pwm_maxcnt,
                                )
                                .await
                            {
                                #[cfg(feature = "trace")]
                                {
                                    drivers[idx].commission_trace_phase = None;
                                }
                                {
                                    let mut state = drivers[idx]
                                        .runtime
                                        .dispatch_state(tmc_state(ch), stepper_state(ch));
                                    apply_stepper_post_action(actions.stepper, &mut state, ch);
                                }
                                foci_firmware::diagnostic::mark_p1_reply_construction_start();
                                emit_terminal_reply(ch, reply);
                                foci_firmware::diagnostic::mark_p1_reply_construction_end();
                            }
                        }
                        ChannelDevice::Invalid => {
                            shutdown!("Invalid TMC channel");
                            continue 'control_loop;
                        }
                    }
                }
            });

            if SHUTDOWN_ACTIVE.load(Relaxed) {
                if !shutdown_handled {
                    #[cfg(feature = "trace")]
                    for d in drivers.iter_mut() {
                        d.commission_trace_phase = None;
                    }
                    let mut complete = true;
                    per_channel!(|ch, idx| {
                        match device_or_invalid(mux, ch) {
                            ChannelDevice::Device(device) => {
                                let mut tmc = tmc_driver(device);
                                let mut backend =
                                    ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                                foci_firmware::tmc_control::lifecycle::apply_shutdown_safe_state(
                                    &mut drivers[idx].runtime,
                                    &mut backend,
                                    tmc_state(ch),
                                    stepper_state(ch),
                                )
                                .await;
                                tmc_state(ch).commission_in_flight.store(false, Relaxed);
                            }
                            ChannelDevice::Invalid => complete = false,
                        }
                    });
                    quiesce_all_channels(drivers, enable_pins);
                    shutdown_handled = complete;
                    integral_candidates.invalidate_all();
                }
                Mono::delay(1.millis()).await;
                continue;
            }
            shutdown_handled = false;

            match crate::tmc_control::containment::classify_lease_check(
                SHUTDOWN_ACTIVE.load(Relaxed),
                crate::tmc_control::containment::lease_expired(read_tim5()),
            ) {
                crate::tmc_control::containment::LeaseCheckDisposition::EnterClockSyncShutdown => {
                    shutdown_reason!(foci_firmware::shutdown::ShutdownReason::HostClockSyncExpired);
                    continue;
                }
                crate::tmc_control::containment::LeaseCheckDisposition::Proceed => {}
            }

            let queue_flush = critical_section::with(|_| {
                if TMC_QUEUE_FLUSH.swap(false, AcqRel) {
                    HOMING_START_PENDING.store(false, Release);
                    true
                } else {
                    false
                }
            });
            if queue_flush {
                homing_wake.reset_for_queue_flush();
                integral_candidates.invalidate_all();
                while rx.try_recv().is_ok() {}
                let needs_disarm = [
                    channel_needs_safe_state(0, &drivers[0]),
                    channel_needs_safe_state(1, &drivers[1]),
                ];
                quiesce_all_channels(drivers, enable_pins);
                #[cfg(feature = "trace")]
                for d in drivers.iter_mut() {
                    d.commission_trace_phase = None;
                }
                per_channel!(|ch, idx| {
                    if needs_disarm[idx] {
                        match device_or_invalid(mux, ch) {
                            ChannelDevice::Device(device) => {
                                let mut tmc = tmc_driver(device);
                                let mut backend =
                                    ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                                let mut state = drivers[idx]
                                    .runtime
                                    .dispatch_state(tmc_state(ch), stepper_state(ch));
                                foci_firmware::tmc_control::lifecycle::disarm_motor(
                                    &mut backend,
                                    &mut state,
                                )
                                .await;
                            }
                            ChannelDevice::Invalid => {
                                shutdown!("Invalid TMC channel");
                            }
                        }
                    }
                    drivers[idx].runtime.reset_runtime_state();
                    foci_firmware::tmc_control::TmcControlRuntime::reset_channel_state(
                        tmc_state(ch),
                        stepper_state(ch),
                    );
                    tmc_state(ch).commission_in_flight.store(false, Relaxed);
                });
                defmt::info!("TMC request queue flushed after config_reset");
            }

            if TMC_DISABLE_SIGNAL.try_take().is_some()
                || FAULT_ACTIVE.load(Relaxed)
                || levels[0]
                || levels[1]
            {
                let deadline_channel =
                    STEPPER_DEADLINE_MISS_CHANNEL.swap(NO_STEPPER_CHANNEL, Relaxed);
                quiesce_all_channels(drivers, enable_pins);
                #[cfg(feature = "trace")]
                for d in drivers.iter_mut() {
                    d.commission_trace_phase = None;
                }
                let mut fault = foci_firmware::tmc_control::EmergencyDisableFault::Generic;
                per_channel!(|ch, _idx| {
                    let status_flags = match device_or_invalid(mux, ch) {
                        ChannelDevice::Device(device) => {
                            let mut tmc = tmc_driver(device);
                            tmc.read_register_retried(foci_tmc4671::registers::STATUS_FLAGS)
                                .await
                                .map_err(|_| ())
                        }
                        ChannelDevice::Invalid => Err(()),
                    };
                    fault = foci_firmware::tmc_control::combine_emergency_disable_fault(
                        fault,
                        foci_firmware::tmc_control::classify_emergency_disable_fault(status_flags),
                    );
                    record_stepper_event(
                        StepperEventReason::TmcDisableSignal,
                        ch,
                        read_tim5(),
                        UNKNOWN_QUEUE_LEN,
                        status_flags.unwrap_or(0),
                        0,
                    );
                    #[cfg(feature = "trace")]
                    crate::usb_trace::request_event(
                        foci_firmware::trace::events::EVENT_LEVEL_ERROR,
                        foci_firmware::trace::events::SUBSYSTEM_SAFETY_SHUTDOWN,
                        crate::usb_trace::EVENT_STATUS_INTERRUPT,
                        [i32::from(ch), status_flags.unwrap_or(0) as i32, 0, 0],
                        2,
                    );
                    match status_flags {
                        Ok(flags) => defmt::error!(
                            "TMC4671 channel {} emergency disable: STATUS={:#010x}",
                            ch,
                            flags
                        ),
                        Err(_) => defmt::error!(
                            "TMC4671 channel {} emergency disable: STATUS read failed",
                            ch
                        ),
                    }
                });

                let levels = status_levels(status_pin0);
                tmc_state(0).status_pin_asserted.store(levels[0], Relaxed);
                tmc_state(1).status_pin_asserted.store(levels[1], Relaxed);
                let fault_still_reported = levels[0] || levels[1] || TMC_DISABLE_SIGNAL.signaled();
                if fault_still_reported {
                    defmt::error!("TMC4671 fault still reported after containment; latch kept");
                } else {
                    FAULT_ACTIVE.store(false, Relaxed);
                }

                match fault {
                    foci_firmware::tmc_control::EmergencyDisableFault::AdcCurrentClipped => {
                        shutdown!("TMC4671: ADC current clipped (overcurrent)");
                    }
                    foci_firmware::tmc_control::EmergencyDisableFault::StatusReadFailed => {
                        shutdown!("TMC4671: STATUS_FLAGS read failed");
                    }
                    foci_firmware::tmc_control::EmergencyDisableFault::Generic => {
                        if deadline_channel != NO_STEPPER_CHANNEL {
                            shutdown_reason!(
                                foci_firmware::shutdown::ShutdownReason::StepperTooFarInPast
                            );
                        } else {
                            shutdown!("TMC4671 fault");
                        }
                    }
                }
                continue;
            }

            per_channel!(|ch, idx| {
                if matches!(
                    drivers[idx].mode(),
                    ChannelMode::Commissioning | ChannelMode::Restore
                ) {
                    let exit = crate::tmc_control::containment::classify_commission_wait_exit(
                        SHUTDOWN_ACTIVE.load(Relaxed),
                        TMC_QUEUE_FLUSH.load(Relaxed),
                        TMC_DISABLE_SIGNAL.signaled(),
                        FAULT_ACTIVE.load(Relaxed),
                        crate::tmc_control::containment::lease_expired(read_tim5()),
                        tmc_state(ch).cancel_requested.load(Relaxed),
                    );
                    if let Some(exit) = exit {
                        if !matches!(
                            exit,
                            crate::tmc_control::containment::CommissionWaitExit::Cancelled
                        ) {
                            drivers[idx].runtime.request_restore_cancel();
                        }
                        if matches!(
                            exit,
                            crate::tmc_control::containment::CommissionWaitExit::EmergencyDisable
                        ) {
                            lower_all_enable_pins(enable_pins);
                        }
                        continue 'control_loop;
                    }
                }
            });

            scheduler.set_mode(0, drivers[0].mode(), now_us());
            scheduler.set_mode(1, drivers[1].mode(), now_us());

            let now = now_us();
            apply_live_maintenance_arm(&mut scheduler, drivers, now);
            let wait = homing_wake.next_wait(
                HOMING_START_PENDING.load(Acquire),
                scheduler.earliest_due_us().is_some_and(|due| due <= now),
            );
            let received_event = match wait {
                HomingWait::ProcessRetained | HomingWait::RunScheduler => None,
                HomingWait::MaintenanceOnly => {
                    scheduler.set_maintenance_arm(MaintenanceArm::Armed, now_us());
                    Mono::delay_until(fugit::Instant::<u64, 1, 1_000_000>::from_ticks(
                        scheduler.maintenance_due_us(),
                    ))
                    .await;
                    None
                }
                HomingWait::ReceiverOrDeadline => {
                    let control_sleep =
                        Mono::delay_until(fugit::Instant::<u64, 1, 1_000_000>::from_ticks(
                            scheduler
                                .earliest_due_us()
                                .unwrap_or_else(|| now_us() + 1_000),
                        ));
                    match select(rx.recv(), control_sleep).await {
                        Either::First(Ok(event)) => Some(event),
                        Either::First(Err(_)) | Either::Second(_) => None,
                    }
                }
            };
            apply_live_maintenance_arm(&mut scheduler, drivers, now_us());

            let request = if matches!(wait, HomingWait::ProcessRetained) {
                homing_wake.take_retained_request()
            } else if let Some(event) = received_event {
                let event = match event {
                    TmcControlEvent::Request(req) => HomingWakeEvent::Request(req),
                    TmcControlEvent::HomingStart => HomingWakeEvent::HomingStart,
                };
                match homing_wake.on_event(event, HOMING_START_PENDING.load(Acquire)) {
                    HomingReceive::Process(req) => Some(req),
                    HomingReceive::Retained | HomingReceive::Doorbell => None,
                    HomingReceive::RetainedSlotOccupied(_req) => {
                        shutdown_reason!(
                            foci_firmware::shutdown::ShutdownReason::TmcRequestValidationFailed
                        );
                        continue;
                    }
                }
            } else {
                None
            };

            if let Some(req) = request {
                if req.channel >= crate::board_config::TMC_CHANNEL_COUNT {
                    emit_invalid_channel_diagnostic(req.channel);
                    shutdown!("Invalid TMC channel");
                    continue;
                }
                let channel = req.channel;
                let idx = channel as usize;
                let commissioning = matches!(
                    drivers[idx].mode(),
                    ChannelMode::Commissioning | ChannelMode::Restore
                );
                let disposition = if commissioning {
                    crate::tmc_control::effects::commission_wait_request_disposition(&req)
                } else {
                    CommissionWaitRequestDisposition::Cancel
                };
                match disposition {
                    CommissionWaitRequestDisposition::Cancel => {
                        if commissioning {
                            let route =
                                crate::tmc_control::effects::classify_commission_wait_cancel(
                                    channel,
                                    req.channel,
                                    crate::board_config::TMC_CHANNEL_COUNT,
                                );
                            if matches!(
                                route,
                                crate::tmc_control::effects::CommissionWaitCancelRoute::Invalid
                            ) {
                                emit_invalid_channel_diagnostic(req.channel);
                                shutdown!("Invalid TMC channel");
                                drivers[idx].runtime.request_restore_cancel();
                                continue 'control_loop;
                            }
                            enable_pin(enable_pins, channel).set_low();
                            tmc_state(channel)
                                .commission_in_flight
                                .store(false, Relaxed);
                        }
                        let commissioning_blocked = match &req.kind {
                            TmcRequestKind::SetEnable(true) => {
                                !drivers[idx].runtime.is_calibrated()
                                    && commissioning_request_blocked(drivers, channel)
                            }
                            TmcRequestKind::Calibrate { .. }
                            | TmcRequestKind::Commission { .. }
                            | TmcRequestKind::Tune { .. }
                            | TmcRequestKind::SelfTest { .. } => {
                                commissioning_request_blocked(drivers, channel)
                            }
                            _ => false,
                        };
                        let outcome = match device_or_invalid(mux, channel) {
                            ChannelDevice::Device(device) => {
                                let mut tmc = tmc_driver(device);
                                crate::tmc_control::request::handle_tmc_control_request(
                                    req,
                                    TmcPreRequestDisposition::ContinueRequest,
                                    crate::tmc_control::request::TmcControlRequestContext {
                                        channel,
                                        commissioning_blocked,
                                        drivers: &mut *drivers,
                                        tmc: &mut tmc,
                                        enable_pins,
                                        integral_candidates: &mut *integral_candidates,
                                    },
                                )
                                .await
                            }
                            ChannelDevice::Invalid => {
                                shutdown!("Invalid TMC channel");
                                continue;
                            }
                        };
                        if let Some(reason) = outcome.shutdown {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    crate::tmc_control::effects::emit_shutdown(reason);
                                }
                            });
                        }
                        if outcome.should_refresh_velocity_scale {
                            refresh_velocity_scale(channel, &mut drivers[idx].runtime);
                        }
                        if !commissioning
                            && matches!(drivers[idx].mode(), ChannelMode::Commissioning)
                        {
                            scheduler.set_commission_due(channel, Some(now_us()));
                        }
                        if commissioning {
                            drivers[idx].runtime.request_restore_cancel();
                            continue 'control_loop;
                        }
                        if matches!(outcome.tail, TmcControlRequestTail::SkipTickTail) {
                            continue 'control_loop;
                        }
                    }
                    CommissionWaitRequestDisposition::RejectBusy => {
                        let outcome =
                            crate::tmc_control::effects::handle_commission_wait_reject(req);
                        if let Some(reason) = outcome.shutdown {
                            ctx.shared.app_state.lock(|state| {
                                if crate::firmware_ctx::enter_shutdown(state) {
                                    crate::tmc_control::effects::emit_shutdown(reason);
                                }
                            });
                        }
                    }
                    CommissionWaitRequestDisposition::DropBusy => {}
                }
                scheduler.set_mode(0, drivers[0].mode(), now_us());
                scheduler.set_mode(1, drivers[1].mode(), now_us());
            }

            let now = now_us();
            apply_live_maintenance_arm(&mut scheduler, drivers, now);
            let homing_state = HOMING_STATE.load(Relaxed);
            let Some(unit) = scheduler.pick(now, homing_state) else {
                continue;
            };
            let snapshot = scheduler
                .maintenance_group()
                .map_or(homing_state, |g| g.homing_snapshot());
            #[cfg(feature = "trace")]
            let engine_tick_deadline =
                Mono::now() + fugit::Duration::<u64, 1, 1_000_000>::millis(1);
            let ch = unit.channel;
            let idx = ch as usize;
            let unit_start = cortex_m::peripheral::DWT::cycle_count();
            let outcome = if unit.class == UnitClass::MaintenanceFinalize {
                let progress = if group_all_ready {
                    HomingPreparationProgress::Advanced
                } else if group_retry {
                    HomingPreparationProgress::Retry
                } else {
                    HomingPreparationProgress::Deferred
                };
                foci_firmware::tmc_control::TmcControlRuntime::advance_homing_state_after_channels(
                    foci_firmware::tmc_control::TmcControlHomingRefs::new(
                        &HOMING_STATE,
                        &HOMING_TRIGGERED,
                        &HOMING_REENTER_PENDING,
                    ),
                    snapshot,
                    group_all_ready,
                );
                if snapshot == 1 && HomingWakePolicy::<TmcRequest>::should_clear_latch(progress) {
                    HOMING_START_PENDING.store(false, Release);
                }
                group_all_ready = true;
                group_retry = false;
                None
            } else {
                match device_or_invalid(mux, ch) {
                    ChannelDevice::Device(device) => {
                        let mut tmc = tmc_driver(device);
                        let mut backend =
                            ouroboros_backend(ch, &mut tmc, enable_pin(enable_pins, ch));
                        let integral_candidate = integral_candidates
                            .slot_mut(ch)
                            .expect("scheduled TMC channel");
                        let ctx_unit = drive_ctx(
                            now,
                            snapshot,
                            levels[idx],
                            &tmc_control_config,
                            integral_candidate,
                        );
                        Some(drivers[idx].drive(unit, &mut backend, ctx_unit).await)
                    }
                    ChannelDevice::Invalid => {
                        shutdown!("Invalid TMC channel");
                        None
                    }
                }
            };
            let unit_cycles = cortex_m::peripheral::DWT::cycle_count().wrapping_sub(unit_start);
            scheduler.record(now_us(), unit_cycles);
            let Some(outcome) = outcome else {
                continue;
            };

            match outcome {
                UnitOutcome::Idle => {}
                UnitOutcome::Reschedule { fast_period_us } => {
                    scheduler.set_fast_period(ch, fast_period_us, now_us());
                }
                UnitOutcome::Ready => {
                    if ch == 0 {
                        enable_status_exti_ch0();
                    } else {
                        enable_status_capture_ch1();
                    }
                    scheduler.set_mode(ch, ChannelMode::Ready, now_us());
                }
                UnitOutcome::Parked(reason) => {
                    scheduler.set_mode(ch, ChannelMode::Parked, now_us());
                    match park_shutdown(ch, reason) {
                        crate::tmc_control::board::ParkShutdown::SpiFailedAtBoot => {
                            shutdown!("TMC4671 SPI failed at boot");
                        }
                        crate::tmc_control::board::ParkShutdown::ChipIdMismatch => {
                            shutdown!("TMC4671 chip ID mismatch");
                        }
                        crate::tmc_control::board::ParkShutdown::PiRepresentationFailedAtBoot => {
                            shutdown!("TMC4671 PI representation failed at boot");
                        }
                        crate::tmc_control::board::ParkShutdown::Reason(reason) => {
                            shutdown_reason!(reason);
                        }
                    }
                }
                UnitOutcome::Shutdown(reason) => {
                    ctx.shared.app_state.lock(|state| {
                        if crate::firmware_ctx::enter_shutdown(state) {
                            crate::tmc_control::effects::emit_shutdown(reason);
                        }
                    });
                }
                UnitOutcome::Maintenance { homing } => {
                    match homing {
                        foci_firmware::tmc_control::HomingMaintenanceResult::ReadyToAdvance => {}
                        foci_firmware::tmc_control::HomingMaintenanceResult::RetryCurrentState => {
                            group_all_ready = false;
                            group_retry = true;
                        }
                        foci_firmware::tmc_control::HomingMaintenanceResult::Deferred => {
                            group_all_ready = false;
                        }
                        foci_firmware::tmc_control::HomingMaintenanceResult::FailClosed => {
                            group_all_ready = false;
                            group_retry = true;
                            tmc_state(ch).homing_clamp_active.store(false, Relaxed);
                            tmc_state(ch).stall_latched.store(false, Relaxed);
                            drivers[idx].runtime.clear_homing_runtime();
                            TMC_DISABLE_SIGNAL.signal(());
                            if HomingWakePolicy::<TmcRequest>::should_clear_latch(
                                HomingPreparationProgress::FailClosed,
                            ) {
                                HOMING_START_PENDING.store(false, Release);
                            }
                        }
                    }
                    #[cfg(feature = "trace")]
                    {
                        let refs = foci_firmware::tmc_control::TmcControlStateRefs::new(
                            tmc_state(ch),
                            stepper_state(ch),
                        );
                        let trace_tick = drivers[idx].runtime.advance_trace_tick();
                        let inputs =
                            foci_firmware::trace::sampling::TraceSampleInputs::from_runtime_trace(
                                drivers[idx].runtime.trace_state(),
                                trace_tick,
                            );
                        let step_timing = if ch == 0 {
                            let snapshot = crate::usb_trace::take_step_timing_snapshot();
                            if crate::usb_trace::is_configured() {
                                Some(snapshot)
                            } else {
                                None
                            }
                        } else {
                            None
                        };
                        if let ChannelDevice::Device(device) = device_or_invalid(mux, ch) {
                            let mut tmc = tmc_driver(device);
                            crate::usb_trace::record_spi_sample_cycles(
                                crate::usb_trace::sample_tick(
                                    &mut tmc,
                                    crate::usb_trace::TraceSampleContext {
                                        channel: ch,
                                        refs,
                                        motor_enabled: drivers[idx].runtime.is_motor_enabled(),
                                        step_timing,
                                    },
                                    inputs,
                                )
                                .await,
                            );
                            crate::usb_trace::emit_status_frames(
                                ch,
                                &mut tmc,
                                trace_tick.control_tick,
                                trace_tick.status_detail_due,
                            )
                            .await;
                            crate::usb_trace::request_spi_duration_event_if_ready();
                            if trace_tick.metadata_due {
                                crate::usb_trace::emit_metadata();
                            }
                        }
                    }
                }
                UnitOutcome::Audit(result) => {
                    match result {
                        AuditResult::Clean | AuditResult::Warning { .. } => {}
                        AuditResult::Critical { .. }
                        | AuditResult::PinMismatch { .. }
                        | AuditResult::ReadFailed => {
                            FAULT_ACTIVE.store(true, Relaxed);
                            TMC_DISABLE_SIGNAL.signal(());
                        }
                    }
                    if ch == 0 {
                        let f0 = scheduler.fast_stats(0);
                        let f1 = scheduler.fast_stats(1);
                        let c0 = scheduler.commission_stats(0);
                        let c1 = scheduler.commission_stats(1);
                        let m = scheduler.maintenance_stats();
                        let a = scheduler.audit_stats();
                        let arm = scheduler.maintenance_arm_stats();
                        defmt::info!(
                            "sched fast0 req={} svc={} miss={} late={}us cyc={} | fast1 req={} svc={} miss={} late={}us cyc={} | comm0 svc={} late={}us | comm1 svc={} late={}us | maint svc={} miss={} late={}us | audit svc={} miss={} | maint arm={:?} a2d={} d2a={}",
                            f0.requested,
                            f0.serviced,
                            f0.missed,
                            f0.max_lateness_us,
                            f0.max_unit_cycles,
                            f1.requested,
                            f1.serviced,
                            f1.missed,
                            f1.max_lateness_us,
                            f1.max_unit_cycles,
                            c0.serviced,
                            c0.max_lateness_us,
                            c1.serviced,
                            c1.max_lateness_us,
                            m.serviced,
                            m.missed,
                            m.max_lateness_us,
                            a.serviced,
                            a.missed,
                            arm.state,
                            arm.armed_to_disarmed,
                            arm.disarmed_to_armed,
                        );
                    }
                }
                UnitOutcome::Commission { event, next } => {
                    if let Some(event) = event {
                        match event {
                            foci_firmware::commissioning::schedule::CommissionEvent::PhaseChanged {
                                oid,
                                phase_id,
                                ..
                            } => emit_phase_reply(oid, phase_id),
                            foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegral {
                                oid,
                                event,
                            } => crate::ankyra::replies::emit_velocity_integral_event(oid, event),
                            foci_firmware::commissioning::schedule::CommissionEvent::BreakawayCampaign {
                                oid,
                                event,
                            } => crate::ankyra::replies::emit_breakaway_campaign_event(oid, event),
                            foci_firmware::commissioning::schedule::CommissionEvent::VelocityIntegralPlan {
                                oid,
                                run_sequence,
                                plan,
                            } => crate::ankyra::replies::emit_velocity_integral_plan(
                                oid,
                                run_sequence,
                                plan,
                            ),
                        }
                    }
                    #[cfg(feature = "trace")]
                    if let Some((engine_kind, phase_id)) = drivers[idx].commission_trace_phase
                        && foci_firmware::trace::commissioning::commission_trace_phase_samples_closed_loop_activation(
                            engine_kind,
                            phase_id,
                        )
                    {
                        let trace_tick = drivers[idx].runtime.advance_trace_sample_tick();
                        let refs = foci_firmware::tmc_control::TmcControlStateRefs::new(tmc_state(ch), stepper_state(ch));
                        let motor_enabled = drivers[idx].runtime.is_motor_enabled();
                        let step_timing = crate::usb_trace::take_step_timing_snapshot();
                        let step_timing = if crate::usb_trace::is_configured() {
                            Some(step_timing)
                        } else {
                            None
                        };
                        match device_or_invalid(mux, ch) {
                            ChannelDevice::Device(device) => {
                                let mut tmc = tmc_driver(device);
                                crate::usb_trace::sample_commission_closed_loop_activation(
                                    &mut tmc,
                                    crate::usb_trace::TraceSampleContext {
                                        channel: ch,
                                        refs,
                                        motor_enabled,
                                        step_timing,
                                    },
                                    trace_tick,
                                    || {
                                        engine_tick_deadline
                                            .checked_duration_since(Mono::now())
                                            .map(|d| d.to_micros())
                                            .unwrap_or(0)
                                    },
                                    || Mono::now() > engine_tick_deadline,
                                )
                                .await;
                            }
                            ChannelDevice::Invalid => {
                                shutdown!("Invalid TMC channel");
                                continue 'control_loop;
                            }
                        }
                    }
                    match next {
                        CommissionNext::PollNow => scheduler.set_commission_due(ch, Some(now_us())),
                        CommissionNext::WaitUntil(deadline_us) => {
                            scheduler.set_commission_due(ch, Some(deadline_us))
                        }
                        CommissionNext::Yield => {
                            scheduler.note_commission_yield(ch);
                            scheduler.set_commission_due(ch, Some(now_us()));
                        }
                    }
                    scheduler.set_mode(ch, drivers[idx].mode(), now_us());
                }
                UnitOutcome::CommissionTerminal { reply, actions } => {
                    {
                        let mut state = drivers[idx]
                            .runtime
                            .dispatch_state(tmc_state(ch), stepper_state(ch));
                        apply_stepper_post_action(actions.stepper, &mut state, ch);
                    }
                    foci_firmware::diagnostic::mark_p1_reply_construction_start();
                    emit_terminal_reply(ch, reply);
                    foci_firmware::diagnostic::mark_p1_reply_construction_end();
                    defmt::info!(
                        "channel {} commissioning terminal max tick cycles: {}",
                        ch,
                        scheduler.commission_stats(ch).max_unit_cycles
                    );
                    scheduler.set_commission_due(ch, None);
                    scheduler.set_mode(ch, drivers[idx].mode(), now_us());
                }
            }
        }
    }
    #[inline]
    fn step_clock_is_due(clock: u32, now: u32) -> bool {
        (clock as i32).wrapping_sub(now as i32) <= 0
    }

    fn program_next_step_compare(
        next_event_clocks: &[Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        let tim5 = embassy_stm32::pac::TIM5;
        if let Some(due) = foci_firmware::stepper_runtime::earliest_due(next_event_clocks) {
            arm_step_timer(due.clock);
        } else {
            tim5.dier().modify(|w| w.set_ccie(0, false));
        }
    }

    fn clear_step_channel(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        let idx = channel as usize;
        if idx < next_event_clocks.len() {
            next_event_clocks[idx] = None;
        }
        if let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) {
            stepper_state.interval.store(0, Relaxed);
            stepper_state.timer_active_mirror.store(false, Relaxed);
        }
        if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
            tmc_state.pid_error_sums_clear.store(true, Relaxed);
        }
        with_trigger_domain(|td| {
            crate::trigger_domain::set_timer_active(td.state_mut(), idx, false);
        });
    }

    fn fail_step_channel_deadline(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
            tmc_state
                .motor_armed
                .store(false, core::sync::atomic::Ordering::Relaxed);
        }
        clear_step_channel(channel, next_event_clocks);
        // SAFETY: ISR stepper state is only accessed from the P3 TIM5 ISR
        // for this channel. P3 tasks cannot preempt each other on Cortex-M.
        unsafe {
            if let Some(stepper) = crate::state::stepper::isr_stepper(channel) {
                stepper.reset();
            }
        }
        let _ = crate::stepper_pins::reset_idle(channel);
        TMC_DISABLE_SIGNAL.signal(());
    }

    fn abort_step_dispatch(
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) {
        use core::sync::atomic::Ordering::Relaxed;

        embassy_stm32::pac::TIM5
            .dier()
            .modify(|w| w.set_ccie(0, false));
        for idx in 0..crate::state::stepper::STEPPER_CHANNEL_COUNT {
            let channel = idx as u8;
            if let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) {
                tmc_state.motor_armed.store(false, Relaxed);
            }
            clear_step_channel(channel, next_event_clocks);
            // SAFETY: ISR stepper state is only accessed from the P3 TIM5 ISR.
            unsafe {
                if let Some(stepper) = crate::state::stepper::isr_stepper(channel) {
                    stepper.reset();
                }
            }
            let _ = crate::stepper_pins::reset_idle(channel);
        }
    }

    fn channel_stepper_ready(channel: u8) -> bool {
        use core::sync::atomic::Ordering::Relaxed;

        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            return false;
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            return false;
        };
        tmc_state.motor_armed.load(Relaxed) && stepper_state.timer_active_mirror.load(Relaxed)
    }

    #[inline(always)]
    fn spin_tim5_ticks(start: u32, ticks: u32) {
        while read_tim5().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }

    struct OuroborosPulseTrainIo {
        channel: u8,
    }

    impl crate::pulse_train::PulseTrainIo for OuroborosPulseTrainIo {
        type Error = ();

        #[inline(always)]
        fn set_step(&mut self, high: bool) -> Result<(), Self::Error> {
            crate::stepper_pins::set_step(self.channel, high)
        }

        #[inline(always)]
        fn set_dir(&mut self, forward: bool) -> Result<(), Self::Error> {
            crate::stepper_pins::set_dir(self.channel, forward)
        }

        #[inline(always)]
        fn wait_ticks(&mut self, ticks: u32) {
            let start = read_tim5();
            spin_tim5_ticks(start, ticks);
        }
    }

    /// Emit one bounded logical event as a complete, non-preemptible pulse train.
    ///
    /// This P3 hot path performs no SPI, logging, allocation, scheduler work,
    /// or yield. The mapper guarantees zero, one, or two physical pulses.
    #[inline(always)]
    fn emit_logical_step_event(
        channel: u8,
        event: foci_klipper::stepper::LogicalStepEvent,
    ) -> Result<(), ()> {
        crate::pulse_train::emit_logical_step_event(
            &mut OuroborosPulseTrainIo { channel },
            event,
            crate::state::stepper::STEP_DIR_SETUP_TICKS,
            crate::state::stepper::STEP_HALF_PERIOD_TICKS,
        )
    }

    fn load_next_step_move(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        use core::sync::atomic::Ordering::Relaxed;

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return StepDispatchOutcome::Continue;
        }
        let Some(tmc_state) = foci_firmware::state::tmc::tmc_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        if stepper.has_active_segment() {
            if next_event_clocks[idx].is_none() {
                next_event_clocks[idx] = Some(stepper.next_step_clock());
            }
            return StepDispatchOutcome::Continue;
        }

        // SAFETY: P3 is the sole consumer of this channel's queue. P3 ISRs
        // cannot preempt each other on Cortex-M.
        let next_move =
            unsafe { crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].isr_dequeue() };
        let Some(move_) = next_move else {
            stepper.mark_idle();
            crate::state::stepper_diag::record_queue_empty(channel);
            if HOMING_STATE.load(Relaxed) != 0 {
                record_stepper_event(
                    StepperEventReason::QueueEmpty,
                    channel,
                    read_tim5(),
                    0,
                    0,
                    0,
                );
            }
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        let loaded_steps = u32::from(move_.count);

        let old_direction = stepper.dir_state();
        let phase_transition = stepper.load_move(move_);
        #[cfg(feature = "trace")]
        if let Some((old_class, new_class)) = phase_transition {
            crate::usb_trace::request_event(
                foci_firmware::trace::events::EVENT_LEVEL_INFO,
                foci_firmware::trace::events::SUBSYSTEM_STEP_TIMER,
                foci_firmware::trace::events::EVENT_PHASE_BOUNDARY,
                [i32::from(channel), old_class as i32, new_class as i32, 0],
                3,
            );
        }
        #[cfg(not(feature = "trace"))]
        let _ = phase_transition;
        let dir_changed = old_direction != stepper.dir_state();
        stepper_state
            .interval
            .store(stepper.current_interval(), Relaxed);
        if dir_changed {
            tmc_state.feedforward_reset.store(true, Relaxed);
        }

        let tim5 = embassy_stm32::pac::TIM5;
        let now = tim5.cnt().read();
        let scheduled = stepper.next_step_clock();
        match stepper.check_deadline(scheduled, now, CLOCK_FREQ) {
            Ok(compare) => {
                crate::state::stepper_diag::record_loaded_segment_timing(
                    channel,
                    loaded_steps,
                    now,
                    scheduled,
                    compare,
                );
                let immediate = compare != scheduled;
                if immediate {
                    next_event_clocks[idx] = Some(now);
                } else {
                    next_event_clocks[idx] = Some(compare);
                }
            }
            Err(foci_klipper::stepper::MissedDeadline) => {
                crate::state::stepper_diag::record_loaded_segment(channel, loaded_steps);
                crate::state::stepper_diag::record_missed_deadline(channel);
                record_stepper_event(
                    StepperEventReason::MissedDeadlineLoad,
                    channel,
                    now,
                    UNKNOWN_QUEUE_LEN,
                    scheduled,
                    now,
                );
                #[cfg(feature = "trace")]
                crate::usb_trace::note_missed_or_deferred_step();
                STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                fail_step_channel_deadline(channel, next_event_clocks);
                return StepDispatchOutcome::Fatal;
            }
        }
        StepDispatchOutcome::Continue
    }

    fn process_due_step_event(
        channel: u8,
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_klipper::stepper::StepEventResult;

        let idx = channel as usize;
        if !channel_stepper_ready(channel) {
            next_event_clocks[idx] = None;
            return StepDispatchOutcome::Continue;
        }
        let Some(stepper_state) = foci_firmware::state::stepper::stepper_channel(channel) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };

        // SAFETY: ISR stepper state is only accessed from this P3 ISR for
        // this channel. P3 tasks cannot preempt each other on Cortex-M.
        let Some(stepper) = (unsafe { crate::state::stepper::isr_stepper(channel) }) else {
            clear_step_channel(channel, next_event_clocks);
            return StepDispatchOutcome::Continue;
        };
        if !stepper.has_active_segment() {
            next_event_clocks[idx] = None;
            return load_next_step_move(channel, next_event_clocks);
        }
        if step_held_for_homing(stepper.next_step_clock()) {
            next_event_clocks[idx] = Some(read_tim5().wrapping_add(CLOCK_FREQ / 1000));
            return StepDispatchOutcome::Continue;
        }

        let result = stepper.step_event();
        let event = result.event();
        if emit_logical_step_event(channel, event).is_err() {
            fail_step_channel_deadline(channel, next_event_clocks);
            return StepDispatchOutcome::Fatal;
        }

        let now = embassy_stm32::pac::TIM5.cnt().read();
        crate::state::stepper::publish_logical_event(channel, stepper, event);
        let logical_delta = if event.direction { 1 } else { -1 };
        stepper_state.position.fetch_add(logical_delta, Relaxed);
        stepper_state.direction.store(event.direction, Relaxed);
        stepper_state
            .interval
            .store(stepper.current_interval(), Relaxed);
        crate::state::stepper_diag::record_executed_step(
            channel,
            event.direction,
            event.physical_pulses,
            now,
        );

        match result {
            StepEventResult::ScheduleNext { clock, .. } => {
                match stepper.check_deadline(clock, now, CLOCK_FREQ) {
                    Ok(compare) => {
                        let immediate = compare != clock;
                        next_event_clocks[idx] = Some(if immediate { now } else { compare });
                    }
                    Err(foci_klipper::stepper::MissedDeadline) => {
                        crate::state::stepper_diag::record_missed_deadline(channel);
                        record_stepper_event(
                            StepperEventReason::MissedDeadlineStep,
                            channel,
                            now,
                            UNKNOWN_QUEUE_LEN,
                            clock,
                            now,
                        );
                        #[cfg(feature = "trace")]
                        crate::usb_trace::note_missed_or_deferred_step();
                        STEPPER_DEADLINE_MISS_CHANNEL.store(channel, Relaxed);
                        fail_step_channel_deadline(channel, next_event_clocks);
                        return StepDispatchOutcome::Fatal;
                    }
                }
            }
            StepEventResult::NeedMove { .. } => {
                next_event_clocks[idx] = None;
                return load_next_step_move(channel, next_event_clocks);
            }
        }
        StepDispatchOutcome::Continue
    }

    fn refresh_step_channels(
        next_event_clocks: &mut [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT],
    ) -> StepDispatchOutcome {
        for idx in 0..crate::state::stepper::STEPPER_CHANNEL_COUNT {
            let channel = idx as u8;
            if !channel_stepper_ready(channel) {
                next_event_clocks[idx] = None;
                continue;
            }
            // A stop that happens outside this ISR (trigger stop, P1 stop,
            // commissioning stimulus stop) resets the channel's stepper engine
            // but cannot touch this ISR-local schedule, and it disables CC1
            // when no other channel is active, so no ISR run follows to clear
            // the entry. When the channel is armed again for its next move it
            // is ready with no active segment but still carries that stale
            // clock, and the dispatch below would wait for the stale value to
            // come round on the 32-bit timer (up to 16.5 s) before loading
            // the first segment, which is then far past its deadline. A ready
            // channel without an active segment always needs a load.
            //
            // SAFETY: ISR stepper state is only accessed from this P3 ISR for
            // this channel. P3 tasks cannot preempt each other on Cortex-M.
            let has_active_segment = unsafe { crate::state::stepper::isr_stepper(channel) }
                .is_some_and(|stepper| stepper.has_active_segment());
            if !has_active_segment {
                next_event_clocks[idx] = None;
            }
            if next_event_clocks[idx].is_none()
                && load_next_step_move(channel, next_event_clocks) == StepDispatchOutcome::Fatal
            {
                return StepDispatchOutcome::Fatal;
            }
        }
        StepDispatchOutcome::Continue
    }

    /// P2 software task: reconcile pending stepper stop and arm state.
    #[task(priority = 2)]
    async fn stepper_reconcile(_ctx: stepper_reconcile::Context) {
        loop {
            STEPPER_RECONCILE_WAKE.wait().await;
            loop {
                let pending = STEPPER_RECONCILE_PENDING.take_pending();
                if pending.is_empty() {
                    break;
                }
                with_trigger_domain(|td| {
                    let mut board = crate::firmware_ctx::OuroborosStepperBoard { tmc_state: None };
                    drain_stepper_reconciliation(td.state_mut(), &mut board, pending);
                });
            }
        }
    }

    /// P2 software task: drain step queues detached by masked trigger
    /// stops (trigger dispatch, shutdown, commissioning stimulus stop), with
    /// interrupts enabled.
    #[task(priority = 2)]
    async fn trigger_drain_task(_ctx: trigger_drain_task::Context) {
        loop {
            crate::trigger_domain::TRIGGER_DRAIN_WAKE.wait().await;
            crate::trigger_domain::drain_all_populated_slots();
        }
    }

    impl StepperStopCheckBoard for crate::firmware_ctx::OuroborosStepperBoard {
        fn stop_check_index(&self, channel: u8) -> Option<usize> {
            Some(channel as usize)
        }

        fn clear_step_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            _channel: u8,
        ) {
            self.set_timer_active(state, stepper_idx, false);
        }
    }

    impl StepperArmCheckBoard for crate::firmware_ctx::OuroborosStepperBoard {
        fn arm_check_index(&self, channel: u8) -> Option<usize> {
            foci_firmware::state::tmc::tmc_channel(channel).map(|_| channel as usize)
        }

        fn motor_armed(&self, channel: u8) -> bool {
            foci_firmware::state::tmc::tmc_channel(channel)
                .map(|state| {
                    state
                        .motor_armed
                        .load(core::sync::atomic::Ordering::Relaxed)
                })
                .unwrap_or(false)
        }

        fn queue_has_moves(&self, channel: u8) -> bool {
            !crate::state::stepper::STEP_QUEUE_CHANNELS[channel as usize].is_empty()
        }

        fn arm_check_timer(
            &mut self,
            state: &mut foci_firmware::context::trsync::TriggerDomainState,
            stepper_idx: usize,
            channel: u8,
        ) {
            let now = read_tim5();
            crate::state::stepper_diag::record_timer_activation(channel, now);
            arm_step_timer_if_earlier(now.wrapping_add(1));
            self.set_timer_active(state, stepper_idx, true);
        }
    }

    /// TIM5 CC1 step timer interrupt.
    ///
    /// Dequeues moves from SPSC consumer, toggles STEP GPIO, schedules
    /// next compare. Same priority as USB OTG and fault EXTI (P3).
    #[task(
        binds = TIM5,
        priority = 3,
        local = [
            next_event_clocks: [Option<u32>; crate::state::stepper::STEPPER_CHANNEL_COUNT] =
                [None; crate::state::stepper::STEPPER_CHANNEL_COUNT],
        ],
    )]
    fn step_timer_irq(ctx: step_timer_irq::Context) {
        use core::sync::atomic::Ordering::Relaxed;

        let tim5 = embassy_stm32::pac::TIM5;
        let sr = tim5.sr().read();
        let dier = tim5.dier().read();
        let cc1_pending = sr.ccif(0) && dier.ccie(0);
        let cc2_pending = sr.ccif(1) && dier.ccie(1);
        if !cc1_pending && !cc2_pending {
            return;
        }

        if cc2_pending {
            clear_tim5_compare_flag(1);
            let mut need_report = false;
            with_trigger_domain(|td| {
                use foci_klipper::scheduler::SchedulerEventKind;
                let now = read_tim5();
                let mut due_events: heapless::Vec<(SchedulerEventKind, u32), 13> =
                    heapless::Vec::new();
                while let Some(slot) = td.state_mut().scheduler.pop_due_slot(now) {
                    let event = slot.event();
                    let _ = due_events.push((event.kind, event.wake_clock));
                }

                for &(kind, wake_clock) in &due_events {
                    match kind {
                        SchedulerEventKind::Stepper(_) => {}
                        SchedulerEventKind::EndstopSample(endstop_idx) => {
                            use foci_klipper::endstop::{EndstopPhase, EndstopSampleResult};
                            let pin_id = ENDSTOP_PIN_IDS[endstop_idx].load(Relaxed);
                            let read_pin = || -> bool {
                                use crate::ankyra::enumerations::Pins;
                                let stall_channel = match Pins::try_from(pin_id) {
                                    Ok(Pins::STALL0) => Some(0),
                                    Ok(Pins::STALL1) => Some(1),
                                    _ => None,
                                };
                                match stall_channel {
                                    Some(channel) => tmc_state(channel).stall_latched.load(Relaxed),
                                    None if pin_id != 0xFF => gpio_read_pin(pin_id),
                                    None => false,
                                }
                            };

                            let phase = td.state().endstops[endstop_idx].phase();
                            let result = match phase {
                                EndstopPhase::Scanning => td.state_mut().endstops[endstop_idx]
                                    .scan(read_pin(), wake_clock),
                                EndstopPhase::Oversampling => td.state_mut().endstops[endstop_idx]
                                    .oversample(read_pin(), wake_clock),
                            };

                            match result {
                                EndstopSampleResult::EnterOversamplingNow => {
                                    match td.state_mut().endstops[endstop_idx]
                                        .oversample(read_pin(), wake_clock)
                                    {
                                        EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                            td.trigger(trsync_oid, reason, now);
                                            need_report = true;
                                        }
                                        EndstopSampleResult::Reschedule { next_clock } => {
                                            td.state_mut().scheduler.try_schedule(
                                                foci_klipper::scheduler::SchedulerEvent {
                                                    wake_clock: next_clock,
                                                    kind: SchedulerEventKind::EndstopSample(
                                                        endstop_idx,
                                                    ),
                                                },
                                            );
                                        }
                                        _ => {}
                                    }
                                }
                                EndstopSampleResult::Triggered { trsync_oid, reason } => {
                                    td.trigger(trsync_oid, reason, now);
                                    need_report = true;
                                }
                                EndstopSampleResult::Reschedule { next_clock } => {
                                    td.state_mut().scheduler.try_schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_clock,
                                            kind: SchedulerEventKind::EndstopSample(endstop_idx),
                                        },
                                    );
                                }
                                EndstopSampleResult::Inactive => {}
                            }
                        }
                        SchedulerEventKind::TrSyncExpire(trsync_idx) => {
                            let oid = td
                                .state()
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid {
                                let expire_reason = td.state().trsync[trsync_idx].expire_reason();
                                td.trigger(oid, expire_reason, now);
                                need_report = true;
                            }
                        }
                        SchedulerEventKind::TrSyncReport(trsync_idx) => {
                            let oid = td
                                .state()
                                .trsync_by_oid
                                .iter()
                                .find(|&(_, &idx)| idx == trsync_idx)
                                .map(|(&oid, _)| oid);
                            if let Some(oid) = oid
                                && td.state().trsync[trsync_idx].can_trigger()
                            {
                                td.state_mut().trsync[trsync_idx].set_liveness_report(oid, now);
                                need_report = true;
                                let report_ticks = td.state().trsync[trsync_idx].report_ticks();
                                if report_ticks > 0 {
                                    let next_report = td.state().trsync[trsync_idx]
                                        .report_clock()
                                        .wrapping_add(report_ticks);
                                    td.state_mut().trsync[trsync_idx].set_report_clock(next_report);
                                    td.state_mut().scheduler.try_schedule(
                                        foci_klipper::scheduler::SchedulerEvent {
                                            wake_clock: next_report,
                                            kind: SchedulerEventKind::TrSyncReport(trsync_idx),
                                        },
                                    );
                                }
                            }
                        }
                    }
                }
                crate::trigger_domain::reprogram_cc2(td.state_mut());
            });
            if need_report {
                REPORT_PENDING.store(true, Relaxed);
                report_task::spawn().ok();
            }
        }

        if !tim5.sr().read().ccif(0) || !tim5.dier().read().ccie(0) {
            return;
        }
        clear_tim5_compare_flag(0);

        let next_event_clocks = ctx.local.next_event_clocks;
        if FAULT_ACTIVE.load(Relaxed) {
            *next_event_clocks = [None; crate::state::stepper::STEPPER_CHANNEL_COUNT];
            tim5.dier().modify(|w| w.set_ccie(0, false));
            return;
        }

        let mut budget = foci_firmware::stepper_runtime::StepIrqBudget::new();
        let outcome = run_step_dispatch(|| {
            if refresh_step_channels(next_event_clocks) == StepDispatchOutcome::Fatal {
                return StepDispatchOutcome::Fatal;
            }
            let Some(due) = foci_firmware::stepper_runtime::earliest_due(next_event_clocks) else {
                return StepDispatchOutcome::Complete;
            };
            let now = tim5.cnt().read();
            if !step_clock_is_due(due.clock, now) {
                return StepDispatchOutcome::Complete;
            }
            if !budget.take() {
                #[cfg(feature = "trace")]
                crate::usb_trace::note_missed_or_deferred_step();
                return StepDispatchOutcome::Complete;
            }
            #[cfg(feature = "trace")]
            crate::usb_trace::record_step_timing(due.clock, now);
            process_due_step_event(due.channel, next_event_clocks)
        });
        if outcome == StepDispatchOutcome::Fatal {
            abort_step_dispatch(next_event_clocks);
            return;
        }
        program_next_step_compare(next_event_clocks);
    }

    /// USB OTG HS interrupt handler. Masks interrupt sources and wakes usb_task.
    ///
    /// Priority 3: the shared handler completes its FIFO and MMIO work through
    /// its terminal barrier before the P2 USB consumer can execute.
    #[task(binds = OTG_HS, priority = 3)]
    fn otg_hs_irq(_ctx: otg_hs_irq::Context) {
        crate::interrupts::otg_fs_irq();
    }

    /// TMC4671 STATUS pin interrupt (PE2, EXTI line 2).
    ///
    /// Fires when any STATUS_MASK-enabled flag is asserted (`adc_i_clipped`).
    /// Signals immediate motor disable; tmc_control reads
    /// STATUS_FLAGS on next tick to log the cause.
    #[task(binds = EXTI2, priority = 3)]
    fn tmc_status_irq(_ctx: tmc_status_irq::Context) {
        crate::interrupts::tmc_status_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    /// TMC4671 channel-1 STATUS pin interrupt (PA2, TIM15 CH1 input capture).
    ///
    /// PA2 cannot share EXTI line 2 with PE2, so channel 1's STATUS line is
    /// routed to a TIM15 input-capture interrupt instead. Same disable path
    /// as `tmc_status_irq`.
    #[task(binds = TIM15, priority = 3)]
    fn tmc_status1_irq(_ctx: tmc_status1_irq::Context) {
        crate::interrupts::tmc_status1_irq(&FAULT_ACTIVE, &TMC_DISABLE_SIGNAL);
    }

    /// Single USB task replacing usb_pump, usb_receive, usb_send, usb_control.
    ///
    /// Handles EP0 setup, bulk RX, protocol dispatch, and bulk TX in one pass.
    /// No scheduling gap between receiving commands and sending acks.
    /// Katapult 1200-baud DTR detection is handled by klipper-usb internally.
    #[task(priority = 2, shared = [app_state])]
    async fn usb_task(mut ctx: usb_task::Context) {
        let mut backend = crate::usb_task::OuroborosUsbBackend::new(
            |f| ctx.shared.app_state.lock(f),
            &REPORT_PENDING,
            &USB_PUMP_EMITTER,
            &USB_PUMP_COUNTERS,
        );
        loop {
            foci_firmware::usb_pump::run_once(&mut backend, &USB_PUMP_COUNTERS).await;
        }
    }

    /// Send pending trsync_state reports outside the TriggerDomain lock.
    ///
    /// Spawned by the TIM5 CC2 ISR when any trsync has a pending report.
    /// Drains all pending snapshots in one brief critical section, then
    /// sends replies at P2 without holding the lock.
    #[task(priority = 2)]
    async fn report_task(_ctx: report_task::Context) {
        crate::heartbeat::emit_trigger_reports(
            |snaps| {
                with_trigger_domain(|td| {
                    for ts in td.state_mut().trsync.iter_mut() {
                        if let Some(snap) = ts.take_report() {
                            let _ = snaps.push(snap);
                        }
                    }
                });
            },
            &REPORT_PENDING,
        );
    }

    /// Send pending stepper diagnostic events outside ISR/P1 contexts.
    #[task(priority = 2)]
    async fn stepper_diag_task(_ctx: stepper_diag_task::Context) {
        crate::heartbeat::emit_stepper_diagnostics(&STEPPER_DIAG_PENDING);
    }

    /// Periodic stats heartbeat sent to Klipper.
    ///
    /// Klipper requires regular stats messages to confirm the MCU is alive.
    /// Sends `stats count=%u sum=%u sumsq=%u` every 5 seconds with basic
    /// CPU usage metrics.
    #[task(priority = 1)]
    async fn stats(_ctx: stats::Context) {
        let mut accumulator = crate::heartbeat::StatsAccumulator::new(read_tim5());
        loop {
            Mono::delay(5000.millis()).await;
            crate::heartbeat::stats_tick(
                &mut accumulator,
                read_tim5(),
                &STATS_SEND_TIME,
                &STATS_SEND_TIME_HIGH,
            );
        }
    }

    /// Service the independent watchdog above sustained priority-1 work.
    #[task(priority = 2, local = [wdg])]
    async fn watchdog(ctx: watchdog::Context) {
        loop {
            Mono::delay(crate::heartbeat::WATCHDOG_INTERVAL_MS.millis()).await;
            crate::heartbeat::watchdog_tick(ctx.local.wdg);
        }
    }

    /// Sample TH0/TH1/MCU_TEMP at low priority and report readings to the
    /// host, entering the shutdown state on a sustained out-of-range
    /// reading.
    ///
    /// The 1ms poll period is coarse relative to `AnalogChannelState`'s own
    /// `sample_ticks`/`rest_ticks` pacing (in TIM5-domain ticks via
    /// `read_tim5()`, not this loop's `Mono` wakeup) -- this task only needs
    /// to wake often enough that no configured channel's due sample is ever
    /// late by more than one period.
    #[task(
        priority = 1,
        shared = [app_state],
        local = [adc, adc_internal, temp_channel, th0_pin, th1_pin, mcu_temp_calibration]
    )]
    async fn analog_sampling(mut ctx: analog_sampling::Context) {
        loop {
            Mono::delay(1.millis()).await;
            let now = read_tim5();
            let shutdown_requested = crate::heartbeat::analog_sampling_tick(
                ctx.local.adc,
                ctx.local.adc_internal,
                ctx.local.temp_channel,
                ctx.local.th0_pin,
                ctx.local.th1_pin,
                ctx.local.mcu_temp_calibration,
                now,
            );
            if shutdown_requested {
                ctx.shared.app_state.lock(|state| {
                    if crate::firmware_ctx::enter_shutdown(state) {
                        foci_firmware::dispatch_shutdown!(
                            foci_firmware::shutdown::ShutdownReason::AdcOutOfRange,
                            now,
                            &mut crate::_ankyra_config::Sender
                        );
                    }
                });
            }
        }
    }
}

foci_firmware::foci_ankyra_config! {
    transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
    context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
    providers = [
        foci_firmware::ankyra::provider::FOCI_FIRMWARE_PROVIDER,
        crate::ankyra::provider::BOARD_PROVIDER,
    ],
    board_static_strings = [
        "TMC4671 PI representation failed at boot",
        "TMC4671: STATUS_FLAGS read failed",
    ],
    app = "FOCI",
    version = env!("FOCI_VERSION"),
    build_versions = env!("FOCI_BUILD_VERSIONS"),
    license = "MIT OR Apache-2.0",
}
