// Booster Landing Controller
//   - does boostback, coasting, re-entry burn, and final descent
//   - ideally this doesn't depend on KSP but thats not been completely possiblle

using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    public enum BLControllerPhase
    {
        Unset,
        TurnAround,
        BoostBack,
        Coasting,
        ReentryBurn,
        AeroDescent,
        LandingBurn,
        // Starship recovery profile phases (await deorbit burn -> coast to
        // the atmosphere -> belly flop -> flip -> shared LandingBurn)
        AwaitDeorbit,
        EntryCoast,
        BellyFlop,
        Flip
    }

    public class BLController : Controller
    {
        // Public parameters
        public double touchdownSpeed = 2;
        public double aeroDescentMaxAoA = 20;
        public double aeroDescentSteerKp = 0.001f;
        public double reentryBurnAlt = 70000;
        public double reentryBurnTargetSpeed = 700;
        public double reentryBurnSteerKp = 0.001f;
        public double reentryBurnMaxAoA = 20;
        public double landingBurnHeight = 0; // Maximum altitude to enable powered descent
        // Ground reference (terrain under the position) that landingBurnHeight
        // was last computed against - forces a recalc when it diverges from
        // the live groundAlt as the vessel descends toward real terrain
        private double landingBurnHeightGroundRef = -10000;
        public double landingBurnSteerKp = 0.001f;
        public double landingBurnMaxAoA = 10;
        private double suicideFactor = 0.9f;
        private double lowestY = 0;
        private PIDclamp pid_reentry = new PIDclamp("reentrySteer", 1, 0, 0, 10);
        private PIDclamp pid_aero = new PIDclamp("aeroSteer", 1, 0, 0, 10);
        private PIDclamp pid_landing = new PIDclamp("landingSteer", 1, 0, 0, 10);
        public double igniteDelay = 3; // ignite engines this many seconds early
        public double simulationsPerSec = 10;
        public bool deployLandingGear = true;
        public double deployLandingGearHeight = 500;
        // Rate damping (seconds): steer corrections are reduced by steerDamping * lateral
        // angular rate to suppress descent/landing oscillation. 0 disables damping.
        public double steerDamping = 1.0;
        // Velocity damping (seconds) for terminal homing: steer target = predicted
        // error minus landingBurnVelDamp * horizontal velocity. DISABLED by default
        // (0) - at the high horizontal speeds seen on flight 12 (>300 m/s at burn
        // onset) the virtual aim point wandered hundreds of metres off-target.
        // Re-enable only for the sub-20 m/s terminal regime once validated.
        public double landingBurnVelDamp = 0.0;
        // Low-altitude AoA schedule: the effective max AoA is capped at
        // lowAoACap degrees (absolute, independent of the GUI-derived base
        // value) below aoaRampLowAlt, ramping linearly to full authority at
        // aoaRampTopAlt. Automates the manual habit (lowering the steer gain
        // so maxAoA ~7 deg below 5 km) that suppresses the descent
        // limit cycle. Flight 16 shook from burn onset (~14km) down to 5km
        // with the old 6km ramp top and was stable below 5km, so the ramp
        // now spans the whole fast descent (a dynamic-pressure model was
        // considered and rejected: 14km has LOWER q than 5km yet wobbled)
        public double lowAoACap = 7;
        public double aoaRampLowAlt = 4000;
        public double aoaRampTopAlt = 15000;
        // Turn-around phase (RCS-only flip to point the nose opposite the
        // horizontal velocity before boostback ignition): complete when the
        // attitude is within turnAroundCompleteAngle of the target, or after
        // turnAroundMaxTime seconds (weak RCS should not deadlock the return)
        public double turnAroundCompleteAngle = 10;
        public double turnAroundMaxTime = 60;
        private double turnAroundStart = -1;
        public double touchdownMargin = 30; // use touchdown speed from this height
        public double noSteerHeight = 200;
        // Below uprightHeight (and once horizontal speed is below uprightMaxHorizSpeed)
        // the landing burn commands a pure vertical attitude. 0 disables forced uprighting.
        public double uprightHeight = 250;
        public double uprightMaxHorizSpeed = 5;
        // Velocity-to-go terminal law. DISABLED by default (v2gHeight=0)
        // after flight 20 regressed to 471m: the position-chasing form
        // (v_des = -posErr/tGo) converges the vessel's POSITION but does not
        // align the trajectory LINE through the target - with no thrust to
        // brake at min throttle (6->2.5km) the vessel kept ~90-125 m/s,
        // flew PAST the target at 72m/2km (the sim's terr=328m was right
        // there, ignored), and the braking slide added 400m. The
        // impact-error law keeps the velocity line through the target and
        // is the proven scheme (flight 16: 31.3m). Re-enable only with an
        // alignment-aware formulation (v_des from the predicted impact
        // error, not from the raw position offset). The pure-braking form
        // below noSteerHeight (v2gTerminal) is safe: there the trajectory
        // is already aligned and killing the residual drift is the only
        // remaining task (flight 17/18: 35m of unopposed drift below 200m)
        public double v2gHeight = 0;
        public double v2gBlend = 2000;
        public double v2gKp = 1.0;       // degrees of correction per m/s of velocity error
        public double v2gMaxSpeed = 25;  // clamp on the commanded closing speed
        public double v2gMaxAoA = 12;    // correction budget for the velocity law
        // Velocity braking hands over from the impact-error law between
        // v2gTermHeight and noSteerHeight (pure v2g below). User feedback
        // from flight 21: engaging only at 200m left 20 m/s to kill in the
        // last seconds, so the vessel was still translating under a
        // 12-degree correction in the final 50m and touched down tilted
        // (broken landing leg). At 600m the throttle is already up (real
        // braking authority, unlike the min-throttle 2.5-6km band of the
        // flight-20 flyby) and the remaining distances are small
        public double v2gTermHeight = 600;
        public bool v2gTerminal = true;  // velocity braking below v2gTermHeight
        // Inside this radius the velocity law switches to BRAKE-ONLY
        // (v_des = 0): flight 22 reached 2.6m from the target at 80m, then
        // the v_des reversal ("stop, then go the other way") pumped the
        // horizontal speed 5->13 m/s at full throttle in the last 50m and
        // the vessel drifted away and tipped over. When it is this close,
        // stopping is the whole job - do not re-home
        public double v2gBrakeOnlyRadius = 15;
        // Hard tilt cap below lowTiltCapHeight: the retro-lean steer term
        // atan(vh/(vy+20)) explodes past 40 degrees as vy->0 at the end of
        // the suicide profile, which is what actually throws the vessel
        // over at touchdown (flight 22)
        public double lowTiltCapHeight = 100;
        public double lowTiltCap = 15;
        // Below this effective height the throttle law switches from the
        // suicide profile to the touchdown taper. MUST be > 0: the suicide
        // profile's feedforward slope d(dvy)/dt ~ 1/sqrt(y) diverges as
        // y->0+, forming a stable fixed point that parks the vessel in a
        // hover exactly at y=0 (flight 25: hovered 33s at 35m until the
        // tanks ran dry). The taper has a finite slope and always brings
        // the vessel down. 2m is above the divergence zone yet deep enough
        // that the taper target is always SLOWER than the suicide profile,
        // so the handoff is a catch (brake), never a dive
        public double taperSwitchY = 2;
        public bool useFAR = false;
        // Recovery profile: "falcon9" (tail-first retrograde descent) or
        // "starship" (belly flop + flip). Persisted per vessel on
        // BoosterGuidanceCore and pushed here by ConfigureController
        public string recoveryProfile = "falcon9";
        // Starship flip-trigger altitude clamp: the physics-boundary flip
        // altitude (fall distance during the fixed rotation + aero-credited
        // landing-burn height) is clamped to these bounds, and the
        // enable-anywhere auto-pick uses them as the Flip regime
        public double flipAltMin = 500;
        public double flipAltMax = 4000;
        // Belly-flop steering (design D6/D7): absolute tilt cap of the belly
        // normal, faded to zero across [flipAltMax, bellyCorrectionFadeAlt]
        // where the flip takes over. starshipCorrectionGain is the live
        // master gain (GUI slider, editable mid-flight; 0 = dumb glide for
        // the T2 attitude-authority test)
        public double bellyFlopMaxAoA = 15;
        public double bellyCorrectionFadeAlt = 6000;
        public double starshipCorrectionGain = 1.0;
        // Belly axis roll offset about the long axis (deg), pushed from the
        // core so the belly-attitude error can be logged in Actual.dat
        public double bellyRollOffset = 0;
        // Saturation ceiling for the belly-steer kp: above ~5 the 15deg
        // correction cap is already saturated for any meaningful error, and
        // an unclamped gain goes Inf at the 70km interface (denormal aero
        // forces) which NaN'd the steer command and the vessel (flight 42
        // crash: NaN torque -> NaN orbit -> flung out of the SOI)
        public double bellyGainMaxKp = 50;
        // Fixed flip rotation time (s, design D8): the flip trigger credits
        // flipTime + igniteDelay seconds of unpowered fall on top of the
        // aero-credited landing-burn height. 2.5 s was fantasy - flight 48's
        // real rotation took 24 s (chute-assisted, nose-heavy). 12 was still
        // short: flight 55's lift-assisted nose-up rotation took 28.8 s (and
        // the chute stall timer at flipTime+10 nearly fired mid-rotation).
        // 25 errs high so the trigger fires with room for the real rotation
        public double flipTime = 25;
        // Nose-up flip target lean (user direction after flights 51-53, real
        // Starship profile): the flip pitches the nose to the LOCAL VERTICAL
        // plus a small anti-horizontal-velocity lean ("pull past vertical to
        // a negative angle, then light the engines"), so the burn starts
        // already braking vh and the rotation's body lift does most of the
        // horizontal-speed kill for free. 0.25 ~ 14 deg of lean
        public double flipUprightLean = 0.25;
        // Burn-during-flip + airbrake assist (Janus1992 starship.ks, user
        // approved f62): real Starship lights the engines AT flip start and
        // lets the gimbal assist the rotation; waiting out the full <45deg
        // rotation wastes ~flipTime seconds of unpowered fall and lands
        // short. Light at partial throttle once the nose is clearly swinging
        // up (attitudeError < flipBurnMaxAngle). Airbrakes deploy through
        // the flip: free drag behind the CoG is a pitch-up torque assist
        // (Janus' aft-flaps-full trick), and the suicide-burn height already
        // credits the flip fall. The sim pays the same burn so the predicted
        // cross stays honest
        public double flipBurnThrottle = 0.35;
        public double flipBurnMaxAngle = 100; // deg off the upright steer at which the flip burn lights
        // Belly-flop AoA schedule v3 (deg, flight 46 data): the hull has
        // exactly two aero trims - a controllable low-AoA glide (user
        // hand-held 15 deg from 70 down to 30 km) and a TAIL-FIRST trim it
        // flips into once q exceeds what the flaps can fight (q~8-20 kPa,
        // 30->20 km). There is NO holdable broadside at meaningful q
        // (flight 45 lost 60 deg at q~2.7 kPa, drifting to the trim). So
        // the schedule keys on DYNAMIC PRESSURE, not speed: while q is low
        // (high alt) fly bellyBrakeAoA near-broadside for maximum drag
        // braking (the user's "brake hard while controllable" segment);
        // once q passes bellyQFullGlide, give up the brake and fly
        // bellyGlideAoA - the minimum controllable AoA glide - until the
        // flip. The tail-first arrival then passes the Flip 45-deg gate by
        // itself (both f45 and f46 flipped instantly). Log-q lerp between
        // bellyQFullBrake and bellyQFullGlide. bellyAoACurrent is this
        // tick's scheduled value - Fly passes it to the attitude PD and
        // the sim uses it for the aero force, keeping prediction honest
        public double bellyBrakeAoA = 85;
        public double bellyGlideAoA = 18;
        public double bellyQFullBrake = 300;
        public double bellyQFullGlide = 3000;
        // Terminal belly-down band (f60 user request): once the horizontal
        // airspeed dies, the 18-deg glide reads as a nose-first missile dive
        // and the flip is a ~160-deg fight the flaps lose ("舵面难以翻身").
        // Real Starship ends the fall belly-down horizontal so the flip is
        // ~90 deg with the tail-first trim ASSISTING the second half. Blend
        // the schedule back to bellyBrakeAoA as vh drops below
        // bellyTerminalVh (full) over bellyTerminalVhBlend. f82: 150/75
        // engaged only in the last ~2-3 km, so at 10 km the ship held the
        // 18-deg trim on a steepening -49 deg path and the user had to
        // force the nose up with RCS + flaps by hand ("一定要用RCS和舵面
        // 强行把攻角救回来"). 250/150 starts the pitch-up around 6-8 km
        public double bellyTerminalVh = 250; // m/s horizontal airspeed
        public double bellyTerminalVhBlend = 150;
        // f83: cap the terminal pitch-up AoA. Uncapped it ran to the full
        // 85-deg brake AoA, the ship leaf-fell nearly VERTICAL for 3 km
        // (7.6 -> 4.3 km), the belly frame went degenerate (wind axis
        // straight up) and the hull wallowed 19-43 deg off the command -
        // that wallow is the "万米左右机身偏转" that grew the cross offset
        // 150 -> 935 m. At 45 deg the ship keeps translating (wind axis
        // stays off-vertical, flaps keep authority) while the nose visibly
        // comes up
        public double bellyTerminalMaxAoA = 45; // deg
        // f84: with the nose-above-horizon floor the terminal fall now
        // commands up to the full 85-deg brake AoA again on steep slow
        // paths. The f83 wallow that motivated the 45-deg cap came from
        // STEERING in the degenerate vertical-wind frame, not from the
        // level attitude - so fade the belly impact CORRECTION out as the
        // horizontal airspeed dies (below this there is no directional lift
        // to steer with anyway) instead of capping the attitude
        public double bellyCorrectionVhFull = 80; // m/s: full correction above, faded to 0 at 0
        public double bellyAoACurrent = 90;
        // f114: slew-limited belly AoA command state (the schedule/governor/
        // vh-floor steps excited the pre-flip wag - see the BellyFlop block)
        private double bellyAoASlew = 90;
        private double bellyAoASlewT = -1;
        private const double bellyAoACmdSlewRate = 10; // deg/s
        // f114: lateral ROLL-pulse command (deg, added on top of the GUI-
        // calibrated bellyRollOffset by BoosterGuidanceCore when calling
        // starshipAtt.Update). Zero outside the pulse. Replaces the f106
        // 4-deg frame-yaw pulse: at belly AoA a roll about the long axis
        // rotates the whole lift vector (banked turn) where the yaw only
        // sideslipped - the user's own cross corrections are pure roll
        // taps (f114 Input.dat: inR=+/-1 x ~0.5 s, no yaw)
        public double bellyRollPulseDeg = 0;
        // f83 cleanup (2026-09-01): the starship aim burn is DELETED (default
        // off since f55's knife-edge kill, superseded by TrajCal). TrajCal
        // owns every powered/unpowered impact correction now
        // Passive glide (user request after the f70-f72 prediction failures):
        // fly the pure aero-brake descent - belly AoA schedule vs q, straight
        // trajectory, no impact prediction, no aero calibration, no airbrake
        // or engine overshoot brake, no aim burn. The pilot compares MJ's
        // landing prediction against the unperturbed real trajectory. Flip
        // and landing burn still fly normally (they are not prediction)
        public bool starshipPassiveGlide = false;
        public bool PassiveGlide() { return (recoveryProfile == "starship") && starshipPassiveGlide; }
        // Traj-driven calibration (user request after f76): the calibration
        // source is TRAJECTORIES' predicted impact (read via reflection,
        // TrajAPI) instead of our own prediction sim - "校准不再看我们预测
        // 落地而是以Trajectories的为准". Impact LONG of the target ->
        // airbrakes; impact SHORT -> a modest burn at the belly attitude
        // extends the glide; the lateral part rides the existing belly
        // correction's lateral leash. LIVE ONLY (sim clones never run
        // this); replaces the legacy prediction sim + overshoot brake
        // while on. Combines with passive glide (which then owns nothing)
        public bool starshipTrajCal = false;
        public bool TrajCalActive() { return (recoveryProfile == "starship") && starshipTrajCal; }
        public double trajCalOverError = 800; // m - impact this far LONG deploys the airbrakes (low-alt floor)
        public double trajCalOverAltFrac = 0.2; // deploy threshold scales with altitude: max(overError, frac*alt) - high up Traj's answer is multi-km noise (f77: +-16k swings at 51 km; f78: phantom +33k at 40 km)
        public double trajCalBrakeMaxAlt = 45000; // m - brake ceiling. f77/f78: phantom deploys fired the moment the 40 km gate opened, so the ceiling was 30000. f81 user directive: decisions follow the Trajectories red mark, period - at 40 km the mark showed a real 13-15 km overshoot, auto-boards refused (ceiling), the user deployed manually and over-braked without the auto-release (turned +14k long into -12k short). Ceiling now 45 km; the alt-scaled threshold (9k at 45 km) + 3s persistence remain as spike guards, and the auto-release at 0.5xthresh prevents the over-brake.
        public double trajCalShortError = 1500; // m - impact this far SHORT lights the extend burn
        public double trajCalBurnExitShort = 500; // m - f93: once burning, hold until the shortfall is below this. f93 (1122 m long): trigger and exit shared -trajCalShortError (zero hysteresis), so each 0.1-0.7 s blip parked the (EMA-smoothed) mark exactly on the -1500 edge and the engines strobed ~30x from 17.2 km to 8.3 km without ever driving the mark to target - user: "发动机给到推力太鬼畜了，不能一次到位推到精准的位置". 500 (not ~0) because trajAlong is EMA-smoothed (tau 2 s at 17 km) while the raw mark runs ~765 m/s during a burn: exiting at EMA -500 lands the raw mark near 0. Never exit into positive along - the brakes own the long side.
        public double trajCalBurnMaxQ = 20000; // Pa - was 3000 (copied from the pilot PROGRADE SAS swing, a different maneuver): a belly burn at 18-deg AoA rides the same loads the glide already holds (16.7 kPa proven) and the flip burns engines at 18-24 kPa routinely. f77/f78: the 3000 gate kept the honest short-burn unfireable below ~33 km, exactly where short-falls are decided. f82 user: "校准中动压的限制还可以放宽点" -> 20000 (hysteresis holds to 22000 once burning)
        public double trajCalBurnMaxAlt = 30000; // m - symmetric phantom guard for the burn (q-gate binds lower anyway)
        public double trajCalBurnMinAlt = 8000; // m - finish the burn well above the flip band (~4.4 km)
        public double trajCalBrakeMinAlt = 3000; // m - no board cycling inside the flip band
        public double trajCalBrakeArmTime = 3; // s of sustained over-threshold before the boards deploy (f69 dirArm lesson)
        public double trajCalBrakeRetractHold = 10; // s before the boards may redeploy after a retract (f77: 0.2 s retract/redeploy flip-flop)
        public double trajCalBrakeMinHold = 5; // s minimum deploy hold - shorter than the legacy 10 s: TrajCal's impact walk is fast (~300 m/s at 29 km) and a long hold rides the correction into short (f78 user feedback)
        public double trajCalReleaseFrac = 0.5; // release when along < frac x current deploy threshold (earlier release high up where the walk is fast)
        public double trajCalBurnMaxDv = 150; // m/s budget - the landing reserve owns the rest. f81: 60 was spent in 10 s against a 12 km short (bought only ~1.5 km) - user: "发动机并没有推多久就没推了"
        public double trajCalBurnQHyst = 2000; // Pa - q-gate hysteresis: engage at maxQ, hold to maxQ+hyst once burning. f81: q hovered exactly at the 15000 gate (thrust pushes q over -> cut -> q falls -> refire) and the burn strobed 27x in 10 s, thrashing the attitude
        public double trajCalBurnThrottle = 0.35; // modest: extend the glide, don't re-fly it
        // f85 user request 暴力减速 (violent brake): below trajCalViolentAlt,
        // when the SHIP'S OWN POSITION enters the trajCalViolentDist circle
        // around the target (user correction: "如果当前的飞船位置距离目标只有
        // 1km内了，才开始暴力减速，而不是红标超过1km就减速"), force the AoA
        // to the full brake end (bellyBrakeAoA) regardless of the glide
        // schedule - max broadside drag so the ship drops onto the target
        // instead of gliding past it (f85: terr grew 2.8->9.2 km from
        // 16->3.3 km with the boards out from 13.5 km). "在保证可控的情况下":
        // q-gated well under the measured tail-first trim-flip point (16.7
        // kPa on EnginePlate3), a sustained attitude departure aborts with a
        // cooldown. The trigger is POSITIONAL; the red mark only arms it
        // (along > +300: no point braking when the impact is already on/
        // short of the target) and releases it (along <= 0 = overshoot
        // killed) - the 0..300 deadband prevents release/re-arm strobing
        public bool trajCalViolentBrake = true; // GUI 暴力减速 toggle, pushed by core
        public double trajCalViolentAlt = 10000; // m - violent brake operates only below this
        public double trajCalViolentDist = 2000; // m - LEGACY (f122): the corridor trigger subsumed the ship-circle gate; still persisted/logged, no longer arms anything
        public double trajCalViolentMaxQ = 8000; // Pa - controllability gate (half the measured 16.7 kPa trim-flip)
        // f122 速度走廊 (velocity corridor - see the arm logic in Fly): the
        // trigger is geometric and mark-free:
        //   overRun = vh*tFall - 0.5*vbDriftDecel*tFall^2 - shipDist
        // = how far PAST the target the ship lands if nothing changes
        private const double vbDriftDecel = 1.5; // m/s^2 - natural belly-glide decel credit (f122: ~3.4 measured at the full 85-deg slam, less at the glide AoA; conservative-low so the corridor brakes a touch early, the vh floor protects the bottom)
        private const double vbFlipSail = 8; // s - the flip maneuver carries vh this long (f124: flip+spool took ~8s at ~5.6 m/s^2, then the burn killed only ~2.2)
        private const double vbPostKill = 3; // m/s^2 - post-flip horizontal kill rate (f124 measured 2.2 in the LandingBurn float; conservative-low so the corridor brakes a touch more)
        private const double trajCalViolentOverRun = 600; // m - arm threshold (below this the landing burn + v2g finish it)
        private const double vbOverRunFull = 1500; // m - overRun for FULL brake AoA (proportional between)
        private bool violentBrakeLatched = false;
        private double violentBrakeSinceT = -1; // when the trigger first held (-1 = not holding)
        private double violentBrakeAbortT = -1; // last controllability abort (5 s cooldown)
        private double violentBrakeDepartT = -1; // when the attitude departure started (-1 = tracking)
        // f95 proportional AoA governor (user design, replaces the bang-bang
        // 85-deg slam in the last trajCalAoaModDist metres): f95 showed the
        // violent brake's full-brake hold over-kills - vh collapsed 121->15
        // m/s, the offset bottomed at 266 m at 6.3 km, then the ship was an
        // UNGUIDED leaf (vh < bellyCorrectionVhFull=80 -> correction faded
        // to zero) and drifted to a 1095 m miss. User: "在200m/s左右保持
        // 水平滑翔靠减速板也降不下水平速度,抬升机头更有效;最后5km内落点
        // 过冲就自动抬升机头直到落点准确再改回水平滑行,反复如此;水平速度
        // 大过冲则加大攻角,水平速度小则保持水平". So: below
        // trajCalAoaModDist, AoA = schedule + gain*max(0,trajAlong), slew-
        // limited, capped at bellyBrakeAoA. While the governor holds the
        // AoA above the schedule, the belly correction is SUSPENDED (user:
        // "抬升机头减速阶段不要有任何横向修正,不然会出现之前的自旋情况,
        // 滑行的时候正常横向修正就行") - the governor IS the along-track
        // corrector in that regime, and frame-tipping at high AoA feeds
        // the f94 spin loop.
        public bool trajCalAoaMod = true; // GUI 比例抬头减速 toggle, pushed by core
        public double trajCalAoaModDist = 5000; // m HORIZONTAL ship-to-target (user: 最后5km指水平距离不是高度) - the governor owns the overshoot inside this circle
        public double trajCalAoaModGain = 0.01; // deg of added AoA per metre of overshoot (1000 m -> +10 deg)
        public double trajCalAoaModSlew = 10; // deg/s - the AoA command slew limit (f94 first abort: fast command sweep on a marginal attitude)
        private double aoaModAoA = -1; // slew-limited governor output (-1 = disengaged, following the schedule)
        public bool aoaModBraking = false; // governor holding AoA above the schedule this tick - the steer branch reads this to suspend the correction
        // f107 vh floor (user picked option A): below 10km the horizontal
        // speed IS the ship's reach - once killed it can never be bought
        // back (bigTrim has an 8km floor, tcBurn is gated above 45deg AoA,
        // the terminal schedule runs ~85deg). f107: the governor saturated
        // to full 85deg broadside on the +4.9km long-biased mark (gain
        // 0.010 x 4959 = +50deg -> cap), vh went 132->21 by 5.6km, ship was
        // 2.4km out with ~900m of reach left = mathematically short, mark
        // still read +200 - landed 2km short. While vhFloorActive the
        // governor may not engage and the AoA is capped (see below).
        private bool vhFloorActive = false;
        // f115: the high-band kill floor exists because the capped low-AoA
        // bands below aoaRampTopAlt can only finish off a small residual -
        // the kill has to happen before running out of full-authority
        // altitude
        // f120: the residual is distance-aware (see the kill floor below);
        // f121 made it SIGNED (projection of the remaining distance onto the
        // velocity direction) with a 0 floor - receding/overflown = full
        // brake. vhKillTau = effective closing time the sub-4km band actually
        // delivers per m/s of arrival speed, measured on two f120 flights:
        // arriving at 50 m/s covered ~1.15-1.2 km under the 7-deg cap + the
        // f89 floor's continuing decay (23-26 s both times). vhKillResidCap
        // bounds the keep-demand so an absurdly far target can't veto braking
        private const double vhKillTau = 25;
        private const double vhKillResidCap = 300;
        // f126: tBand collapses toward 0 when the burn STARTS right at the
        // 4 km gate (light 3.75m booster, no airbrakes, burn start 4.1 km)
        // and the band term (vhNow-vhRes)/tBand exploded to 65 m/s^2 ->
        // thr=1.00 spike while the hull was still 60 deg off the steer.
        // Nothing can be killed in under ~4 s anyway; the band below the
        // gate owns whatever residual remains
        private const double vhKillTBandMin = 4;
        // f131 (user: 精度好但有点废燃料,小误差不开火行不行 - picked 方案A
        // 死区): f131 logs show the floor's small-error raises are aLatReq
        // 0.1-2.5 m/s^2 (thr 0.04-0.33 flickering through the whole coast)
        // while genuine corrections demand 6-42 - the threshold sits between
        // the populations. Skipped residuals (vh excess ~1.5*tBand ~ 6 m/s
        // worst case) are absorbed free by the terminal v2g under the final
        // burn's high throttle. f128's dead-zone fix stays alive: vh=20 at
        // the 4km line demands 3.25 > deadband = still fires; f123's fast
        // drift (aLatStop 4.5-32) also always clears it
        private const double vhKillDeadband = 1.5;
        // f132: release side of the deadband hysteresis (engage 1.5 / release
        // 0.6) - see the floor-block comment; kills the 5 Hz bang-bang that
        // halved lateral authority and put the 3.75m into the launch tower
        private const double vhKillDeadbandRelease = 0.6;
        // f127: horizontal-kill-aware ignition - m of extra ignition height
        // per m/s of horizontal speed, applied ONLY when the natural
        // (vertical-energy) burn height sits below the low-AoA-cap line
        // (light boosters). vh=117 -> ignite ~6.9 km instead of 3.3 km so
        // the higher-cap band above the line has ~6 s to kill ~80 m/s
        private const double vhIgniteGain = 25;
        // f104: the f85 experimental high-altitude trim burn (>55km) is REMOVED
        // at the user's request ("5w5米以上的实验性发动机调控功能去掉吧不好用").
        // High altitude is exactly the phantom-mark regime (red mark swings
        // +/-40-150km with zero thrust, f86/f101/f102/f103) so an engine trim
        // driven by it could only chase phantoms; the f97 bigTrim below keeps
        // the high-alt RETRO case, which is the safe failure direction.
        // The reversal-cooldown constant survives because the big trim uses it.
        public double trajCalBigTrimFlipCd = 10; // s cooldown before a direction reversal may start (f104: renamed - it now serves the big trim only)
        // f97 BIG-error high-alt correction burn (user request: "高空如果误差
        // 大过30km,可以尝试使用类似猎鹰九模式的引擎反推来调控落点...高空油门
        // 非常敏感所以一定要限制阀门不能大于10%,并且一定要朝向调控准确之后再
        // 启动油门"). Why it exists: ignored, the high-alt error really is
        // 30-70km (f95/f96) and the low-alt brakes cannot recover that; high
        // up a small dV moves the impact a LOT (~200m per m/s at 40km in
        // f96), so this is the most fuel-efficient place to fix a big error.
        // This is the BIG sibling of the f85 fine trim (which refuses
        // |along|>30km as not-a-fine-trim-job). Differences: gentle 10%
        // throttle cap, and it RELEASES at trajCalBigTrimExit (10km, user:
        // 追到10km) instead of driving to zero - chasing the swinging
        // high-alt mark all the way to zero is exactly the f96 fuel-killer,
        // so the last ~10km belongs to the honest <30km regime and its own
        // systems. Point-first is automatic: the swing flies the pilot-burn path and Core's 10-deg
        // steer gate holds mainThrottle at 0 until the nose is on +/-vel_air.
        // Fuel floor: never engages/continues below 1.25x the landing reserve
        public bool trajCalBigTrim = true; // GUI 高空大偏差反推 toggle, pushed by core
        public double trajCalBigTrimMinAlt = 30000; // m - below this the mark is honest and the low-alt systems own it
        public double trajCalBigTrimError = 30000; // m - engage when |along| beyond this
        public double trajCalBigTrimExit = 10000; // m - release inside this (user: 追到10km) - still short of chasing the swinging mark to zero (f96), the honest <30km regime owns the rest
        public double trajCalBigTrimMaxQ = 8000; // Pa retro gate - tail-first is the natural trim (same scale as the old overshoot brake); prograde keeps pilotBurnProgradeMaxQ
        // f98: the fixed 10% cap is GONE (user: "高空反推的油门限制可能太大
        // 了,你还参考mj那种平滑的点火吧,把限制去掉") - the throttle is sized
        // MechJeb-style from the dV the mark error needs (along-excess over
        // the online leverage) delivered over ~tau seconds: full thrust on a
        // big error, tapering smoothly to zero at the release band. This
        // field survives as the max cap only.
        // f100: cap back at 30% (user: "还是加一个30%的油门限制吧") - at
        // 0.87-1.0 the burns were too violent for the knife-edge mark regime
        // above ~50km (the mark swings +/-40-150km on its own there; f100
        // burned the whole 300 m/s budget chasing it and the user reverted)
        public double trajCalBigTrimThrottle = 0.30; // max throttle cap for the big trim burn
        public double trajCalBigTrimTau = 8; // s - MJ-style taper: the burn aims to deliver the needed dV over about this long
        public double trajCalBigTrimMaxDv = 300; // m/s budget per enable - f96 burn #2 moved 72k->13.5k for ~280 m/s, so 300 covers one big fix
        public double dvAvailable = -1; // m/s, pushed by core every tick (f96 fuel watch); -1 = not computed yet
        public double landingReserveDv = 350; // m/s, pushed by core every tick
        private int bigTrimDir = 0; // 0 = off, +1 = prograde, -1 = retro
        private double bigTrimSpentDv = 0;
        private double bigTrimFlipT = -100;
        private double bigTrimStartAlong = 0;
        private double bigTrimAbortT = -100;
        // f99: the runaway guard fired twice on PHANTOM mark motion during
        // the swing-in (spentDv=0.0 both times - no thrust had been
        // delivered yet, the mark swings multi-km on its own in the
        // knife-edge zone). Arm the guard only after 30 m/s of REAL
        // delivered dV, re-baseline the reference at arming (the swing-in
        // drift is not the burn's fault), and require the wrong-way motion
        // to persist 3 s (a single phantom refresh spike is not a runaway)
        private bool bigTrimArmed = false;
        private double bigTrimRunawayT = -1;
        // f107 decisive burn (user technique, approved together with the
        // 40km retro ceiling): a short-mark burn that STARTS in the honest
        // zone (y <= trajCalBigTrimMinAlt) keeps the BELLY/glide steer and
        // burns sustained at up to full throttle until along > -5km,
        // instead of swinging nose-first in 2s 30% pulses. f107 measured
        // the pulses at ~15 m/s total where ~150 was needed (mark -18.9km
        // at 20km kept walking back through the pulses, looking like
        // prograde was hurting); the user's manual save held ~20deg AoA,
        // firewalled 160 m/s in 5.4s, mark back to target, ship ballooned
        // +1.5km storing the energy as altitude. Chosen ONCE at burn start
        // so a balloon above 30km mid-burn does not flip the style.
        private bool bigTrimBellyBurn = false;
        private bool coneGuard = false; // f113 falcon cone guard: full-budget v2g takeover when the dive will overshoot the pad
        private double attErrPrevTick = 0; // f126: last tick's true tracking error, captured before the per-tick zeroing - gates the throttle floors (attitudeError itself is already zeroed where the floors run)
        private double bigTrimLeverage = 200; // m of along-track mark shift per m/s of dV - seeded from f98 (37010 m / 191.3 m/s = 194), tracked online while burning
        private double lastBigTrimDiagT = -100; // f98 gate diagnostic rate limit
        public double trajImpactMaxAge = 30; // s - older impact data = calibration off (passive behavior)
        private double trajCalSpentDv = 0;
        private Vector3d trajImpact = Vector3d.zero; // last good impact, body-rel frame at trajImpactT
        private double trajImpactT = -1;
        private bool trajImpactValid = false;
        private bool trajAlwaysUpdateSet = false;
        // f136 dual-prediction logging (user-picked option C): Trajectories'
        // own predicted impact is appended to every Actual.dat row (traj_x
        // traj_z, target-frame meters) so BG-sim vs Traj can be reconciled
        // after EVERY flight instead of argued about. Logging only - the
        // control path never reads these. NaN when Traj is absent or has
        // no trajectory
        private Vector3d trajLogImpact = Vector3d.zero; // body-rel frame at trajLogImpactT
        private double trajLogImpactT = -1;
        private bool trajLogForceDone = false;
        private Vector3d trajErrSmooth = Vector3d.zero; // altitude-smoothed impact error vector (f77: raw swings drove a 95-deg attitude chase)
        private bool trajErrSmoothInit = false;
        private double trajAlong = 0; // smoothed along-track error (+ = impact LONG), m
        private double trajCross = 0; // smoothed cross-track error magnitude, m
        // Raw (unsmoothed) Traj impact error magnitude - exactly what the red
        // map mark is doing right now. The PANEL displays this (user f83:
        // 面板落点误差以红十字为准, not the old own-sim number); the steering
        // keeps the EMA-smoothed value above (f77: raw swings drove a 95-deg
        // attitude chase, that stays)
        private double trajErrRawMag = 0;
        private bool trajErrRawValid = false;
        private double trajCalOverSinceT = -1; // when the over-threshold condition first held (-1 = not holding)
        private double trajCalRetractT = -1; // last retract time (-1 = never)
        private bool trajBoardsExtend = false; // boards-extend-glide suppression with hysteresis (engage >1.35x, release <1.25x)
        private bool trajCalBurnWasOn = false;
        // f83 cleanup: ALL aim-burn fields deleted with the burn itself
        // Prediction diagnostics (flight 55: displayed terr stayed 31-96k
        // while same-state phase-transition sims showed 530k - log on change
        // whether the prediction sim timed out)
        private bool predDiagTimeout = false;
        private double predDiagLastLogT = -10;
        // Prediction wall-clock throttle (flight 58): this sim runs inside
        // GetControlOutputs = once per Fly FRAME, and the starship glide at
        // PredictionMaxT=1500 takes 60-110 ms wall. Untrottled it collapsed
        // the frame rate to 10-15 fps, and the belly cascade went unstable
        // at dt~0.1 s (TORQUE diag: bellyErr swinging 0-176 deg with cmd
        // slamming +-0.7/1.0). Cap the sim's CPU share: the next run waits
        // lastDuration/predCpuFraction seconds and steering reuses the stale
        // prediction in between - predBodyRelPos is body-rotation-compensated
        // so staleness is pure trajectory drift (2 s at vh 2000 = 4 km, fine
        // for the gentle belly correction)
        private static readonly System.Diagnostics.Stopwatch predClock = System.Diagnostics.Stopwatch.StartNew();
        private double predWallT = -1;    // predClock time of the last run
        private double predWallDur = 0.05; // wall seconds the last sim took
        public double predCpuFraction = 0.12; // max share of wall time for the sim
        // Trim flip (flight 46 proved, flight 51 confirmed): above
        // bellyTrimFlipQ the hull's tail-first trim beats the flaps, the
        // belly glide is lost and the ship lands SHORT of the glide
        // prediction. Live: detected via belly att_err stuck >100 deg for 5 s
        // (glideLost -> pure retrograde hold, stop saturating the flaps).
        // Sim: EulerStep latches glideLost at the q threshold so the
        // cross/path honestly show the short landing. Flight 57: the
        // EnginePlate3 craft HELD the belly glide through q=16.7 kPa at
        // 24 km (vr -53..-80 down to 4 km) while the sim dove ballistic at
        // 8 kPa (vr -153 at 28 km, -187 at 26 km) -> prediction short-biased
        // -> correction homed the wrong point -> +115.7 km overshoot, pure
        // along-track. Raising the default: a too-HIGH threshold self-corrects
        // (live att_err>100/5s detection latches a real flip and poisons only
        // the remaining sims); a too-LOW one poisons the whole flight. The
        // 星舰货运 cargo variant genuinely flips at 8.2-8.8 kPa - reflying it
        // needs this lowered again.
        public double bellyTrimFlipQ = 25000; // Pa, flight 57 (EnginePlate3 held 16.7 kPa)
        public bool glideLost = false;
        private double bellyDepartT = -1;
        private bool bellyAttAcquired = false; // f102: glideLost arms only AFTER the belly attitude has been acquired once - enabling guidance mid-swing (164 deg off for 3.1 s at enable, both f102 flights) latched glideLost during the swing-in and silenced the big-trim burn until the user toggled guidance
        private double bellyReacquireT = -1;      // f108: glideLost auto-unlatch timer (belly error <45 sustained)
        private double pilotBurnGraceUntil = -1;  // f108: no glideLost latch within 15 s after a pilot burn ends (the swing-back sits >100 deg for ~10 s at 33 km)
        // Last-resort stock parachutes: flip stalled past flipTime+10 s, or
        // landing-burn attitude lost low and fast -> deploy rather than
        // crater (user request after flight 51)
        public bool parachuteBackup = true;
        private bool chutesDeployed = false;
        private double flipStartT = -1;
        // f83 cleanup: the legacy overshoot brake (trend/direction-armed
        // airbrakes + f66b positional engine brake) is DELETED - dead code
        // under TrajCal and its own-sim prediction input is gone
        public bool bellyBraking = false; // EulerStep reads this to fly tail-first aero during the burn
        // Airbrake overshoot control (f60 user request): deploying the stock
        // airbrakes (ModuleAeroSurface internal-drag parts) is FREE drag.
        // Driven by the TrajCal block below; BoosterGuidanceCore reads this
        // and drives deploy
        public bool airbrakeWanted = false;
        private bool airbrakeLatched = false; // hysteresis latch so prediction noise cannot strobe the brakes (f62: armed->retracted in 12 s)
        private double airbrakeLatchedT = -100;
        // Pilot burn attitude (f62 user redesign - two GUI buttons, 顺向
        // prograde / 逆向 retro): the pilot picks the burn direction
        // EXPLICITLY instead of the controller inferring it from the
        // predicted error sign (the f61 auto-direction asked the pilot to
        // trust a noisy prediction they cannot see). While a button is on,
        // the ship swings onto +/-vel_air via the SAS path IMMEDIATELY - it
        // is already aligned when the pilot then adds throttle - and
        // switching the button off returns to the belly-forward guidance
        // attitude on its own. The belly hold otherwise locks AoA at ~85deg,
        // making pilot thrust perpendicular to the flight path. Fly blocks
        // the throttle passthrough unless this accepted the attitude
        // (pilotBurnAttitude), so a refused prograde is inert rather than a
        // perpendicular shove
        public float pilotThrottle = 0; // live only, handed in by Fly every tick
        public bool pilotBurnAttitude = false; // true = SAS flies +/-vel_air this tick
        public string pilotRefuseWhy = ""; // f85: WHY a pressed 顺向/逆向 button is refused ("" = accepted or no button); shown in the Core refusal log/message so a latched gate (glideLost, guidance burn) is not indistinguishable from "button off"
        public int swingDir = 0; // +1 = swinging nose-prograde, -1 = nose-retro, 0 = not swinging - set with pilotBurnAttitude so Core feedback can name the direction for BOTH manual buttons and the f85 auto high trim
        public int pilotAttitudeMode = 0; // 0 = guidance attitude, +1 = 顺向 prograde button, -1 = 逆向 retro button; written by the GUI via core, live only (never persisted, cleared on enable)
        public double pilotBurnProgradeMaxQ = 20000; // Pa - above this a nose-first burn is refused. f66: 1500 refused the user's range-extension burn at q=1502-1645; 3000 then refused the f103 顺向 button EIGHT times at q=3033-19330 while the ship glided 19.5km short ("动压限制太敏感了,发动机根本没办法正常工作"). Raised to the tcBurn/flip-burn level: flip burns run engines at 18-24 kPa routinely (f82 comment), and the f102 low-alt auto prograde already rides this same 20 kPa gate. A nose-first BURNING ship has gimbal authority like any launcher through max-q; the swing transient through broadside is the real risk, so don't raise further without flight data
        private double pilotBurnRefuseLogT = -100;
        // Live aero calibration (flight 59): the stock cache says the 18-deg
        // glide at 5-10kPa makes ~4.5-5 m/s2 of lift; the real ship holding
        // the same schedule (att_err<2deg, flaps active) realized 8-10.
        // Measure the realized aero accel each live glide tick (thrust=0, so
        // dV/dt - g IS the aero force), ratio it against the cache model at
        // the same state and scheduled AoA, EMA it, and let the prediction
        // sim scale lift/drag by the measured factors (Simulate.EulerStep).
        // Self-adapts to per-craft trim differences with no tuning. Glide
        // regime only (scheduled AoA <= 45deg): the 85-deg brake band matches
        // the cache fine and sampling there would dilute the glide correction
        public bool aeroLiveCal = true;
        public double aeroCalLift = 1; // realized/model lift ratio (EMA)
        public double aeroCalDrag = 1; // realized/model drag ratio (EMA)
        public double aeroCalRate = 0.02; // EMA gain per tick (~1s at 50Hz)
        // f64: a single kL/kD cannot fit the whole descent - the measured
        // truth walked 1.53 (q~2kPa) -> 0.9 (q~15kPa) -> 2.33 (low alt,
        // brakes out) in ONE flight, every EMA update moved the predicted
        // impact ~10km, and the steering chased each move sideways
        // (cross-track 0.6km -> 46km). Calibrate per dynamic-pressure band
        // and let the sim apply the band of the step it is integrating;
        // bands with too few samples fall back to the global EMA
        public double[] aeroCalLiftQ = new double[3] { 1, 1, 1 };
        public double[] aeroCalDragQ = new double[3] { 1, 1, 1 };
        public int[] aeroCalSamplesQ = new int[3] { 0, 0, 0 };
        // f67: deployed airbrakes are NOT lift-neutral on every hull - on
        // 星舰货运 the boards nearly DOUBLED measured lift (kL 1.58 -> 3.0
        // at q~2-3kPa with bErr=2deg, so no attitude artifact), the
        // braking descent landed 52km LONG, and the boards-out samples
        // blended into the shared bands poisoned the saved seed for the
        // next unbraked flight. Boards-out lift is a different
        // calibration than boards-in: keep it in its own band set.
        // (Drag keeps the brakes-out gate entirely - the boards' drag
        // lies exactly on the drag axis and cannot be split out)
        public double[] aeroCalLiftAbQ = new double[3] { 1, 1, 1 };
        public int[] aeroCalSamplesAbQ = new int[3] { 0, 0, 0 };
        // Set on SIM CLONES from the live airbrakeWanted (copy ctor): the
        // boards stay out for the rest of a braking descent, so a braking
        // prediction must integrate the boards-out lift bands it is
        // currently measuring - that self-consistency is what made f67's
        // terr converge 2.4k -> 53k onto the real 52.4km overshoot
        public bool simBrakesOut = false;
        // Boards only help an overshoot when they do NOT extend the glide:
        // when the boards-out band for the current q shows lift over
        // airbrakeMaxLiftRatio x the boards-in band, deploying is a range
        // EXTENDER on this hull (f67: ratio 1.9 at q<4kPa) - do not arm
        public double airbrakeMaxLiftRatio = 1.3;
        public int calBinMinSamples = 20; // samples before a band stands on its own
        public static readonly double[] AeroCalQEdges = new double[2] { 4000, 10000 }; // Pa: band 0 thin, 1 mid, 2 dense
        private bool calBinGuardLogged = false;
        // f64 hotfix: the live game threw IndexOutOfRange inside Effective*
        // 2188x in one session (exception flood = the "非常卡") although the
        // IL and every writer produce length-3 arrays - root cause still
        // unknown, so guard, fall back to the global EMA, and log the REAL
        // state once so the next log tells us what was actually short
        private bool CalBinsBad(string who, int b)
        {
            bool bad = (AeroCalQEdges == null) || (AeroCalQEdges.Length < 2)
                || (aeroCalLiftQ == null) || (aeroCalLiftQ.Length != 3)
                || (aeroCalDragQ == null) || (aeroCalDragQ.Length != 3)
                || (aeroCalSamplesQ == null) || (aeroCalSamplesQ.Length != 3)
                || (aeroCalLiftAbQ == null) || (aeroCalLiftAbQ.Length != 3)
                || (aeroCalSamplesAbQ == null) || (aeroCalSamplesAbQ.Length != 3)
                || (b < -1) || (b > 2); // -1 = "no bin" sentinel from the log caller
            if (bad && (!calBinGuardLogged))
            {
                calBinGuardLogged = true;
                Log.Info(string.Format("[AeroCal] BIN-GUARD {0}: b={1} edges={2} liftQ={3} dragQ={4} samplesQ={5} liftAbQ={6} samplesAbQ={7} minS={8}",
                    who, b,
                    (AeroCalQEdges == null) ? -1 : AeroCalQEdges.Length,
                    (aeroCalLiftQ == null) ? -1 : aeroCalLiftQ.Length,
                    (aeroCalDragQ == null) ? -1 : aeroCalDragQ.Length,
                    (aeroCalSamplesQ == null) ? -1 : aeroCalSamplesQ.Length,
                    (aeroCalLiftAbQ == null) ? -1 : aeroCalLiftAbQ.Length,
                    (aeroCalSamplesAbQ == null) ? -1 : aeroCalSamplesAbQ.Length,
                    calBinMinSamples));
                // f65: the guard DID trip on a live controller (len-0 arrays on
                // the scene-load scope instance) and no writer in the source
                // can produce that - capture the call path next time
                Log.Info("[AeroCal] BIN-GUARD stack: " + System.Environment.StackTrace);
            }
            return bad;
        }
        public int AeroCalBin(double q)
        {
            if ((AeroCalQEdges == null) || (AeroCalQEdges.Length < 2))
                return 2; // guard state; Effective* logs it
            if (q < AeroCalQEdges[0]) return 0;
            if (q < AeroCalQEdges[1]) return 1;
            return 2;
        }
        public double EffectiveCalLift(double q)
        {
            int b = AeroCalBin(q);
            if (CalBinsBad("lift", b))
                return aeroCalLift;
            // Boards-out prediction (sim clones with simBrakesOut) uses the
            // boards-out bands once they stand on their own; thin boards-out
            // bands fall back to the boards-in band, then the global EMA
            if ((simBrakesOut) && (aeroCalSamplesAbQ[b] >= calBinMinSamples))
                return aeroCalLiftAbQ[b];
            if (aeroCalSamplesQ[b] >= calBinMinSamples)
                return aeroCalLiftQ[b];
            return aeroCalLift;
        }
        public double EffectiveCalDrag(double q)
        {
            int b = AeroCalBin(q);
            if (CalBinsBad("drag", b))
                return aeroCalDrag;
            if (aeroCalSamplesQ[b] >= calBinMinSamples)
                return aeroCalDragQ[b];
            return aeroCalDrag;
        }
        private Vector3d calPrevV = Vector3d.zero;
        private double calPrevT = -1;
        private double calLastLogT = -100;
        public int calSamples = 0; // public: the core rate-limits PluginData saves on its growth (f63 revert-proof persistence)
        private double prevThrottleOut = 0;
        // Belly-tracking error captured at the pre-log assignment (live only).
        // attitudeError at method END is nose-vs-steer (~160 deg in belly
        // flight), so the cal gate must NOT read that overwritten value.
        // f112: public so Core's throttle steer-gate can use it for
        // belly-attitude burns (nose-vs-steer is ~160 deg by construction
        // there, so the old nose gate held every belly burn at zero throttle)
        public double bellyAttErrLive = 0; // live belly-attitude error, deg
        // Emergency land-anywhere mode (GUI 应急着陆 button): the target
        // anchor follows the vessel (1 Hz refresh, ground altitude under the
        // ship) and the impact error is replaced by a horizontal-velocity
        // braking demand - error = vh * emergencyBrakeTau, positive sign so
        // every downstream correction accelerates AGAINST the sideways
        // motion and fades to zero as the drift stops. What remains is
        // attitude hold + the shared suicide burn: land vertically, wherever
        // the ship happens to be. Targeting, the flip trigger and the burn
        // height logic are all untouched (they run off the moving anchor)
        public bool emergencyLanding = false;
        public double emergencyBrakeTau = 3;
        private double lastEmergencyAnchor = -10;
        private double lastVhFloorLogT = -100; // f89 vh-floor log rate limiter
        // f117: the f115 kill-floor needs its OWN limiter - sharing
        // lastVhFloorLogT meant the f89 floor logged every 5 s through every
        // burn, so the kill floor's log was shadowed forever and f117's
        // diagnosis had to prove a negative from silence
        private double lastVhKillLogT = -100;
        // f132 hysteresis latch for the vh-kill deadband (real flight only -
        // sim ticks must not mutate it); cleared when the floor gate or the
        // vh>res demand condition drops
        private bool vhKillLatched = false;
        // f92 coast-translation floor: the suicide law manages VERTICAL speed
        // only, so the mid-burn coast (vy under the profile -> throttle ~0)
        // has zero translation authority - the steer tilts but there is no
        // thrust behind it, and the residual offset waits for the dense low
        // band where the hull's shuttlecock restoring force eats most of the
        // tilt (f90: 16deg tilt for ~30m net below 650m; f92: 1800m of coast
        // at ax=0 carrying a ~300m residual to touchdown). While the suicide
        // law wants LESS than coastTransThrottle, high up, with real position
        // error left and time to spend, hold a small throttle so the tilt
        // actually translates - thin air is where translation is cheapest.
        public double coastTransThrottle = 0.2;  // throttle floor (0 disables)
        public double coastTransMinAlt = 800;    // m - stay above the dense-air aero-fight band
        public double coastTransMinErr = 100;    // m of real position error that justifies the floor
        private double lastCoastTransLogT = -100;
        // f96 give-up: the floor burned 0.2 throttle for 80+ s while posErr
        // sat at ~1250 m (out-gunned at 32 t - the tilt accel could not beat
        // the drift), ~0.44 t of landing fuel for nothing, then the tanks
        // ran dry at 1.6 km. If 15 s of floor does not shrink the error by
        // at least 10%, stop flooring for the rest of this landing
        private double coastTransStartT = -1;
        private double coastTransStartErr = 0;
        private bool coastTransGiveUp = false;
        private PIDclamp pid_belly = new PIDclamp("bellySteer", 1, 0, 0, 15);
        // f64 lateral leash: the belly correction's cross-track component
        // positive-feedbacks in flight (the broadside plate's lateral force
        // response does not follow the along-track sign fix) - it dragged
        // the ship 0.6km -> 46km sideways in 4.5 min while the unsteered
        // natural drift is only ~1.5 m/s. Cross-track error below the
        // deadband is ignored outright; above it, only a tenth-strength
        // nudge is applied.
        // f87: the 3000m deadband never engaged in the TrajCal era (cross
        // sat at ~500-600m the whole descent, user: "横向的误差好像没有修正")
        // - deadband lowered to 500m, but with a runaway watchdog: if the
        // cross-track error GROWS >2km past its engagement value while the
        // leash is active, the f64 feedback is back - cut the lateral
        // component for the rest of the flight ([TrajCal] LATERAL logs)
        // f90/f91: with the f88 sign fix verified (cross pinned AT the 500m
        // deadband all glide, no runaway), the deadband IS the delivered
        // cross accuracy - the correction parks the ship at its edge. The
        // belly phase has minutes of big-aero-authority flight to spend, so
        // tighten to 100m (watchdog unchanged). Matches real practice: the
        // cross-range work belongs to the belly flop, not the landing burn.
        public double bellyLateralGain = 0.1;
        public double bellyLateralDeadband = 100; // m
        // f98: the cross-track error gets its OWN tilt term (bellyLateralGain
        // per 100m, capped here) instead of a leashed share of the error
        // vector - inside errSteer a big along-track error swamped it
        // geometrically (0.1*0.7km cross vs 47km along = a 0.09-degree tilt,
        // the f98 user saw no lateral correction all glide). It yaws the
        // lift vector sideways and does not eat the pitch-plane AoA budget
        public double bellyLateralMaxDeg = 4; // deg cap on the independent lateral tilt (SUPERSEDED f114: the lateral pulse now rolls - see bellyLateralRollDeg)
        // f114: commanded bank bump for the lateral roll pulse (deg of
        // rollOffset about the long axis). 20 deg at the PD's 20 deg/s rate
        // cap = ~1 s roll in, brief hold, roll back on release - the user's
        // tap/coast rhythm (their manual taps are ~0.5 s FULL deflection,
        // but they fly without the rate cap)
        public double bellyLateralRollDeg = 20;
        // f118 phantom-zone gate (user: "高层大气稀薄,这样校准没用" - the
        // flight pulsed at max cadence every 9.5 s for 2.5 min straight,
        // 52->36 km, while the cross reading swung 592->1050->115->976:
        // up there the Traj mark is phantom-grade noise (same regime as the
        // +/-40-150 km along swings) AND the thin air gives a 1.5 s roll tap
        // almost no cross dV - pure twitch (抽搐) for nothing. Below the
        // honest-zone line (same 30 km as tcBurnMaxAlt/the big-trim honest
        // band) the mark is readable and the plate has bite - f107's pulses
        // took cross 520->5 m there
        public double bellyLateralMaxAlt = 30000; // m - no roll pulses above
        private double lateralEngageCross = -1; // cross magnitude when the leash engaged (-1 = disengaged)
        private bool lateralAbort = false; // runaway watchdog: lateral correction cut for this enable
        // f106 user-technique pulse lateral (replaces the f98 continuous
        // tilt - user: "你应该学习我1w米左右如何解决横向偏移"): the user
        // corrects cross with discrete full-deflection roll TAPS - tilt,
        // let the hull answer, hands off, re-read the mark only after it
        // settles. The continuous tilt sat in the loop every tick fighting
        // the hull's aero feedback (f88 runaway family). Now: cross past
        // the deadband + cooldown expired -> one fixed side-tilt pulse
        // (bellyLateralMaxDeg for lateralPulseTime s), then zero demand
        // until the cooldown lets the mark settle (Traj refreshes ~1Hz)
        private double lateralPulseUntil = -1;     // mission t when the active pulse demand ends
        private double lateralCooldownUntil = -1;  // mission t when the next pulse may fire
        private double lateralRollSign = 0;        // f114: +/-1 roll sense frozen at pulse start - the mark rotates as the ship yaws
        private const double lateralPulseTime = 1.5; // s of tilt demand (user's taps: 1-2s full deflection)
        private const double lateralPulseGap = 8;    // s hands-off after a pulse (user: 5-10s between corrections)

        // Private parameters
        private double minError = double.MaxValue;
        // Smoothed target error and sustained-growth timer for the boostback
        // cutoff: the raw prediction can jump >1km in a single tick near the
        // cutoff (flight 17: 883 -> 1960m in 0.1s), and min-of-noisy-series
        // biases the 1.5x threshold tight, so the burn was being cut while
        // still converging ~30m/0.1s. Track the minimum of the SMOOTHED
        // error and require growth to persist before giving up
        private double smError = -1;
        private double boostbackGrowTime = 0;
        private double logStartTime;
        private double logLastTime = 0;
        private Transform logTransform;
        private const float deg2rad = Mathf.PI / 180;
        private float aeroMult = 3;
        private float aeroThrustPropMargin = 0.3f; // full steer gain only when one of FAero/FThrust is clearly dominant
        private float aeroThrustBlendEnd = 0.6f; // steer gain fades linearly to zero as the force ratio reaches this

        // Attitude history for numeric lateral angular rate (rate damping)
        private Vector3d attPrev = Vector3d.zero;
        private bool attPrevValid = false;

        private Trajectories.VesselAerodynamicModel aeroModel = null;
        private double landingBurnAMax = 100; // amax when landing burn alt computed (so we can recalc if needed)
        private bool setLandingEnginesDone = false;
        private bool noSteerReported = false;
        private bool uprightReported = false;
        private bool uprightLatched = false;
        private bool actualRowLogged = false;
        private string lastSettingsLine = "";

        // Outputs
        public Vector3d predBodyRelPos = Vector3d.zero;
        public BLControllerPhase phase = BLControllerPhase.Unset;
        public List<ModuleEngines> landingBurnEngines = null;
        public double steerGain = 0;
        public double elapsed_secs = 0;
        // Propellant (kg) the last full-return simulation burned in each phase
        // (written by Simulate.ToGround, used for the ignition-point fuel hint)
        public double simFuelBoostbackKg = 0;
        public double simFuelReentryKg = 0;
        public double simFuelLandingKg = 0;
        // Surface-relative speed (m/s) the last simulation carried when first
        // descending past 2km above the target (-1 if never reached). Written
        // by Simulate.ToGround; the return-fuel estimate floors its landing
        // reserve at this + margin because the zero-steer estimate sim's own
        // landing-burn accounting is unreliable near the ground (it returned
        // ~44kg for a burn that actually took 18.2t - flight 29)
        public double simLowSpeed = -1;

        // Cache previous values - only calculate new at log interval
        // -1 = no tick yet: live t is vessel.missionTime (seconds since
        // launch), so a fresh controller's first dt would be the whole
        // mission elapsed time without the guard in GetControlOutputs
        // (flight 50: late in-atmosphere enable -> one AimBurn tick of
        // spentDv += a*dt blew the entire 150 m/s budget and latched the
        // burn off for the rest of the flight)
        private double lastt = -1;
        private Vector3d last_vel_air = Vector3d.zero;

        public BLController()
        {
        }

        ~BLController()
        {
            // NOTE: do NOT call StopLogging() here. Simulation copies of this controller
            // are created every guidance tick and garbage collected at arbitrary times;
            // their finalizers must not shut down the global flight log.
            BoosterGuidanceCore.controllers.Remove(this);
        }

        public BLController(BLController v) : base()
        {
            phase = v.phase;
            vessel = v.vessel;
            tgtLatitude = v.tgtLatitude;
            tgtLongitude = v.tgtLongitude;
            tgtAlt = v.tgtAlt;
            lowestY = v.lowestY;
            reentryBurnAlt = v.reentryBurnAlt;
            reentryBurnMaxAoA = v.reentryBurnMaxAoA;
            reentryBurnSteerKp = v.reentryBurnSteerKp;
            reentryBurnTargetSpeed = v.reentryBurnTargetSpeed;
            aeroModel = v.aeroModel;
            aeroDescentSteerKp = v.aeroDescentSteerKp;
            landingBurnHeight = v.landingBurnHeight;
            landingBurnAMax = v.landingBurnAMax;
            landingBurnSteerKp = v.landingBurnSteerKp;
            landingBurnEngines = v.landingBurnEngines;
            suicideFactor = v.suicideFactor;
            targetError = v.targetError;
            igniteDelay = v.igniteDelay;
            noSteerHeight = v.noSteerHeight;
            uprightHeight = v.uprightHeight;
            uprightMaxHorizSpeed = v.uprightMaxHorizSpeed;
            v2gHeight = v.v2gHeight;
            v2gBlend = v.v2gBlend;
            v2gKp = v.v2gKp;
            v2gMaxSpeed = v.v2gMaxSpeed;
            v2gMaxAoA = v.v2gMaxAoA;
            v2gTermHeight = v.v2gTermHeight;
            v2gTerminal = v.v2gTerminal;
            v2gBrakeOnlyRadius = v.v2gBrakeOnlyRadius;
            lowTiltCapHeight = v.lowTiltCapHeight;
            lowTiltCap = v.lowTiltCap;
            taperSwitchY = v.taperSwitchY;
            recoveryProfile = v.recoveryProfile;
            flipAltMin = v.flipAltMin;
            flipAltMax = v.flipAltMax;
            starshipCorrectionGain = v.starshipCorrectionGain;
            bellyRollOffset = v.bellyRollOffset;
            bellyGainMaxKp = v.bellyGainMaxKp;
            flipTime = v.flipTime;
            flipUprightLean = v.flipUprightLean;
            bellyBrakeAoA = v.bellyBrakeAoA;
            bellyGlideAoA = v.bellyGlideAoA;
            bellyQFullBrake = v.bellyQFullBrake;
            bellyQFullGlide = v.bellyQFullGlide;
            bellyTerminalVh = v.bellyTerminalVh;
            bellyTerminalVhBlend = v.bellyTerminalVhBlend;
            bellyTerminalMaxAoA = v.bellyTerminalMaxAoA;
            bellyCorrectionVhFull = v.bellyCorrectionVhFull;
            starshipPassiveGlide = v.starshipPassiveGlide;
            starshipTrajCal = v.starshipTrajCal;
            trajCalViolentBrake = v.trajCalViolentBrake;
            trajCalAoaMod = v.trajCalAoaMod;
            trajCalAoaModDist = v.trajCalAoaModDist;
            trajCalAoaModGain = v.trajCalAoaModGain;
            trajCalAoaModSlew = v.trajCalAoaModSlew;
            trajCalOverError = v.trajCalOverError;
            trajCalOverAltFrac = v.trajCalOverAltFrac;
            trajCalBrakeMaxAlt = v.trajCalBrakeMaxAlt;
            trajCalShortError = v.trajCalShortError;
            trajCalBurnExitShort = v.trajCalBurnExitShort;
            trajCalBurnMaxQ = v.trajCalBurnMaxQ;
            trajCalBurnMaxAlt = v.trajCalBurnMaxAlt;
            trajCalBurnMinAlt = v.trajCalBurnMinAlt;
            trajCalBrakeMinAlt = v.trajCalBrakeMinAlt;
            trajCalBrakeArmTime = v.trajCalBrakeArmTime;
            trajCalBrakeRetractHold = v.trajCalBrakeRetractHold;
            trajCalBrakeMinHold = v.trajCalBrakeMinHold;
            trajCalReleaseFrac = v.trajCalReleaseFrac;
            trajCalBurnMaxDv = v.trajCalBurnMaxDv;
            trajCalBurnQHyst = v.trajCalBurnQHyst;
            trajCalBurnThrottle = v.trajCalBurnThrottle;
            trajImpactMaxAge = v.trajImpactMaxAge;
            bellyTrimFlipQ = v.bellyTrimFlipQ;
            glideLost = v.glideLost;
            parachuteBackup = v.parachuteBackup;
            flipBurnThrottle = v.flipBurnThrottle;
            flipBurnMaxAngle = v.flipBurnMaxAngle;
            aeroLiveCal = v.aeroLiveCal;
            aeroCalLift = v.aeroCalLift;
            aeroCalDrag = v.aeroCalDrag;
            // f65: clone only healthy bins - a corrupt (len!=3) source would
            // otherwise spread into every per-refresh sim copy
            aeroCalLiftQ = ((v.aeroCalLiftQ != null) && (v.aeroCalLiftQ.Length == 3)) ? (double[])v.aeroCalLiftQ.Clone() : new double[3] { 1, 1, 1 };
            aeroCalDragQ = ((v.aeroCalDragQ != null) && (v.aeroCalDragQ.Length == 3)) ? (double[])v.aeroCalDragQ.Clone() : new double[3] { 1, 1, 1 };
            aeroCalSamplesQ = ((v.aeroCalSamplesQ != null) && (v.aeroCalSamplesQ.Length == 3)) ? (int[])v.aeroCalSamplesQ.Clone() : new int[3] { 0, 0, 0 };
            aeroCalLiftAbQ = ((v.aeroCalLiftAbQ != null) && (v.aeroCalLiftAbQ.Length == 3)) ? (double[])v.aeroCalLiftAbQ.Clone() : new double[3] { 1, 1, 1 };
            aeroCalSamplesAbQ = ((v.aeroCalSamplesAbQ != null) && (v.aeroCalSamplesAbQ.Length == 3)) ? (int[])v.aeroCalSamplesAbQ.Clone() : new int[3] { 0, 0, 0 };
            simBrakesOut = v.airbrakeLatched; // f67: a braking descent predicts with the boards-out lift bands it is measuring. Source the LATCH, not airbrakeWanted: Fly resets airbrakeWanted=false before the prediction clone runs, so the flag would never propagate
            bellyLateralGain = v.bellyLateralGain;
            bellyLateralDeadband = v.bellyLateralDeadband;
            bellyLateralMaxDeg = v.bellyLateralMaxDeg;
            bellyLateralRollDeg = v.bellyLateralRollDeg;
            bellyLateralMaxAlt = v.bellyLateralMaxAlt;
            airbrakeMaxLiftRatio = v.airbrakeMaxLiftRatio;
            emergencyLanding = v.emergencyLanding;
            emergencyBrakeTau = v.emergencyBrakeTau;
            steerDamping = v.steerDamping;
            landingBurnVelDamp = v.landingBurnVelDamp;
            lowAoACap = v.lowAoACap;
            aoaRampLowAlt = v.aoaRampLowAlt;
            aoaRampTopAlt = v.aoaRampTopAlt;
            turnAroundCompleteAngle = v.turnAroundCompleteAngle;
            turnAroundMaxTime = v.turnAroundMaxTime;
            setLandingEnginesDone = false;
        }

        public BLController(Vessel a_vessel, bool useFAR = false) : base()
        {
            AttachVessel(a_vessel, useFAR);
        }

        public void AttachVessel(Vessel a_vessel, bool useFAR = false)
        {
            vessel = a_vessel;
            aeroModel = Trajectories.AerodynamicModelFactory.GetModel(vessel, vessel.mainBody, useFAR);
            lowestY = KSPUtils.FindLowestPointOnVessel(vessel);
        }

        public void InitReentryBurn(float kP, float maxAngle, double alt, double tgtSpeed)
        {
            pid_reentry = new PIDclamp("reentrySteer", 1, 0, 0, maxAngle);
            reentryBurnSteerKp = kP;
            reentryBurnMaxAoA = maxAngle;
            reentryBurnAlt = alt;
            reentryBurnTargetSpeed = tgtSpeed;
        }

        public void InitAeroDescent(float kP, float maxAngle)
        {
            pid_aero = new PIDclamp("aeroSteer", 1, 0, 0, maxAngle);
            aeroDescentSteerKp = kP;
            aeroDescentMaxAoA = maxAngle;
        }

        public void InitLandingBurn(float kP, float maxAngle)
        {
            pid_landing = new PIDclamp("landingSteer", 1, 0, 0, maxAngle);
            landingBurnSteerKp = kP;
            landingBurnMaxAoA = maxAngle;
        }

        public void SetTarget(double latitude, double longitude, double alt)
        {
            tgtLatitude = latitude;
            tgtLongitude = longitude;
            tgtAlt = alt;
        }

        public void SetLandingBurnEnginesFromString(string s)
        {
            if (vessel == null)
                return;
            if (s == "current")
                landingBurnEngines = null;
            else
            {
                string[] flags = s.Split(',');
                List<ModuleEngines> engines = KSPUtils.GetAllEngines(vessel);
                if (flags.Length != engines.Count)
                {
                    Log.Info("Vessel " + vessel.name + " has " + engines.Count + " but landing burn engines list has length " + flags.Length);
                    landingBurnEngines = null;
                    setLandingEnginesDone = false;
                    return;
                }
                landingBurnEngines = new List<ModuleEngines>();
                for (int i = 0; i < flags.Length; i++)
                {
                    if (flags[i] == "1")
                        landingBurnEngines.Add(engines[i]);
                    else if (flags[i] != "0")
                        Log.Info("Found invalid string '" + s + "' for landingBurnEngines. Expected a boolean list of active engines. e.g. 0,0,1,1,0 or current");
                }
            }
            setLandingEnginesDone = false;
        }

        public void SetPhase(BLControllerPhase a_phase)
        {
            minError = double.MaxValue; // reset so boostback doesn't give up
            smError = -1;
            boostbackGrowTime = 0;
            lowestY = KSPUtils.FindLowestPointOnVessel(vessel); // in case its changed

            // SetPhase(Unset) means "re-pick the phase from vessel state" at
            // EVERY caller (EnableGuidance, EmergencyLand, pad-armed apex
            // takeover) - flight 47: emergency pressed during BellyFlop hit
            // the old "(already Unset) && Unset" condition's else branch,
            // set a literal Unset, and the controller idled 22 s with zero
            // attitude/throttle control all the way into the ground
            if (a_phase == BLControllerPhase.Unset)
            {
                // (Re-)enable gets a fresh TrajCal burn budget and board latch
                // - the mid-flight latches must not survive a manual
                // disable->enable cycle
                airbrakeLatched = false;
                airbrakeLatchedT = -100;
                violentBrakeLatched = false; // f85 暴力减速 latch resets with the other mid-flight latches
                violentBrakeSinceT = -1;
                violentBrakeAbortT = -1;
                violentBrakeDepartT = -1;
                aoaModAoA = -1; // f95 governor re-seeds from the live schedule on the next enable
                aoaModBraking = false;
                vhFloorActive = false; // f107 vh floor resets with the other latches
                coastTransStartT = -1; // f96 trans-floor give-up state resets too
                coastTransGiveUp = false;
                bigTrimDir = 0; // f97 big-error burn latches reset too
                bigTrimSpentDv = 0;
                bigTrimFlipT = -100;
                bigTrimStartAlong = 0;
                bigTrimAbortT = -100;
                bigTrimLeverage = 200; // f98: leverage estimate re-seeds on enable
                bigTrimArmed = false; // f99 runaway-guard arming state resets too
                bigTrimRunawayT = -1;
                bigTrimBellyBurn = false; // f107 decisive-burn style resets with the burn latches
                pilotAttitudeMode = 0; // manual attitude buttons start OFF on every enable
                lateralEngageCross = -1; // f87 lateral watchdog resets with the other latches
                lateralAbort = false;
                glideLost = false;
                bellyDepartT = -1;
                bellyAttAcquired = false; // f102: glide-lost detector re-arms only after the belly attitude is acquired once
                bellyReacquireT = -1;
                pilotBurnGraceUntil = -1;
                chutesDeployed = false;
                flipStartT = -1;
                if (recoveryProfile == "starship")
                {
                    // Enable-anywhere (starship design D2): pick the entry
                    // phase from vessel state - stable orbit waits for the
                    // deorbit burn, an atmosphere-dipping orbit coasts,
                    // in-atmosphere enters the belly flop (or, low down, the
                    // flip / shared landing burn directly).
                    // EMERGENCY override (flight 46): this hull's natural
                    // aero trim at terminal q is TAIL-FIRST (engine-heavy,
                    // like the falcon) - the belly PD could not pull it off
                    // that trim (surf torque saturated, bellyErr stuck at
                    // 50-70 deg) and the landing then had no attitude
                    // control at all. So an emergency landing skips the
                    // belly flop entirely: Flip passes its 45-deg gate
                    // instantly when falling tail-first (att ~= -vel), and
                    // the shared landing burn descends retrograde on the
                    // trim the ship WANTS to be in
                    double atmDepth = vessel.mainBody.atmosphereDepth;
                    if ((emergencyLanding) && (vessel.altitude < atmDepth))
                    {
                        double yGround = vessel.altitude - Math.Max(0, vessel.terrainAltitude);
                        phase = (yGround > flipAltMin) ? BLControllerPhase.Flip : BLControllerPhase.LandingBurn;
                    }
                    else if (vessel.altitude < atmDepth)
                    {
                        double yToTgt = vessel.altitude - Math.Max(0, vessel.terrainAltitude);
                        if (yToTgt > flipAltMax)
                            phase = BLControllerPhase.BellyFlop;
                        else if (yToTgt > flipAltMin)
                            phase = BLControllerPhase.Flip;
                        else
                            phase = BLControllerPhase.LandingBurn;
                    }
                    else if ((vessel.orbit != null) && (vessel.orbit.PeA < atmDepth))
                        phase = BLControllerPhase.EntryCoast; // deorbit burn already flown
                    else
                        phase = BLControllerPhase.AwaitDeorbit; // stable orbit: wait for the burn
                }
                else
                {
                    // While ascending the only sensible starting point is the
                    // turn-around + boostback sequence (boostback nominally starts
                    // before apex). Picking AeroDescent here let the landing-burn
                    // transition fire immediately (landingBurnHeight is
                    // meaningless against an ascending trajectory), leaving the
                    // controller stuck commanding an impossible landing burn during
                    // the ascent - "enable does nothing until pressed several times"
                    bool ascending = Vector3d.Dot(vessel.GetObtVelocity(), Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position)) > 0;
                    if ((vessel.altitude > reentryBurnAlt) || ascending)
                        phase = BLControllerPhase.TurnAround; // flip first, then BoostBack
                    else
                        phase = BLControllerPhase.AeroDescent;
                }
            }
            else
                phase = a_phase;
            if (Utils.LoggingActive)
                LogSimulation();
        }

        public override string PhaseStr()
        {
            if (phase == BLControllerPhase.TurnAround)
                return Localizer.Format("#BoosterGuidance_TurnAround");
            if (phase == BLControllerPhase.BoostBack)
                return Localizer.Format("#BoosterGuidance_Boostback");
            if (phase == BLControllerPhase.Coasting)
                return Localizer.Format("#BoosterGuidance_Coasting");
            if (phase == BLControllerPhase.ReentryBurn)
                return Localizer.Format("#BoosterGuidance_ReentryBurn");
            if (phase == BLControllerPhase.AeroDescent)
                return Localizer.Format("#BoosterGuidance_AeroDescent");
            if (phase == BLControllerPhase.LandingBurn)
                return Localizer.Format("#BoosterGuidance_LandingBurn");
            if (phase == BLControllerPhase.AwaitDeorbit)
                return Localizer.Format("#BoosterGuidance_AwaitDeorbit");
            if (phase == BLControllerPhase.EntryCoast)
                return Localizer.Format("#BoosterGuidance_EntryCoast");
            if (phase == BLControllerPhase.BellyFlop)
                return Localizer.Format("#BoosterGuidance_BellyFlop");
            if (phase == BLControllerPhase.Flip)
                return Localizer.Format("#BoosterGuidance_Flip");
            return Localizer.Format("#BoosterGuidance_Unset");
        }

        // Prediction sim duration cap: a healthy starship belly glide from the
        // 70 km interface takes ~900-1000 s (vr ~ -55..-80), so the falcon
        // default 600 timed out EVERY tick and the surface-intersect fallback
        // returned fantasy impacts (flight 55/56 display garbage)
        public double PredictionMaxT()
        {
            // Flight 58: match the deorbit scope's 3600 - from an exo-
            // atmospheric enable (~86 km) the coast+glide takes ~1700-2000 s,
            // so 1500 still timed out and the clamped surface projection
            // diverged from the scope's completed sim = the user's "开不开
            // 引导两个落点". The wall-clock throttle (predClock) absorbs the
            // extra cost by refreshing less often
            return (recoveryProfile == "starship") ? 3600 : 600;
        }

        public void LogSimulation()
        {
            String name = PhaseStr().Replace(" ", "_");
            Vector3d tgt_r = vessel.mainBody.GetWorldSurfacePosition(tgtLatitude, tgtLongitude, tgtAlt) - vessel.mainBody.position;
            BLController tc = new BLController(this);
            // f115: same TurnAround/BoostBack -> Coasting conversion as the
            // live prediction (the sim cannot closed-loop a boostback - see
            // the prediction call site), so this diagnostic log stops showing
            // fantasy unbraked impacts during the early phases
            if ((phase == BLControllerPhase.BoostBack) || (phase == BLControllerPhase.TurnAround))
                tc.phase = BLControllerPhase.Coasting;
            Simulate.ToGround(tgtAlt, vessel, aeroModel, vessel.mainBody, tc, tgt_r, out targetT, Utils.LogType.simuate, logTransform, vessel.missionTime - logStartTime, PredictionMaxT());
            //Simulate.ToGround(tgtAlt, vessel, aeroModel, vessel.mainBody, null, tgt_r, out targetT, Utils.LogType.free, logTransform, vessel.missionTime - logStartTime);
        }

        // Estimate the propellant needed to fly back from the CURRENT state.
        // A coast simulation with ALL STEERING DISABLED gives the unguided
        // ballistic impact point, the flight time and the reentry/landing
        // propellant. (The first version left steering on: the simulated
        // controller then flew the descent to the target, so the "ballistic"
        // miss came out ~0 and the boostback estimate collapsed to tens of
        // kg - the hint always read "need 0.0t".) The boostback burn itself
        // cannot be simulated closed-loop in one shot (the sim's inner control
        // ticks never see the predicted error - that needs the finished
        // trajectory - so a simulated boostback just idles at min throttle and
        // the estimate comes out near-zero), so its propellant is estimated
        // analytically: translating the impact point by E over flight time T
        // costs roughly |E|/T of delta-V, inflated by a margin for burn-arc
        // and gravity losses. The margin grows with how hot/flat the ascent
        // is: a fast downrange profile spends much of the burn reversing
        // downrange velocity rather than translating the impact point.
        // Calibrated against actual burns as a linear function of the
        // horizontal speed at enable: flight 29 (vh=526 m/s, 94km miss)
        // needed 1.42, flight 30 (vh=1170 m/s, 247km miss) needed 1.57 -
        // slope 2.33e-4 per m/s through both points, intercept 1.30 which
        // recovers the old fixed margin for slow/steep ascents. Clamped to
        // [1.3, 1.8]: the fit is two points, so extrapolation is bounded
        // until more flights calibrate it. Combined with the post-boostback
        // mass correction in the dV display both flights come out +-5%.
        // Does not log or disturb the real controller; safe to call at
        // ~1 Hz from a dedicated hint controller.
        public double boostbackDvMargin = 1.3;
        public double boostbackDvMarginPerVh = 2.33e-4;
        public double boostbackDvMarginMax = 1.8;
        public void EstimateReturnFuel()
        {
            Vector3d up = Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position);
            Vector3d tgt_r = vessel.mainBody.GetWorldSurfacePosition(tgtLatitude, tgtLongitude, tgtAlt) - vessel.mainBody.position;
            BLController tc = new BLController(this);
            tc.phase = BLControllerPhase.Coasting;
            // Unguided: zero every steering budget so the sim cannot home
            // the impact point onto the target (EffectiveMaxAoA clamps all
            // corrections to zero; the unsteered landing burn still burns
            // essentially the same propellant)
            tc.landingBurnMaxAoA = 0;
            tc.aeroDescentMaxAoA = 0;
            tc.reentryBurnMaxAoA = 0;
            tc.lowAoACap = 0;
            double flightT;
            Vector3d impact = Simulate.ToGround(tgtAlt, vessel, aeroModel, vessel.mainBody, tc, tgt_r, out flightT);
            simFuelReentryKg = tc.simFuelReentryKg;
            simFuelLandingKg = tc.simFuelLandingKg;
            double ballisticError = Vector3d.Exclude(up, impact - tgt_r).magnitude;
            double isp = KSPUtils.GetAverageIsp(vessel);
            // Analytic landing-reserve floor: nulling the unbraked descent
            // speed at 2km plus a 50 m/s margin (touchdown, gravity losses,
            // profile wiggle). Calibrated on actual burns - flight 25: 343 m/s,
            // flight 29: 409 m/s (the sim's own accounting said ~1 m/s there).
            // Uses the same linear kg approximation as the boostback share;
            // at the CURRENT (pre-staging) mass, i.e. conservative by design
            if (tc.simLowSpeed > 0)
            {
                double kgFloor = 1000 * vessel.totalMass * (tc.simLowSpeed + 50) / (isp * 9.80665);
                simFuelLandingKg = Math.Max(simFuelLandingKg, kgFloor);
            }
            double dvBoostback = 0;
            if (flightT > 1)
            {
                double vhEnable = Vector3d.Exclude(up, vessel.obt_velocity).magnitude;
                double margin = HGUtils.Clamp(boostbackDvMargin + boostbackDvMarginPerVh * vhEnable, boostbackDvMargin, boostbackDvMarginMax);
                dvBoostback = margin * ballisticError / flightT;
            }
            // vessel.totalMass is in TONNES (KSP mass unit) - x1000 for kg.
            // Without the conversion the boostback share was 1000x small and
            // the hint still read "need 0.1t" (flight 20: dv=396 m/s showed
            // bb=35 "kg" which was really 35 t).
            // Rocket equation, NOT the linear M*dv/ve approximation: the
            // boostback is a large burn whose mass drops as it goes, so the
            // linear form overshot the boostback propellant by 13-19%
            // (flight 31: 74.5t hinted vs 65.7t actual; flight 32: 78.8t
            // vs 66.4t) which ALSO dragged massAtLanding ~12t low in the
            // dV display and padded the landing share by ~50 m/s
            simFuelBoostbackKg = 1000 * vessel.totalMass * (1 - Math.Exp(-dvBoostback / (isp * 9.80665)));
            // Diagnostics for the hint driver log line
            simLastBallisticError = ballisticError;
            simLastFlightT = flightT;
            simLastDvBoostback = dvBoostback;
        }
        public double simLastBallisticError = 0;
        public double simLastFlightT = 0;
        public double simLastDvBoostback = 0;


        // Note: filename is the basename from which we appent
        //  .actual.dat
        //  .after_boostback.dat
        public void StartLogging(string filename)
        {
            if (!Utils.LoggingActive)
            {
                Utils.StartLogging(filename);
                SetUpLogTransform(filename);
                Utils.Log(Utils.LogType.actual, "time phase x y z vx vy vz ax ay az att_err amin amax steer_gain target_error totalMass traj_x traj_z");
                logStartTime = vessel.missionTime;
                Log.Info("BLController.StartLogging: " + filename + " phase=" + phase + " missionTime=" + vessel.missionTime);
                LogSimulation();
            }
            else
                Log.Info("BLController.StartLogging: already active, ignored (" + filename + ")");
        }

        void SetUpLogTransform(string filename)
        {
            logTransform = Targets.SetUpTransform(vessel.mainBody, tgtLatitude, tgtLongitude, tgtAlt);
        }

        public void StopLogging()
        {
            if (Utils.LoggingActive)
                Log.Info("BLController.StopLogging: phase=" + phase);
            Utils.EndLogging();
        }

        // Vector adjustment to steer vector (must be descending)
        // If gain is positive steer aerodynamically (towards target)
        // otherwise steer to fire thrust in opposite direction
        private Vector3d GetSteerAdjust(Vector3d tgtError, double gain, double maxAoA)
        {
            Vector3d adj = tgtError * gain * deg2rad;
            double maxAdj = maxAoA * deg2rad; // approx as 45 degress sideways component so unit vector is 1
            if (adj.magnitude > maxAdj)
                adj = Vector3d.Normalize(adj) * maxAdj;
            return adj;
        }

        // Steering correction + rate damping with the TOTAL clamped to the AoA budget.
        // The bare "- steerDamping * omegaLat" was unbounded: with real (large) body
        // rates in a fast turbulent descent it could rotate the steer command past
        // horizontal, so the booster ended up thrusting sideways at full throttle
        // during the landing burn (flight 12 crash)
        private Vector3d GetSteerCorrection(Vector3d tgtError, double gain, double maxAoA, Vector3d omegaLat)
        {
            Vector3d corr = tgtError * gain * deg2rad - steerDamping * omegaLat;
            double maxCorr = maxAoA * deg2rad;
            if (corr.magnitude > maxCorr)
                corr = Vector3d.Normalize(corr) * maxCorr;
            return corr;
        }

        // Velocity-to-go terminal correction: the horizontal velocity that
        // nulls the position error by touchdown is v_des = -posErr/tGo (with
        // tGo the constant-deceleration time-to-ground 2y/|vy|, floored so
        // the command cannot blow up in the last metres). Leaning against
        // the velocity ERROR (v_des - v) brakes the approach instead of
        // pumping it: as the velocity tracks v_des the position error
        // shrinks, and v_des itself shrinks toward zero at touchdown. Uses
        // only local position/velocity - no simulation, no noise
        // f83: starship terminal-calibration authority. The ship enters the
        // landing burn slow with a km-class residual from the belly phase
        // and 30+ s of powered descent to spend - earlier handoff, faster
        // closing speed and a bigger velocity-law budget than the
        // falcon-validated defaults (falcon9 behavior unchanged)
        private double V2gTermHeightEff { get { return (recoveryProfile == "starship") ? Math.Max(v2gTermHeight, 1500) : v2gTermHeight; } }
        private double V2gMaxAoAEff { get { return (recoveryProfile == "starship") ? Math.Max(v2gMaxAoA, 18) : v2gMaxAoA; } }
        private double V2gMaxSpeedEff { get { return (recoveryProfile == "starship") ? Math.Max(v2gMaxSpeed, 45) : v2gMaxSpeed; } }

        private Vector3d VelocityToGoCorrection(Vector3d posErr, Vector3d vel_air, Vector3d up, double y, double vy, double maxAoA, Vector3d omegaLat)
        {
            double tGo = Math.Max(2, 2 * y / Math.Max(5, -vy));
            // Brake-only when nearly on target: a v_des reversal at full
            // throttle in the last seconds pumps speed instead of killing
            // it (flight 22 tip-over)
            Vector3d vDes = (posErr.magnitude > v2gBrakeOnlyRadius) ? -posErr / tGo : Vector3d.zero;
            // f113 terminal brake-only (user-approved 方案C): in the terminal
            // zone, when the remaining time-to-ground cannot cover the offset
            // even at full correction speed, stop translating and just kill
            // the horizontal speed - f111/f113 overshot the pad then crawled
            // back at 5-6 m/s THROUGH touchdown (near tip-over). Landing
            // stopped-but-short beats closer-but-moving. Falcon only.
            if ((recoveryProfile != "starship") && (y < v2gTermHeight)
                && (posErr.magnitude > V2gMaxSpeedEff * tGo))
                vDes = Vector3d.zero;
            if (vDes.magnitude > V2gMaxSpeedEff)
                vDes = Vector3d.Normalize(vDes) * V2gMaxSpeedEff;
            return GetSteerCorrection(vDes - Vector3d.Exclude(up, vel_air), v2gKp, maxAoA, omegaLat);
        }

        // Fade the velocity-braking angle budget out near the ground:
        // flight 21 was still translating under a 12-degree correction in
        // the last 50m and touched down tilted (broken landing leg). Full
        // budget above 150m, a quarter of it by 30m - enough to null the
        // last m/s, not enough to land tilted
        private double TerminalAoAFade(double y)
        {
            return HGUtils.Clamp((y - 30) / 120, 0.25, 1);
        }

        // Altitude-scheduled AoA cap: full authority above aoaRampTopAlt,
        // capped at lowAoACap degrees (absolute) below aoaRampLowAlt, linear
        // ramp between. Replicates the manual "lower the steer gain at low
        // altitude" procedure that stops the descent limit cycle, without
        // depending on the GUI-derived base maxAoA
        private double EffectiveMaxAoA(double maxAoA, double y)
        {
            double f = HGUtils.Clamp((y - aoaRampLowAlt) / Math.Max(1, aoaRampTopAlt - aoaRampLowAlt), 0, 1);
            return Math.Min(maxAoA, lowAoACap + (maxAoA - lowAoACap) * f);
        }

        private double CalculateSteerGain(double throttle, Vector3d vel_air, Vector3d r, double y, double totalMass, bool log)
        {
            Vector3d Faero = aeroModel.GetForces(vessel.mainBody, r, vel_air, 180 * deg2rad); // 180 degrees (retrograde);

            // Find lift by just considering change in aero force vector
            Vector3d Faero2 = aeroModel.GetForces(vessel.mainBody, r, vel_air, (180 - 15) * deg2rad);

            // Calculate lift vector orthogonal to the drag vector when retrograde
            Vector3d Fdiff = Faero2 - Faero;
            Vector3d Flift = Fdiff - Vector3d.Project(Fdiff, Faero);

            double sideFA = Flift.magnitude * aeroMult; // aero dynamic lift at 15 degrees AoA
            double thrust = 0;
            if (throttle > 0)
                thrust = minThrust + throttle * (maxThrust - minThrust);
            double sideFT = thrust * Math.Sin(15 * deg2rad); // sideways component of thrust at 15 degrees
            double gain = 0;
            // When its a toss up whether thrust or aerodynamic steering is better gain can become very high or infinite.
            // Fade steering authority continuously to zero as the side-force ratio approaches aeroThrustBlendEnd
            // (a hard zero-band here caused steer authority to flicker on/off and drove a limit cycle).
            double ratio = Math.Min(sideFA, sideFT) / Math.Max(sideFA, sideFT);
            if (Math.Abs(sideFA - sideFT) > 0)
            {
                double fade = HGUtils.Clamp((aeroThrustBlendEnd - ratio) / (aeroThrustBlendEnd - aeroThrustPropMargin), 0, 1);
                // REVERTED per user order (退回前天稳定批次): f137's abs()
                // removed - the gain goes negative again when thrust
                // dominates (sideFT > sideFA), inverting the homing
                // correction in the powered near-hover regime (f137's 70 s
                // hover-fall ingredient). Accepted with the revert
                gain = fade * totalMass / (sideFA - sideFT);
            }

            if (log)
                Log.Info("sideFA(15 degrees)=" + sideFA + " sideFT(15 degrees)=" + sideFT + " throttle=" + throttle + " alt=" + vessel.altitude + " gain=" + gain + " minThrust=" + minThrust + " maxThrust=" + maxThrust);

            return gain;
        }

        // Belly-flop steer gain (design D6): same units convention as
        // CalculateSteerGain (totalMass / side-force-at-15-degrees) but the
        // force gradient is probed around 90 deg AoA - how much sideways
        // force a 15-degree tilt of the belly normal buys. No thrust term:
        // the belly flop is unpowered. 0 when the craft has no measurable
        // lateral authority broadside (the correction then amounts to
        // nothing, which the T2 dumb glide is there to confirm)
        // Dynamic pressure from altitude + airspeed; StockAeroUtil.GetDensity
        // is documented to match Vessel.atmDensity exactly, so the same call
        // serves the real vessel AND the prediction sim (whose "vessel" state
        // is the real ship's - its atmDensity would be the wrong altitude's)
        public static double DynamicPressure(double alt, double vair, CelestialBody body)
        {
            return 0.5 * Trajectories.StockAeroUtil.GetDensity(alt, body) * vair * vair;
        }

        // Scheduled belly-flop AoA (deg) for the current dynamic pressure:
        // log-q lerp from the brake end (thin air, max drag) to the glide
        // end (the most the flaps can hold before the hull flips tail-first)
        public double BellyAoADeg(double q, double vh = -1, double pathDownDeg = -1)
        {
            double lo = Math.Max(1, bellyQFullBrake);
            double hi = Math.Max(lo * 1.01, bellyQFullGlide);
            double t = HGUtils.Clamp((Math.Log(Math.Max(1, q)) - Math.Log(lo)) / (Math.Log(hi) - Math.Log(lo)), 0, 1);
            double aoa = bellyBrakeAoA + t * (bellyGlideAoA - bellyBrakeAoA);
            if (vh >= 0)
            {
                double w = HGUtils.Clamp((bellyTerminalVh - vh) / Math.Max(1, bellyTerminalVhBlend), 0, 1);
                aoa += w * (Math.Min(bellyBrakeAoA, bellyTerminalMaxAoA) - aoa);
            }
            // User directive (f84): even in the final low-speed phase - down
            // to vh = 0 - the nose must NEVER drop below the horizon: a level
            // fuselage is what keeps the flip + relight + retro landing
            // stable. Nose elevation = AoA - pathDownDeg, so once the
            // terminal band has engaged (vh < bellyTerminalVh) floor the
            // commanded AoA at the path's steepness (capped at the full-
            // brake end of the schedule). This overrides the terminal 45 deg
            // blend cap on steep slow falls - the wallow that cap guarded
            // against came from STEERING in the degenerate vertical-wind
            // frame, not from the level attitude itself (the correction fade
            // at low vh below owns that job now). Deliberately gated to the
            // terminal band: flooring the fast glide/entry at its steep path
            // would dump the glide's energy (f84's high-altitude bleed)
            if ((pathDownDeg > 0) && (vh >= 0) && (vh < bellyTerminalVh))
                aoa = Math.Max(aoa, Math.Min(pathDownDeg, bellyBrakeAoA));
            return aoa;
        }

        // Path angle of the velocity vector below the horizon (deg, >= 0),
        // for the nose-above-horizon AoA floor above
        public static double PathDownDeg(Vector3d vel_air, Vector3d up)
        {
            double vv = -Vector3d.Dot(vel_air, up);
            if (vv <= 0)
                return 0;
            return Math.Atan2(vv, Math.Max(1, Vector3d.Exclude(up, vel_air).magnitude)) / deg2rad;
        }

        private double CalculateBellySteerGain(Vector3d vel_air, Vector3d r, double totalMass, double aoaDeg)
        {
            double a = aoaDeg * deg2rad;
            Vector3d F0 = aeroModel.GetForces(vessel.mainBody, r, vel_air, a);
            Vector3d F1 = aeroModel.GetForces(vessel.mainBody, r, vel_air, a - 15 * deg2rad);
            Vector3d Fdiff = F1 - F0;
            Vector3d Flat = Fdiff - Vector3d.Project(Fdiff, F0);
            double sideF = Flat.magnitude * aeroMult;
            if ((sideF <= 0) || double.IsNaN(sideF))
                return 0;
            return totalMass / sideF;
        }

        // Suicide-burn throttle law as a pure function (no KSP types) so the
        // diagnostic replay harness can drive it with logged flight states.
        // y is the effective height (alt - tgtAlt - touchdownMargin + lowestY):
        // >0 in the suicide-profile zone, <=0 deep inside the touchdown margin.
        // Above taperSwitchY the vessel tracks the suicide profile
        //   dvy = -sqrt((1+sf)*av*y) - touchdownSpeed
        // with profile-slope feedforward; below it the touchdown taper
        //   dvy = -(touchdownSpeed + 0.25 * yGround), yGround = y + margin
        // descends proportionally to lowest-point height, tapering to
        // touchdownSpeed at the surface.
        //
        // FLIGHT 25 WARNING - do not let the suicide branch run to y=0:
        // its feedforward slope d(dvy)/dt = (1+sf)*av*(-vy)/(2*sqrt((1+sf)*av*y))
        // diverges like 1/sqrt(y) as y->0+, which creates a stable fixed
        // point at y~1cm: any descent is met by an arbitrarily large
        // deceleration demand, so a gently-arriving vessel parks in a hover
        // exactly at the profile zero and burns its tanks dry (33s hover at
        // 35m, thr=g/amax=0.14 reproduced to the cent). A hotter arrival
        // punches through on momentum (flight 26 with 4 engines: -11.4 m/s
        // at the boundary) - which branch you get is a lottery. Switching to
        // the finite-slope taper at taperSwitchY=2 makes the handoff
        // deterministic: the taper target there is always SLOWER than the
        // suicide profile, so it is always a catch (brake), never a dive.
        internal static double SuicideBurnThrottle(double y, double vy, double av, double g,
            double amin, double amax, double suicideFactor, double touchdownSpeed,
            double touchdownMargin, double dt, double minThrottle, double taperSwitchY,
            out double dvy, out double dvyDt, double vh = 0)
        {
            if (y > taperSwitchY)
            {
                // f110: the suicide budget is a TOTAL-speed budget - the same
                // thrust must kill vh too. The 7.5 m core entered the burn at
                // vy=-734 with vh=650: the vy-only profile allowed -1132 at
                // 16.5 km while the true stoppable total was ~984, and the
                // tilt tax (thrust leaned ~40 deg off vertical to kill the
                // horizontal) ate the 5% margin - arrived at 11 m doing -35
                // and exploded (the burn sim itself ended -278 at 31 m, i.e.
                // the plan never reached the ground). Reserve vh^2 of the
                // v^2 budget for the horizontal kill; as vh dies the profile
                // relaxes back to the proven vy-only shape
                double vStop2 = Math.Max(0, (1 + suicideFactor) * av * y - vh * vh); // (1+sf): factor is 2 for a perfect suicide burn, lower for margin
                dvy = -Math.Sqrt(vStop2) - touchdownSpeed;
                dvyDt = (1 + suicideFactor) * av * (-vy) / (2 * Math.Sqrt(Math.Max(1, vStop2)));
            }
            else
            {
                // Inside the touchdown margin: instead of creeping at touchdownSpeed
                // for the whole margin (a 12s+ hover with touchdownMargin=30), descend
                // proportionally to height above ground, tapering to touchdownSpeed
                // at the surface
                double yGround = Math.Max(0, y + touchdownMargin); // lowest-point height above target altitude
                dvy = -(touchdownSpeed + 0.25 * yGround);
                dvyDt = 0.25 * (-vy);
            }
            dvyDt = HGUtils.Clamp(dvyDt, 0, av); // never demand more than full decel capability
            double err_dv = vy - dvy; // +ve is velocity too high
            // Feedforward tracks the profile, the fixed 0.5s feedback term only
            // handles the residual - no steady-state deficit on the steep part,
            // no bang-bang limit cycle near the ground
            double da = g + dvyDt - err_dv / Math.Max(dt, 0.5);
            return HGUtils.Clamp((da - amin) / (0.01 + amax - amin), minThrottle, 1);
        }

        public override string GetControlOutputs(
                        Vessel vessel,
                        double totalMass,
                        Vector3d r, // world pos relative to body
                        Vector3d v, // world velocity
                        Vector3d att, // attitude
                        double minThrust, double maxThrust,
                        double t,
                        CelestialBody body,
                        bool simulate, // if true just go retrograde (no corrections)
                        out double throttle, out Vector3d steer,
                        out bool landingGear, // true if landing gear requested (stays true)
                        bool bailOutLandingBurn = false, // if set too true on RO set throttle=0 if thrust > gravity at landing
                        bool showCpuTime = false)

        {
            // height of lowest point with additional margin
            double y = r.magnitude - body.Radius - tgtAlt - touchdownMargin + lowestY;
            // Height above the GROUND BELOW the (possibly simulated) position
            // rather than above the target-altitude plane (flight 50: the
            // landing came down 559 km from the target where the terrain is
            // higher than tgtAlt - every ground-proximity trigger referenced
            // to the target plane (flip, burn ignition, suicide profile, gear,
            // tilt caps) fired late in true terms: 9 s at min throttle with a
            // perfect 0.5-deg attitude, then a panic burn that flamed out at
            // 31 m/s). Live: vessel.terrainAltitude (per-frame cached). Sim:
            // body terrain at the SIM position, gated to low altitude so the
            // prediction sim does not pay terrain lookups at cruise height
            double yG = y;
            double groundAlt = tgtAlt;
            if (y < 15000)
            {
                if ((!simulate) && (vessel != null))
                    groundAlt = Math.Max(0, vessel.terrainAltitude);
                else
                {
                    Vector3d wpos = r + body.position;
                    groundAlt = Math.Max(0, body.TerrainAltitude(body.GetLatitude(wpos), body.GetLongitude(wpos)));
                }
                yG = r.magnitude - body.Radius - groundAlt - touchdownMargin + lowestY;
            }
            Vector3d up = Vector3d.Normalize(r);
            Vector3d vel_air = v - body.getRFrmVel(r + body.position);
            double vy = Vector3d.Dot(vel_air, up);
            double amin = minThrust / totalMass;
            double amax = maxThrust / totalMass;
            float minThrottle = 0.01f;
            BLControllerPhase lastPhase = phase;
            bellyRollPulseDeg = 0; // f114: re-armed every tick; only the BellyFlop lateral pulse block raises it (a stale value would bank the flip)
            Vector3d tgt_r = body.GetWorldSurfacePosition(tgtLatitude, tgtLongitude, tgtAlt) - body.position;

            System.Diagnostics.Stopwatch timer = new System.Diagnostics.Stopwatch();
            timer.Start();
            string msg = ""; // return message about the status

            landingGear = (yG < deployLandingGearHeight) && (deployLandingGear);

            double g = FlightGlobals.getGeeForceAtPosition(r + body.position).magnitude;
            double dt = t - lastt;
            if ((lastt < 0) || (dt < 0))
                dt = 0; // fresh controller / clock rebase: nothing integrable yet (see lastt)

            // Lateral angular rate from attitude history (rad/s) for rate damping.
            // Derived from att in world frame rather than vessel.angularVelocity (KSP frame
            // convention is ambiguous) and rather than d(error)/dt (amplifies prediction noise).
            Vector3d omegaLat = Vector3d.zero;
            if ((attPrevValid) && (dt > 0))
            {
                Vector3d omega = Vector3d.Cross(attPrev, att) / dt;
                if (omega.magnitude < 10) // ignore absurd one-tick jumps (decoupler, time warp)
                    omegaLat = Vector3d.Exclude(att, omega);
            }

            // No thrust - retrograde relative to surface (default and Coasting phase
            throttle = 0;
            steer = -Vector3d.Normalize(vel_air);
            bool airbrakeOutPrev = airbrakeWanted; // last tick's state (this tick's is computed below) - f64: brake drag folded into kD (1.0->1.5) and warped kL (1.0->2.33) at low alt, then got SAVED to file and poisoned the next flight's seed. f66: kD keeps that gate, kL does not (see below)
            airbrakeWanted = false; // default retracted; the BellyFlop overshoot block re-arms it

            // Live aero calibration measurement (see aeroCalLift comment).
            // attitudeError here is still LAST tick's (zeroed below): only
            // sample while the ship is actually on the scheduled attitude,
            // unpowered last tick, in the glide AoA regime, with enough q for
            // a clean signal. dt bounds reject warp/teleport jumps
            if ((!simulate) && (vessel != null))
            {
                double calDt = (calPrevT >= 0) ? (t - calPrevT) : -1;
                if ((aeroLiveCal) && (phase == BLControllerPhase.BellyFlop) && (!PassiveGlide())
                    && (calDt > 0.005) && (calDt < 0.2) && (prevThrottleOut <= 0.001)
                    && ((pilotThrottle <= 0.01f) || (!pilotBurnAttitude)) // pilot burns bypass the throttle field - gate them out or thrust accel poisons kL/kD. pilotBurnAttitude here is LAST tick's (set below): a REFUSED burn flows no thrust, so a held-but-blocked lever must not silence the cal - f66's user held full throttle 70+s against the q-guard and the whole 22km descent sampled n=0
                    && (bellyAttErrLive < 10) && (vel_air.magnitude > 50))
                {
                    double qCal = DynamicPressure(y, vel_air.magnitude, body);
                    double aoaCal = BellyAoADeg(qCal, Vector3d.Exclude(up, vel_air).magnitude, PathDownDeg(vel_air, up));
                    if ((qCal > 200) && (aoaCal <= 45))
                    {
                        Vector3d aeroReal = (v - calPrevV) / calDt - r * (-body.gravParameter / (r.magnitude * r.magnitude * r.magnitude));
                        Vector3d dragAxis = -Vector3d.Normalize(vel_air);
                        // Lift axis = up with the along-velocity part removed
                        // (perpendicular to the wind, up-ish). f61 bug: this
                        // was Exclude(up, vel_air) = the HORIZONTAL VELOCITY
                        // direction - in a shallow glide that is nearly
                        // anti-parallel to dragAxis, so lR <= 0 forever and
                        // kL never sampled (frozen at 1.00 through 2990
                        // samples while kD moved 1.0 -> 1.4)
                        Vector3d liftPerp = Vector3d.Exclude(vel_air, up);
                        if (liftPerp.magnitude > 0.01)
                        {
                            Vector3d liftAxis = Vector3d.Normalize(liftPerp);
                            Vector3d modelAcc = aeroModel.GetForces(body, r, vel_air, aoaCal * deg2rad) / totalMass;
                            double dM = Vector3d.Dot(modelAcc, dragAxis);
                            double lM = Vector3d.Dot(modelAcc, liftAxis);
                            double dR = Vector3d.Dot(aeroReal, dragAxis);
                            double lR = Vector3d.Dot(aeroReal, liftAxis);
                            bool sampled = false;
                            int calBin = AeroCalBin(qCal);
                            bool binOk = !CalBinsBad("sample", calBin);
                            if (binOk && (!airbrakeOutPrev) && (aeroCalSamplesQ[calBin] == 0))
                            {
                                // a fresh band starts from the global EMA so its
                                // first samples do not jerk the prediction (f64)
                                aeroCalLiftQ[calBin] = aeroCalLift;
                                aeroCalDragQ[calBin] = aeroCalDrag;
                            }
                            if (binOk && (airbrakeOutPrev) && (aoaCal <= 25) && (aeroCalSamplesAbQ[calBin] == 0))
                            {
                                // a fresh boards-out band starts from the
                                // boards-in band (the closest known state)
                                aeroCalLiftAbQ[calBin] = aeroCalLiftQ[calBin];
                            }
                            // kD stays brakes-gated (f64): deployed boards
                            // drag lies EXACTLY on the drag axis, so a
                            // brakes-out sample measures boards+hull, not
                            // the hull - that poisoned kD 1.0->1.5 and the
                            // saved seed
                            if ((dM > 0.2) && (dR > 0) && (!airbrakeOutPrev))
                            {
                                double kd = HGUtils.Clamp(dR / dM, 0.3, 3);
                                aeroCalDrag += aeroCalRate * (kd - aeroCalDrag);
                                if (binOk)
                                    aeroCalDragQ[calBin] += aeroCalRate * (kd - aeroCalDragQ[calBin]);
                                sampled = true;
                            }
                            // kL samples with or without brakes (f66, user
                            // directive "把滑翔段KL重新算") - the f64 blanket
                            // gate starved the q>4kPa bins on every braking
                            // descent (flight 65's 28km-long impact walk).
                            // BUT boards-out lift is a different calibration
                            // (f67: 星舰货运 kL 1.58 -> 3.0 with boards out at
                            // q~2-3kPa): brakes-in samples feed the global EMA
                            // and the boards-in bands; brakes-out samples feed
                            // ONLY the boards-out bands, never the global -
                            // the saved global must stay boards-in truth or
                            // the next unbraked flight predicts a fantasy
                            // glide. The 10deg belly-error gate above still
                            // bounds the trim shift the boards induce
                            if ((lM > 0.2) && (lR > 0))
                            {
                                double kl = HGUtils.Clamp(lR / lM, 0.3, 3);
                                if (!airbrakeOutPrev)
                                {
                                    aeroCalLift += aeroCalRate * (kl - aeroCalLift);
                                    if (binOk)
                                        aeroCalLiftQ[calBin] += aeroCalRate * (kl - aeroCalLiftQ[calBin]);
                                    sampled = true;
                                }
                                // f82: boards-out lift only counts in the pure
                                // glide regime (scheduled AoA <= 25 deg). The
                                // boards-extend verdict decides whether boards
                                // may auto-deploy against a GLIDE overshoot, but
                                // the high-AoA brake phase is different physics:
                                // 38-48 deg scheduled-AoA samples at 44 km (plus
                                // old flip-phase aoa=85 samples) polluted kLqab
                                // to a 3.5-4x ratio, the verdict then read
                                // "boards extend the glide" on a hull where they
                                // are a pure brake, and it blocked a real
                                // 8-11 km overshoot redeploy at 24 km
                                else if ((binOk) && (aoaCal <= 25))
                                {
                                    aeroCalLiftAbQ[calBin] += aeroCalRate * (kl - aeroCalLiftAbQ[calBin]);
                                    sampled = true;
                                }
                            }
                            if (sampled)
                            {
                                calSamples++;
                                if (binOk)
                                {
                                    if (airbrakeOutPrev)
                                    { if (aoaCal <= 25) aeroCalSamplesAbQ[calBin]++; }
                                    else
                                        aeroCalSamplesQ[calBin]++;
                                }
                            }
                        }
                    }
                }
                // One status line per 15 s through the whole glide - samples
                // or not, the gate values must be visible (f60: a silently
                // blocked gate cost the whole flight's calibration)
                if ((aeroLiveCal) && (phase == BLControllerPhase.BellyFlop) && (!PassiveGlide())
                    && (vel_air.magnitude > 50) && (t - calLastLogT >= 15))
                {
                    double qLog = DynamicPressure(y, vel_air.magnitude, body);
                    calLastLogT = t;
                    if (CalBinsBad("log", -1))
                        Log.Info(string.Format("[AeroCal] t={0:F0} alt={1:F0} q={2:F0} aoa={3:F0} bErr={4:F1} dt={5:F3} thr={6:F3} kL={7:F2} kD={8:F2} n={9}",
                            t, y, qLog, BellyAoADeg(qLog, Vector3d.Exclude(up, vel_air).magnitude, PathDownDeg(vel_air, up)), bellyAttErrLive, calDt, prevThrottleOut, aeroCalLift, aeroCalDrag, calSamples));
                    else
                    Log.Info(string.Format("[AeroCal] t={0:F0} alt={1:F0} q={2:F0} aoa={3:F0} bErr={4:F1} dt={5:F3} thr={6:F3} ab={7} kL={8:F2} kD={9:F2} n={10} kLq={11:F2}/{12:F2}/{13:F2} kDq={14:F2}/{15:F2}/{16:F2} nq={17}/{18}/{19} kLqab={20:F2}/{21:F2}/{22:F2} nqab={23}/{24}/{25}",
                        t, y, qLog, BellyAoADeg(qLog, Vector3d.Exclude(up, vel_air).magnitude, PathDownDeg(vel_air, up)), bellyAttErrLive, calDt, prevThrottleOut, airbrakeOutPrev ? 1 : 0, aeroCalLift, aeroCalDrag, calSamples,
                        aeroCalLiftQ[0], aeroCalLiftQ[1], aeroCalLiftQ[2], aeroCalDragQ[0], aeroCalDragQ[1], aeroCalDragQ[2],
                        aeroCalSamplesQ[0], aeroCalSamplesQ[1], aeroCalSamplesQ[2],
                        aeroCalLiftAbQ[0], aeroCalLiftAbQ[1], aeroCalLiftAbQ[2],
                        aeroCalSamplesAbQ[0], aeroCalSamplesAbQ[1], aeroCalSamplesAbQ[2]));
                }
                calPrevV = v;
                calPrevT = t;
            }

            Vector3d error = Vector3d.zero;
            attErrPrevTick = attitudeError; // f126: preserve last tick's tracking error for the throttle-floor attitude gate below
            attitudeError = 0;
            // Passive glide: no prediction sim at all - the attitude schedule
            // flies the pure aero brake and nothing consumes an impact error.
            // TrajCal: no prediction sim either - TRAJECTORIES owns the impact
            // Impact error source (f83 cleanup): the FALCON path below runs
            // the 1 Hz own-sim prediction. Starship NEVER does (its own-sim
            // prediction is deleted - the f55-f72 failure history): TrajCal
            // reads Trajectories' red mark, anything else is passive
            if ((!simulate) && (recoveryProfile != "starship") && (y > noSteerHeight) && (phase != BLControllerPhase.AwaitDeorbit))
            {
                double predInterval = HGUtils.Clamp(predWallDur / predCpuFraction, 0.1, 2.0);
                bool predFresh = (predWallT < 0) || (predClock.Elapsed.TotalSeconds - predWallT >= predInterval);
                if (predFresh)
                {
                    double pw0 = predClock.Elapsed.TotalSeconds;
                BLController tc = new BLController(this);
                // Only simulate phases beyond boostback so boostback minimizes error and simulate includes just
                // the remaining phases and doesn't try to redo reentry burn for instance
                // f115: TurnAround converts too. The sim cannot closed-loop a
                // boostback (its inner ticks see no predicted error, so it
                // idles at min throttle - see EstimateReturnFuel's comment),
                // and from TurnAround the sim's tick-1 BoostBack inherited the
                // PREVIOUS prediction's targetError: once that read >10 m the
                // in-sim BoostBack could never exit to Coasting (needs
                // targetError < 10) and the sim fell the whole way in a fake
                // BoostBack, "impacting" unbraked ~90 km off (f115 sim run 0:
                // 1440 m/s at the ground while run 1 - identical start state,
                // targetError still 0 - exited to Coasting and landed soft).
                // The garbage impact then poisoned the next copy's targetError
                // = the red cross dancing 84-107 km all through TurnAround/
                // BoostBack. Show the honest un-corrected arc instead - the
                // same semantics the BoostBack phase already flies by (where
                // you land if the correction stops now)
                if ((phase == BLControllerPhase.BoostBack) || (phase == BLControllerPhase.TurnAround))
                    tc.phase = BLControllerPhase.Coasting;

                Vector3d newPred = Simulate.ToGround(tgtAlt, vessel, aeroModel, body, tc, tgt_r, out targetT, Utils.LogType.none, null, 0, PredictionMaxT());
                landingBurnAMax = tc.landingBurnAMax;
                landingBurnHeight = tc.landingBurnHeight; // Update from simulation
                    predWallDur = Math.Max(0.001, predClock.Elapsed.TotalSeconds - pw0);
                    predWallT = predClock.Elapsed.TotalSeconds;
                // Flight-55/56 diagnostic: is the prediction sim completing or
                // timing out into the fantasy projection? Rate-limited (f56's
                // limit cycle flooded the log at 5 lines/s)
                bool predTimeout = targetT >= PredictionMaxT() - 0.1;
                predBodyRelPos = newPred;
                if ((predTimeout != predDiagTimeout) && (t - predDiagLastLogT >= 2))
                {
                    predDiagTimeout = predTimeout;
                    predDiagLastLogT = t;
                    Log.Info(string.Format("[Predict] t={0:F1} phase={1} terr={2:F0} simT={3:F1} simEndPhase={4} timeout={5}",
                        t, phase, targetError, targetT, tc.phase, predTimeout));
                }
                }
                // Steering always consumes the latest prediction (stale is
                // fine - see predClock comment); recomputing the horizontal
                // error against the CURRENT up keeps it honest as the vessel
                // moves between runs
                error = predBodyRelPos - tgt_r;
                error = Vector3d.Exclude(up, error); // make sure error is horizontal
                targetError = error.magnitude;
            }
            // Traj-driven calibration error source (f76 user request): read
            // TRAJECTORIES' predicted impact (reflection, TrajAPI) and take
            // the impact error against OUR target from it. The existing
            // belly-correction pipeline (along-track full + lateral leash)
            // then steers against it, and the TrajCal brake/burn block in
            // BellyFlop consumes the smoothed along/cross parts. The impact
            // is stored with its read time and rotated forward with the
            // body spin between Trajectories' own refreshes (~2 s), so a
            // stale read drifts by meters instead of by surface speed
            // (~174 m/s on Kerbin). Note the read-time approximation:
            // Trajectories rotated it back to ITS compute time, at most one
            // of its refresh periods older - absorbed by the EMA+hysteresis
            else if ((!simulate) && TrajCalActive() && (y > noSteerHeight) && (phase != BLControllerPhase.AwaitDeorbit))
            {
                if (!trajAlwaysUpdateSet)
                {
                    trajAlwaysUpdateSet = true;
                    TrajAPI.SetAlwaysUpdate(true); // keep Trajectories computing with its window closed
                    if (!TrajAPI.Available)
                        Log.Info("[TrajCal] Trajectories not found - calibration idle (pure passive behavior)");
                }
                Vector3d? imp = TrajAPI.GetImpactPosition();
                if (imp.HasValue)
                {
                    trajImpact = imp.Value;
                    trajImpactT = t;
                    if (!trajImpactValid)
                        Log.Info(string.Format("[TrajCal] impact data live at t={0:F1}", t));
                    trajImpactValid = true;
                }
                if ((trajImpactValid) && (t - trajImpactT < trajImpactMaxAge))
                {
                    Vector3d impNow = trajImpact;
                    double impAge = t - trajImpactT;
                    if (impAge > 0.01)
                        impNow = (Vector3d)(Quaternion.AngleAxis((float)(impAge * body.angularVelocity.magnitude * Mathf.Rad2Deg), body.angularVelocity.normalized) * (Vector3)trajImpact);
                    Vector3d errRaw = Vector3d.Exclude(up, impNow - tgt_r);
                    trajErrRawMag = errRaw.magnitude;
                    trajErrRawValid = true;
                    // f77: Traj's answer swings multi-km between refreshes
                    // high up (bistable glide solutions - 8.4 km in 4.8 s at
                    // 51 km); the raw error drove the belly correction into
                    // a 67-95 deg chase. Smooth the error vector itself
                    // with an altitude-scaled time constant (2 s low,
                    // 8 s at 80 km+) and derive along/cross from it
                    double tauTC = HGUtils.Clamp(y / 10000, 2, 8);
                    double kTC = trajErrSmoothInit ? HGUtils.Clamp(dt / tauTC, 0, 1) : 1;
                    trajErrSmooth += (errRaw - trajErrSmooth) * kTC;
                    trajErrSmoothInit = true;
                    error = trajErrSmooth;
                    targetError = error.magnitude;
                    Vector3d vhT = Vector3d.Exclude(up, vel_air);
                    if (vhT.magnitude > 10)
                    {
                        Vector3d vhDirT = Vector3d.Normalize(vhT);
                        trajAlong = Vector3d.Dot(error, vhDirT);
                        trajCross = (error - vhDirT * trajAlong).magnitude;
                    }
                }
                else
                {
                    // no usable Traj data: pure passive behavior
                    error = Vector3d.zero;
                    targetError = 0;
                    airbrakeLatched = false;
                    airbrakeWanted = false;
                    trajErrSmoothInit = false;
                    trajErrRawValid = false;
                }
            }
            else if ((!simulate) && (recoveryProfile == "starship"))
            {
                // Starship without TrajCal is PASSIVE by construction (f83
                // cleanup: the own-sim prediction is deleted - there is no
                // third mode). The GUI 被动滑翔 toggle additionally suspends
                // cal sampling (see the aeroLiveCal gates) so test flights of
                // other hulls stay stock-comparable
                targetError = 0; // no prediction - display 0, not a stale fantasy
                // Toggling passive mid-glide with the boards latched would
                // otherwise leave them out forever (the brake block below is
                // skipped). The Flip phase's own pitch-assist write comes
                // later in this method and still wins when it applies
                airbrakeLatched = false;
                airbrakeWanted = false;
            }

            // Below noSteerHeight the full simulation is skipped (its
            // near-ground jitter historically caused noise-chasing), but
            // freezing the error entirely lets the residual horizontal
            // velocity drift the touchdown tens of metres past the freeze
            // point (flight 17: 12m offset at 295m grew to 47m at touchdown
            // with vh~18 m/s at the freeze). Dead-reckon the impact point
            // locally instead: current horizontal offset plus the drift
            // integral vh * y/|vy| (the constant-deceleration time-to-ground
            // is 2y/|vy| and vh brakes roughly linearly over it, so the
            // integral of vh is vh * y/|vy| - flight 18 showed the halved
            // estimate under-leading the aim by ~2x). Position and velocity
            // are clean signals (no sim noise) so the correction stays smooth
            if ((!simulate) && (yG <= noSteerHeight) && (yG > 0) && (vy < 0))
            {
                Vector3d posErr = Vector3d.Exclude(up, r - tgt_r);
                double tauG = yG / Math.Max(5, -vy);
                error = posErr + Vector3d.Exclude(up, vel_air) * tauG;
                targetError = error.magnitude;
                // Low/landed: the panel must show the ACTUAL miss distance,
                // not a stale Traj prediction from before the 200m handoff
                trajErrRawValid = false;
            }

            // Emergency land-anywhere: refresh the target anchor to the
            // ground point under the vessel (1 Hz - the burn-height logic
            // needs the LOCAL ground altitude, not the original target's),
            // then replace the impact error with a horizontal braking
            // demand. Positive sign: the correction accelerates AGAINST the
            // sideways motion and fades to zero as the drift stops (normal
            // mode accelerates along -error to shrink it; with error = vh*tau
            // that IS the brake). Capped so entry-speed pseudo-errors don't
            // saturate the PIDs harder than a real 1 km miss would
            if (emergencyLanding && (!simulate))
            {
                if (t - lastEmergencyAnchor >= 1)
                {
                    lastEmergencyAnchor = t;
                    tgtLatitude = vessel.latitude;
                    tgtLongitude = vessel.longitude;
                    tgtAlt = Math.Max(0, vessel.altitude - vessel.radarAltitude);
                }
                error = Vector3d.Exclude(up, vel_air) * emergencyBrakeTau;
                if (error.magnitude > 1000)
                    error = Vector3d.Normalize(error) * 1000;
                targetError = error.magnitude;
            }

            // TURN AROUND: RCS-only flip to point the nose along the thrust
            // direction the boostback burn will need (engines facing the
            // predicted impact point) BEFORE the burn ignites, so the burn
            // starts aligned instead of thrusting through the flip. In
            // simulation the turn is assumed instant (skip straight to
            // BoostBack) so predictions are unchanged and no sim time is
            // lost coasting
            if (phase == BLControllerPhase.TurnAround)
            {
                if (simulate)
                {
                    phase = BLControllerPhase.BoostBack;
                }
                else
                {
                    Vector3d vhVec = Vector3d.Exclude(up, vel_air);
                    // Aim opposite the predicted impact error, i.e. along the
                    // future boostback thrust direction. This is the optimal
                    // pre-alignment for ANY impact geometry: an impulse
                    // shifts the impact point along its own horizontal
                    // direction, so thrusting toward -error moves the impact
                    // straight onto the target whether it overshoots
                    // downrange (nose anti-velocity, braking), falls short
                    // (nose pro-velocity, accelerating) or misses cross-range
                    // at any angle (30/45/90 degrees off the velocity - the
                    // error vector decomposes automatically). Fall back to
                    // plain horizontal retrograde while the error is too
                    // small for its direction to be meaningful
                    if (error.magnitude > 100)
                        steer = -Vector3d.Normalize(error);
                    else
                        steer = (vhVec.magnitude > 1) ? -Vector3d.Normalize(vhVec) : att; // hold if no horizontal velocity yet
                    throttle = 0;
                    if (turnAroundStart < 0)
                        turnAroundStart = t;
                    attitudeError = HGUtils.angle_between(att, steer);
                    if ((attitudeError < turnAroundCompleteAngle) || (t - turnAroundStart > turnAroundMaxTime))
                    {
                        phase = BLControllerPhase.BoostBack;
                        msg = Localizer.Format("#BoosterGuidance_SwitchedToBoostback");
                    }
                }
            }

            // Set default gains for steering (before ALL phase branches:
            // the starship branches above the falcon9 ones also set it)
            steerGain = 0;

            // AWAIT DEORBIT (starship): stable orbit, deorbit burn not yet
            // flown. Hands off - the user executes their maneuver node (the
            // deorbit scope shows the predicted impact). Watch the periapsis
            // and coast once the orbit dips into the atmosphere
            if (phase == BLControllerPhase.AwaitDeorbit)
            {
                if ((vessel.orbit != null) && (vessel.orbit.PeA < body.atmosphereDepth))
                {
                    phase = BLControllerPhase.EntryCoast;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToEntryCoast");
                }
            }

            // ENTRY COAST (starship): post-burn fall to the atmosphere. The
            // steer default (-vel_air) makes the attitude PD hold the belly
            // toward the velocity vector - into the upcoming wind - from the
            // moment the deorbit burn ends, through any warp (design D10)
            if (phase == BLControllerPhase.EntryCoast)
            {
                // Schedule the AoA here too: exo-atmospheric it is free, and
                // the ship then crosses the interface already at the cruise
                // AoA instead of snapping 30 deg when the air grabs it.
                // Exo-atmospheric q is ~0 -> full brake end of the schedule
                bellyAoACurrent = BellyAoADeg(DynamicPressure(y, vel_air.magnitude, body), Vector3d.Exclude(up, vel_air).magnitude);
                bellyAoASlew = bellyAoACurrent; // f114: keep the slew-limited command state synced outside BellyFlop so the glide capture never steps
                if ((r.magnitude - body.Radius < body.atmosphereDepth) && (vy < 0))
                {
                    // f83 cleanup: the aim burn (powered high-atmosphere
                    // correction off the OWN prediction) is deleted - TrajCal
                    // owns all impact correction now. Straight to the glide.
                    phase = BLControllerPhase.BellyFlop;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToBellyFlop");
                }
            }

            // BELLY FLOP (starship, design D6): steer by tilting the belly
            // normal toward/away from the predicted impact error through the
            // shared correction pipeline (GetSteerCorrection, same
            // error/angle/maxAoA semantics as AeroDescent), with the gain
            // from a 90-degree-AoA force-gradient probe. Absolute cap
            // bellyFlopMaxAoA, faded to zero across the flip regime.
            // starshipCorrectionGain scales the result live (0 = dumb
            // glide, design D7). NOTE the correction sign convention is
            // inherited from falcon9 nose-tilting; a broadside plate's lift
            // response may be opposite - T3 flight calibration checks this
            // first. In simulation the correction is skipped (same
            // convention as AeroDescent) and the sim flies broadside (π/2)
            if (phase == BLControllerPhase.BellyFlop)
            {
                bellyAoACurrent = BellyAoADeg(DynamicPressure(y, vel_air.magnitude, body), Vector3d.Exclude(up, vel_air).magnitude, PathDownDeg(vel_air, up));
                // Trim-flip detection (flight 51): belly attitude stuck >100
                // deg off target for 5 s = the hull has flipped to its
                // tail-first trim (flight 46) and the glide is lost. Stop
                // saturating the flaps and hold pure retrograde, which IS
                // where the hull sits - att_err collapses, surfaces unload,
                // and the flip gate later passes instantly. Reads
                // bellyAttErrLive (last tick's real belly error), NOT
                // attitudeError which was just zeroed above - with the field
                // zeroed every tick this detector was dead code and glideLost
                // only ever came from the sim's q-latch (removed in f61:
                // wrong per-craft guess every flight)
                if (!simulate)
                {
                    // pilotBurnAttitude = the ship is DELIBERATELY off the belly
                    // target (flying +/-vel_air for the pilot's burn) - a prograde
                    // burn reads ~180deg here and must not latch glideLost
                    // f102: bellyAttAcquired - the detector arms only once the
                    // swing-in has completed (error <45 deg at least once), so
                    // enabling guidance mid-swing can no longer latch glideLost
                    if ((!bellyAttAcquired) && (bellyAttErrLive < 45))
                        bellyAttAcquired = true;
                    // f108 B: 15 s grace after any pilot burn - the swing-back
                    // from the tail-first burn attitude sits >100 deg for
                    // ~10 s at 33 km (thin air, slow rotation) and latched
                    // glideLost right as the ship arrived back on the belly
                    // (f108: correction frozen 31 km -> flip, cross parked
                    // 556 m, user manual saves at 15.6/4.5 km)
                    if (pilotBurnAttitude)
                        pilotBurnGraceUntil = t + 15;
                    if ((bellyAttErrLive > 100) && (!pilotBurnAttitude) && (bellyAttAcquired) && (t > pilotBurnGraceUntil))
                    {
                        if (bellyDepartT < 0)
                            bellyDepartT = t;
                        if ((!glideLost) && (t - bellyDepartT > 5))
                        {
                            glideLost = true;
                            msg = "Tail-first trim won - glide lost, holding retrograde";
                        }
                    }
                    else
                        bellyDepartT = -1;
                    // f108 A: auto-unlatch - a REAL tail-first trim lock
                    // (flight 51: hull aero-locked) never re-acquires the
                    // belly attitude, but a false latch (slow vacuum swing-in
                    // at enable, post-burn swing-back) always does within
                    // seconds. Until now there was NO in-flight release -
                    // only a guidance re-enable reset the latch, which is why
                    // the f108 user got correction back at 46 km but lost it
                    // for good at 31 km
                    if (glideLost)
                    {
                        if (bellyAttErrLive < 45)
                        {
                            if (bellyReacquireT < 0)
                                bellyReacquireT = t;
                            if (t - bellyReacquireT > 2)
                            {
                                glideLost = false;
                                bellyDepartT = -1;
                                bellyReacquireT = -1;
                                Log.Info(string.Format("[TrajCal] GLIDE RE-ACQUIRED t={0:F1} alt={1:F0} - glideLost released, correction resumed", t, y));
                            }
                        }
                        else
                            bellyReacquireT = -1;
                    }
                }
                bellyBraking = false; // re-requested by the TrajCal burn below every tick it fires
                // f82: this reset must use the ACTIVE brake ceiling - TrajCal's
                // ceiling (45 km) sits ABOVE the legacy 40 km, so at 40-45 km
                // the latch was reset every tick: 345x per-tick "AIRBRAKES OUT"
                // log spam and the TrajCal release branch went dead (the latch
                // was always false at its check), holding the boards out until
                // the arm failed instead of releasing at half the threshold
                if ((!simulate) && ((phase != BLControllerPhase.BellyFlop) || (y >= trajCalBrakeMaxAlt)))
                {
                    airbrakeLatched = false;
                }
                // f83 cleanup (user directive 2026-09-01: 只保留验证成功的体系):
                // the legacy own-sim-prediction brake machinery (trend/direction
                // arming, stopShort, lateral gate, boards-extend suppression,
                // f66b positional engine brake) is DELETED - under TrajCal it
                // was all dead code (gated !TrajCalActive), and the own-sim
                // prediction it consumed is gone too. Overshoot handling now
                // lives ONLY in the TrajCal block below (Traj red-mark driven)

                // Traj-driven calibration (f76 user request): overshoot per
                // TRAJECTORIES' impact -> airbrakes (same anti-strobe
                // discipline as the legacy brake: deploy above the band,
                // release below half, minimum hold, boards-extend-glide
                // suppression per the saved cal bands); impact SHORT -> a
                // modest burn at the belly attitude extends the glide. The
                // burn is gated to the glide regime (AoA <= 45, thrust
                // ~prograde) with the pilot-burn q ceiling, room above the
                // flip band, and a dV budget so the landing reserve is
                // untouched. LIVE ONLY - no sim clone runs this
                if ((!simulate) && TrajCalActive())
                {
                    double qTC = DynamicPressure(y, vel_air.magnitude, body);
                    bool tcDataOk = (trajImpactValid) && (t - trajImpactT < trajImpactMaxAge);
                    // boards-extend-glide suppression WITH HYSTERESIS (f77:
                    // live Ab-band kL sampling drifted across the 1.3x
                    // threshold mid-brake and retracted the boards for
                    // 0.2 s): engage above +0.05, release below -0.05,
                    // hold state in between
                    int bTC = AeroCalBin(qTC);
                    if (!CalBinsBad("trajcal", bTC))
                    {
                        if ((aeroCalSamplesAbQ[bTC] >= calBinMinSamples) && (aeroCalSamplesQ[bTC] >= calBinMinSamples))
                        {
                            double ratioTC = aeroCalLiftAbQ[bTC] / Math.Max(0.01, aeroCalLiftQ[bTC]);
                            if (ratioTC > airbrakeMaxLiftRatio + 0.05)
                                trajBoardsExtend = true;
                            else if (ratioTC < airbrakeMaxLiftRatio - 0.05)
                                trajBoardsExtend = false;
                        }
                    }
                    // f109 terminal zone (user: "下落阶段基本已经没有足够的水平
                    // 速度,下落阶段不算滑翔了,不能再相信Traj的红标"): the belly is
                    // FALLING, not gliding, and the mark carries its ~2.2 km
                    // low-speed long bias there (f107 calibration) - f109 braked
                    // on a +754 m phantom at 4.8 km and landed 1.4 km short. f114
                    // widened it 6 km -> 12 km: the phantom-long inflation (+3~6
                    // km in the 29->10 km band, the Traj-profile mach-5.5 kink
                    // family) fired airbrakes at 26.7/19.9/12 km and an 11 s
                    // AOAMOD broadside at 9.9 km on inflated readings -> landed
                    // 1.1 km short AGAIN. No mark-driven braking in this zone:
                    // boards retract via this forced path, the aoaMod governor is
                    // banned below; the 2 km slam + flip + landing-burn
                    // translation own whatever is actually left
                    double vhTC = Vector3d.Exclude(up, vel_air).magnitude;
                    bool trajTerminal = (vhTC < 75) || (y < 12000);
                    if ((!tcDataOk) || (y >= trajCalBrakeMaxAlt) || (y < trajCalBrakeMinAlt) || (trajTerminal))
                    {
                        if ((trajTerminal) && (airbrakeLatched))
                            Log.Info(string.Format("[TrajCal] AIRBRAKES IN (terminal, mark untrusted) t={0:F1} alt={1:F0} vh={2:F0}", t, y, vhTC));
                        airbrakeLatched = false;
                        airbrakeWanted = false;
                        trajCalOverSinceT = -1;
                    }
                    else
                    {
                        // f77/f78: high up, Traj's impact answer is phantom-
                        // grade noise (a +7.6k/+33k "overshoot" at 40 km
                        // braked ships that were really going SHORT - f78
                        // landed 18.2 km short because of it). The deploy
                        // threshold scales with altitude and must persist
                        double deployThresh = Math.Max(trajCalOverError, trajCalOverAltFrac * y);
                        if ((trajAlong > deployThresh) && (!trajBoardsExtend))
                        {
                            if (trajCalOverSinceT < 0)
                                trajCalOverSinceT = t;
                        }
                        else
                            trajCalOverSinceT = -1;
                        if (!airbrakeLatched)
                        {
                            if ((trajCalOverSinceT >= 0) && (t - trajCalOverSinceT >= trajCalBrakeArmTime)
                                && ((trajCalRetractT < 0) || (t - trajCalRetractT >= trajCalBrakeRetractHold)))
                            {
                                airbrakeLatched = true;
                                airbrakeLatchedT = t;
                                Log.Info(string.Format("[TrajCal] AIRBRAKES OUT t={0:F1} alt={1:F0} along={2:F0} cross={3:F0} q={4:F0} thresh={5:F0}", t, y, trajAlong, trajCross, qTC, deployThresh));
                            }
                        }
                        else if ((t - airbrakeLatchedT >= trajCalBrakeMinHold) && ((trajAlong < trajCalReleaseFrac * deployThresh) || ((trajBoardsExtend) && (trajAlong < deployThresh))))
                        {
                            airbrakeLatched = false;
                            trajCalRetractT = t;
                            // f79: boardsExtend may release the boards only when there is NO
                            // active overshoot - at 10.4 km it retracted them during a REAL
                            // 63.7 km overshoot (kLqab bin polluted by flip-phase aoa=85
                            // samples drifting across the 1.35 ratio). A lift-glide release
                            // is a fine-tuning decision; an active overshoot outranks it.
                            Log.Info(string.Format("[TrajCal] AIRBRAKES IN t={0:F1} alt={1:F0} along={2:F0} relThresh={3:F0}", t, y, trajAlong, trajCalReleaseFrac * deployThresh));
                        }
                        airbrakeWanted = airbrakeLatched;
                    }
                    // q-gate hysteresis (trajCalBurnQHyst): once burning,
                    // hold until q exceeds maxQ+hyst so thrust feedback
                    // across the gate cannot strobe the engines (f81)
                    double tcBurnQLimit = trajCalBurnWasOn ? (trajCalBurnMaxQ + trajCalBurnQHyst) : trajCalBurnMaxQ;
                    // f93: along-threshold hysteresis, same shape as the q-gate
                    // above - trigger needs the full shortError, but once burning
                    // hold until the shortfall is under exitShort so the mark gets
                    // driven to target in ONE sustained burn instead of ~30 blips
                    double tcBurnAlongLimit = trajCalBurnWasOn ? -trajCalBurnExitShort : -trajCalShortError;
                    bool tcBurn = (tcDataOk) && (trajAlong < tcBurnAlongLimit)
                        && (bellyAoACurrent <= 45) && (qTC < tcBurnQLimit)
                        && (y > trajCalBurnMinAlt) && (y < trajCalBurnMaxAlt)
                        && (trajCalSpentDv < trajCalBurnMaxDv);
                    if (tcBurn)
                    {
                        if (!trajCalBurnWasOn)
                            Log.Info(string.Format("[TrajCal] BURN ON t={0:F1} alt={1:F0} short={2:F0} q={3:F0} aoa={4:F0}", t, y, -trajAlong, qTC, bellyAoACurrent));
                        throttle = trajCalBurnThrottle;
                        bellyBraking = true; // hold the plain belly attitude through the burn
                        trajCalSpentDv += (amin + throttle * (amax - amin)) * dt;
                    }
                    else if (trajCalBurnWasOn)
                        Log.Info(string.Format("[TrajCal] BURN OFF t={0:F1} alt={1:F0} along={2:F0} spentDv={3:F1} dataOk={4}", t, y, trajAlong, trajCalSpentDv, tcDataOk ? 1 : 0));
                    trajCalBurnWasOn = tcBurn;
                    // f85 user request 暴力减速: low-altitude max-drag
                    // maneuver. f122 REBUILD (user-picked 速度走廊): the
                    // trigger is now GEOMETRIC and mark-free. The old trigger
                    // (ship in the 2 km circle + red mark along > 300/1500)
                    // chased the mark inside its own untrusted zone - VB only
                    // works below 10 km, exactly where the mark carries a
                    // 2-5 km terminal long-bias. f119: slammed vh 98->5 on a
                    // phantom, leaf-rebounded 1.8 km short. f122: the approach
                    // was already perfect (vh=107 closing, dist=1940, att=1deg
                    // - natural glide decel would have landed it dead on:
                    // (107+0)/2*36s = 1926m ~= 1940m), VB chased along=+6065,
                    // killed vh to 9, dead-cone leaf glide reversed the ship
                    // 800m+ backwards, landed 1598m off. The corridor instead:
                    //   overRun = vh*tFall - 0.5*dNat*tFall^2 + sail(vh) - dist
                    // = how far PAST the target the ship lands if nothing
                    // changes, including the post-flip sail (f124). dNat credits the belly glide's natural
                    // deceleration (f122 measured ~1.5 m/s^2 at the schedule
                    // AoA; the full 85-deg slam gives ~3.4). Arm only when
                    // overRun > 600m (below that the landing burn + v2g
                    // finish it), and the AoA is PROPORTIONAL to the excess
                    // instead of the binary 85-deg slam - a thermostat, not a
                    // hammer. True overshoots still fire (f101: vh=142 at
                    // 1.1 km out -> overRun 1810 -> full slam). The vh floor
                    // (25-35 toward target) remains the corridor's lower
                    // bound, so the dead cone can never be re-created.
                    double shipTgtDist = Vector3d.Exclude(up, r - tgt_r).magnitude;
                    double vbTFall = Math.Max(500, y - flipAltMax) / Math.Max(30, -vy);
                    // f124 (user-picked A+B): the corridor's horizon ended at
                    // the FLIP - it released at 4.6km/626m-out with vh=88
                    // because overRun(to flip)=-5, then the ship sailed 2.7km
                    // THROUGH the flip (62m from the target at 5.4km -> 2771m
                    // past; the LandingBurn float kills only ~2.2 m/s^2). The
                    // flip is not a wall: add the post-flip sail = flip
                    // maneuver time at speed + the burn's kill distance,
                    // estimated from the speed the glide still HAS at the
                    // flip (vh - dNat*tFall), so the corridor brakes to a
                    // genuinely stoppable arrival instead
                    double vbVhFlip = Math.Max(vhTC - vbDriftDecel * vbTFall, 0);
                    double vbOverRun = vhTC * vbTFall - 0.5 * vbDriftDecel * vbTFall * vbTFall
                        + vbVhFlip * vbFlipSail + (vbVhFlip * vbVhFlip) / (2 * vbPostKill) - shipTgtDist;
                    bool vbArm = (trajCalViolentBrake) && (y < trajCalViolentAlt)
                        && (qTC < trajCalViolentMaxQ)
                        && ((violentBrakeAbortT < 0) || (t - violentBrakeAbortT > 5))
                        && (vbOverRun > trajCalViolentOverRun)
                        // f119 (user-picked 方案B): two arm guards so the slam
                        // can never CREATE the leaf-in-the-wind state it then
                        // cannot fix - (1) never arm below vh=40 (below the
                        // f94 noise cone ~70 there is no directional authority
                        // to protect; braking 40->5 just makes the hull a leaf,
                        // and f119's kill to vh=5 is exactly what set up the
                        // wrong-way rebound), (2) never arm at/below the flip
                        // doorstep (f119 re-armed at 4.1 km, 1.3 s before the
                        // flip started - a slam there can only disrupt the flip)
                        && (vhTC >= 40)
                        && (y > flipAltMax + 500);
                    // HOLD while the geometry still says hot (overRun > 0):
                    // overflying the target still fast is exactly when the
                    // brake must keep working
                    bool vbHold = (trajCalViolentBrake) && (y < trajCalViolentAlt)
                        && (qTC < trajCalViolentMaxQ + 1000)
                        && (vbOverRun > 0);
                    if (!violentBrakeLatched)
                    {
                        if (vbArm)
                        {
                            if (violentBrakeSinceT < 0)
                                violentBrakeSinceT = t;
                            if (t - violentBrakeSinceT >= 0.5)
                            {
                                violentBrakeLatched = true;
                                violentBrakeDepartT = -1;
                                Log.Info(string.Format("[TrajCal] VIOLENT-BRAKE ON t={0:F1} alt={1:F0} shipDist={2:F0} overRun={3:F0} q={4:F0} vh={5:F0} -> proportional brake", t, y, shipTgtDist, vbOverRun, qTC, vhTC));
                            }
                        }
                        else
                            violentBrakeSinceT = -1;
                    }
                    else
                    {
                        if (bellyAttErrLive > 45)
                        {
                            if (violentBrakeDepartT < 0)
                                violentBrakeDepartT = t;
                        }
                        else
                            violentBrakeDepartT = -1;
                        if ((violentBrakeDepartT >= 0) && (t - violentBrakeDepartT > 2))
                        {
                            violentBrakeLatched = false;
                            violentBrakeAbortT = t;
                            Log.Info(string.Format("[TrajCal] VIOLENT-BRAKE ABORT t={0:F1} alt={1:F0} shipDist={2:F0} overRun={3:F0} q={4:F0} bellyErr={5:F0} - attitude departure, 5s cooldown", t, y, shipTgtDist, vbOverRun, qTC, bellyAttErrLive));
                        }
                        else if (vhTC < 40)
                        {
                            // f119 (方案B): the mark that armed the slam lives in
                            // the terminal phantom zone, so the along exit alone
                            // over-brakes - f119 killed vh 98->5, the hull became
                            // a leaf, departed, and rebounded in the WRONG
                            // direction. Below vh=40 there is no authority left
                            // to protect; hand the ship back to the steer/governor
                            // with enough speed to still mean something. No
                            // cooldown needed: vbArm itself now requires vh>=40
                            violentBrakeLatched = false;
                            Log.Info(string.Format("[TrajCal] VIOLENT-BRAKE OFF (vh<40) t={0:F1} alt={1:F0} shipDist={2:F0} overRun={3:F0} q={4:F0} vh={5:F0} - releasing with directional authority intact", t, y, shipTgtDist, vbOverRun, qTC, vhTC));
                        }
                        else if (!vbHold)
                        {
                            violentBrakeLatched = false;
                            Log.Info(string.Format("[TrajCal] VIOLENT-BRAKE OFF t={0:F1} alt={1:F0} shipDist={2:F0} overRun={3:F0} q={4:F0}", t, y, shipTgtDist, vbOverRun, qTC));
                        }
                    }
                    if (violentBrakeLatched)
                    {
                        // f122: PROPORTIONAL brake, not the binary slam - AoA
                        // scales from the schedule value to bellyBrakeAoA as
                        // overRun grows 0->1500m. Thermostat: braking shrinks
                        // vh, overRun falls, the AoA relaxes before the dead
                        // cone (the vh>=40 arm guard + vh<40 OFF stay as the
                        // hard backstop)
                        double vbExcess = HGUtils.Clamp(vbOverRun / vbOverRunFull, 0, 1);
                        bellyAoACurrent = bellyAoACurrent + vbExcess * (bellyBrakeAoA - bellyAoACurrent);
                        if (!trajBoardsExtend)
                            airbrakeWanted = true; // 暴力 = every drag device out, unless boards add lift on this hull
                    }
                    // f95 proportional AoA governor (user design - see the
                    // field comment): inside trajCalAoaModDist HORIZONTAL of
                    // the target (user: 5km指水平距离不是高度) the overshoot is
                    // killed by RAISING THE NOSE proportionally to the Traj
                    // mark's along error, not by the 85-deg slam - the ship
                    // keeps enough vh to stay steerable, the mark is flown
                    // onto the target, and the AoA relaxes back to the glide
                    // schedule as the overshoot dies (and re-engages if the
                    // glide re-extends the mark - the user's sawtooth). The
                    // aoaModBraking flag suspends the belly correction while
                    // the AoA is elevated (user: no lateral correction during
                    // the pull-up - the frame tilt feeds the f94 spin loop).
                    // The y<20000 ceiling keeps the governor out of the
                    // high-altitude phantom-mark regime (>29 km the mark can
                    // read tens of km long - f95/f77/f78); down here the mark
                    // is honest.
                    bool aoaModNewBraking = false;
                    if ((trajCalAoaMod) && (tcDataOk) && (shipTgtDist < trajCalAoaModDist) && (y < 20000) && (!violentBrakeLatched) && (!bellyBraking) && (!vhFloorActive) && (!trajTerminal))
                    {
                        double schedAoA = bellyAoACurrent; // the q/vh/path schedule value computed above
                        if (aoaModAoA < 0)
                        {
                            // f96 headroom guard: when the schedule is already
                            // AT the brake cap (terminal band runs 85 deg) or
                            // the mark is not long, the governor can add
                            // nothing - engaging anyway spammed ENGAGE every
                            // tick (engage -> relax-check -> reset loop)
                            double govCheck = (trajAlong > 0) ? Math.Min(bellyBrakeAoA, schedAoA + trajCalAoaModGain * trajAlong) : schedAoA;
                            if (govCheck > schedAoA + 0.5)
                            {
                                aoaModAoA = schedAoA; // engage from the live schedule - no step
                                Log.Info(string.Format("[TrajCal] AOAMOD ENGAGE t={0:F1} alt={1:F0} shipDist={2:F0} along={3:F0} schedAoA={4:F0}", t, y, shipTgtDist, trajAlong, schedAoA));
                            }
                        }
                        if (aoaModAoA >= 0) // engaged (just now or earlier)
                        {
                            double govAoA = (trajAlong > 0) ? Math.Min(bellyBrakeAoA, schedAoA + trajCalAoaModGain * trajAlong) : schedAoA;
                            aoaModAoA += HGUtils.Clamp(govAoA - aoaModAoA, -trajCalAoaModSlew * dt, trajCalAoaModSlew * dt);
                            if (aoaModAoA > schedAoA + 0.5)
                            {
                                bellyAoACurrent = aoaModAoA;
                                aoaModNewBraking = aoaModAoA > schedAoA + 2; // correction off while clearly above the schedule
                            }
                            else
                                aoaModAoA = -1; // fully relaxed - back on the schedule, re-engage fresh next overshoot
                        }
                    }
                    else if (aoaModAoA >= 0)
                    {
                        Log.Info(string.Format("[TrajCal] AOAMOD OFF t={0:F1} alt={1:F0} shipDist={2:F0} along={3:F0}", t, y, shipTgtDist, trajAlong));
                        aoaModAoA = -1;
                    }
                    if (aoaModNewBraking != aoaModBraking)
                        Log.Info(string.Format("[TrajCal] AOAMOD {0} t={1:F1} alt={2:F0} shipDist={3:F0} along={4:F0} aoa={5:F0}", aoaModNewBraking ? "BRAKE(corr off)" : "GLIDE(corr on)", t, y, shipTgtDist, trajAlong, aoaModAoA));
                    aoaModBraking = aoaModNewBraking;
                    // f107 vh floor (user picked A): below 10km, never brake
                    // the horizontal speed under what the remaining distance
                    // needs - floor = dist/60 (clamped 25..90 m/s), i.e. ~60 s
                    // of glide reach at the current distance. On the floor the
                    // governor is disengaged (blocked above) and the AoA is
                    // capped at 55 deg so the glide keeps/recovers its reach;
                    // any REAL excess is owned by the 2km slam and the flip,
                    // which can always kill speed - nothing can buy it back.
                    // Not applied while the slam owns the attitude
                    // (violentBrakeLatched) or a tcBurn is flying (bellyBraking).
                    if ((phase == BLControllerPhase.BellyFlop) && (y < 10000) && (!violentBrakeLatched) && (!bellyBraking))
                    {
                        double vhCur = Vector3d.Exclude(up, vel_air).magnitude;
                        // f109: the floor must deliver the ship to the target
                        // BY FLIP ALTITUDE - the glide ends at the flip
                        // (flipAltMax ~4 km), not at the ground. dist/60 assumed
                        // a 60 s glide to the surface and demanded only 54 m/s
                        // at 7.7 km where ~88 was needed - f109 arrived at the
                        // flip 1.4 km short. floor = dist / time-to-fall-to-flip
                        double tToFlip = Math.Max(500, y - flipAltMax) / Math.Max(30, -vy);
                        double vhFloorRaw = shipTgtDist / tToFlip;
                        double vhFloor = HGUtils.Clamp(vhFloorRaw, 25, 90);
                        // f119 sanity gates on the LATCH (user-picked 方案A):
                        // the floor is a keep-reach tool, so only let it bite
                        // when it can actually help. f119 latched it 6.4 s
                        // before the flip (tToFlip tiny), with the raw demand
                        // clamped at 90 (unreachable by construction), while
                        // vh pointed AWAY from the target (post-VB-departure
                        // rebound) - the 55-deg cap then ACCELERATED the
                        // wrong-way speed 38->84 and walked the ship from 637 m
                        // short to 1801 m short. Gates:
                        //   time:      >=15 s of glide left, or nothing the
                        //              floor does can change the flip state
                        //   reachable: raw demand within the 90 clamp
                        //   direction: vh actually carries TOWARD the target
                        //              (keep-reach pointed away = accelerate
                        //              away); below vh 10 the direction is
                        //              noise, so don't latch at all
                        Vector3d vhVecF = Vector3d.Exclude(up, vel_air);
                        Vector3d tgtHF = Vector3d.Exclude(up, tgt_r - r);
                        bool vhTowardTgt = (vhCur >= 10) && (tgtHF.magnitude > 1) && (Vector3d.Dot(vhVecF, tgtHF) > 0);
                        bool floorSane = (tToFlip >= 15) && (vhFloorRaw <= 90) && (vhTowardTgt);
                        if ((vhCur < vhFloor) && (floorSane))
                        {
                            if (!vhFloorActive)
                            {
                                vhFloorActive = true;
                                Log.Info(string.Format("[TrajCal] VH FLOOR ON t={0:F1} alt={1:F0} dist={2:F0} vh={3:F0} floor={4:F0} along={5:F0} aoa={6:F0} -> cap 55", t, y, shipTgtDist, vhCur, vhFloor, trajAlong, bellyAoACurrent));
                            }
                            if (aoaModAoA >= 0)
                            {
                                Log.Info(string.Format("[TrajCal] AOAMOD OFF (vh floor) t={0:F1} alt={1:F0} shipDist={2:F0} along={3:F0}", t, y, shipTgtDist, trajAlong));
                                aoaModAoA = -1;
                                aoaModBraking = false;
                            }
                        }
                        else if ((vhFloorActive) && (vhCur > 15) && (tgtHF.magnitude > 1) && (Vector3d.Dot(vhVecF, tgtHF) < 0))
                        {
                            // f119: latched while valid, then the direction
                            // flipped against us (post-departure rebound can
                            // do this in one tick) - the 55-deg cap is now
                            // accelerating AWAY from the target. Release; the
                            // broadside schedule brakes the wrong-way drift
                            vhFloorActive = false;
                            Log.Info(string.Format("[TrajCal] VH FLOOR OFF (direction lost) t={0:F1} alt={1:F0} dist={2:F0} vh={3:F0} floor={4:F0}", t, y, shipTgtDist, vhCur, vhFloor));
                        }
                        else if ((vhFloorActive) && (vhCur > vhFloor + 15) && (y > 6000)) // f109: no release in the terminal zone - the floor owns speed-keeping all the way to the flip
                        {
                            vhFloorActive = false;
                            Log.Info(string.Format("[TrajCal] VH FLOOR OFF t={0:F1} alt={1:F0} dist={2:F0} vh={3:F0} floor={4:F0}", t, y, shipTgtDist, vhCur, vhFloor));
                        }
                        // f114: the cap rides the LATCH, not the instantaneous
                        // vh<floor comparison - f114 had vh hovering exactly on
                        // the floor line, so the AoA target bang-banged
                        // 55<->85 EVERY TICK (cap on the tick below the floor,
                        // terminal schedule on the tick above) and the belly
                        // cascade chased the square wave into the pre-flip
                        // multi-axis wag the user saw ("机头左右摇摆"). Latched
                        // = capped; the +15 hysteresis above owns the release
                        if (vhFloorActive)
                            bellyAoACurrent = Math.Min(bellyAoACurrent, 55);
                    }
                }
                // f114: slew-limit the belly AoA command. The schedule itself
                // is smooth in q, but the governor engage (18->85 step), the
                // vh-floor cap engage/release and any re-capture step the
                // target tens of degrees in one tick - each step re-excites
                // the cascade. Walk the command at bellyAoACmdSlewRate deg/s
                // instead. EntryCoast keeps the slew state synced to its own
                // schedule.
                // f118: the violent brake is no longer exempt - f114 exempted
                // it ("delayed drag is lost braking"), but the one-tick
                // 50->85 slam at 8.6 km ran the frame ahead of the hull
                // (rate cap 20 deg/s), spun the roll channel to -13 deg/s,
                // overshot AoA to 99.8 and baked ~13 deg of roll into the
                // 4 km broadside hold (att_err 34 spike). 30 deg/s reaches
                // full brake in ~1.2 s - a negligible fraction of the ~1 km
                // stop - without decoupling the cascade
                double slewRate = violentBrakeLatched ? bellyAoACmdSlewRate * 3 : bellyAoACmdSlewRate;
                if (bellyAoASlewT < 0)
                {
                    bellyAoASlew = bellyAoACurrent;
                    bellyAoASlewT = t;
                }
                else
                {
                    double dtAoA = HGUtils.Clamp(t - bellyAoASlewT, 0.005, 0.2);
                    bellyAoASlewT = t;
                    double dAoA = bellyAoACurrent - bellyAoASlew;
                    double maxStep = slewRate * dtAoA;
                    bellyAoASlew += HGUtils.Clamp(dAoA, -maxStep, maxStep);
                    bellyAoACurrent = bellyAoASlew;
                }
                if (!simulate)
                {
                    if ((glideLost) || (bellyBraking) || (violentBrakeLatched) || ((aoaModBraking) && (TrajCalActive())))
                    {
                        // retrograde hold: the trim attitude (or the burn
                        // attitude) - the belly correction is either lost
                        // with the glide or deliberately suspended for the
                        // brake. f94: violent brake included - the drop at
                        // bellyBrakeAoA is a blunt max-drag maneuver whose
                        // targeting is handled by the along-based release;
                        // a correction tipped into the frame there (corrCap
                        // = AoA-2 = 83 deg at entry, vh still >80 so the f84
                        // fade was 1) just sweeps the command on an already
                        // marginal attitude. f95: aoaModBraking included -
                        // user: "抬升机头减速阶段不要有任何横向修正,不然会
                        // 出现之前的自旋情况,滑行的时候正常横向修正就行"
                        steer = -Vector3d.Normalize(vel_air);
                    }
                    else
                    {
                    double bellyKp = aeroDescentSteerKp * CalculateBellySteerGain(vel_air, r, totalMass, bellyAoACurrent);
                    if (double.IsNaN(bellyKp) || double.IsInfinity(bellyKp))
                        bellyKp = 0; // denormal aero forces at the interface -> Inf gain -> NaN steer (flight 42 crash)
                    pid_belly.kp = Math.Min(bellyKp, bellyGainMaxKp);
                    steerGain = pid_belly.kp;
                    // f64 lateral leash: correct only the along-track part of
                    // the impact error at full strength (see
                    // bellyLateralGain comment - the cross-track loop
                    // positive-feedbacks on the real hull)
                    Vector3d errSteer = error;
                    Vector3d vhSteer = Vector3d.Exclude(up, vel_air);
                    if (vhSteer.magnitude > 10)
                    {
                        Vector3d vhDir = Vector3d.Normalize(vhSteer);
                        double errAlong = Vector3d.Dot(error, vhDir);
                        Vector3d errCross = error - vhDir * errAlong;
                        // f87 lateral watchdog around the pulse correction
                        // f118: the whole lateral engage/fire path is banned
                        // above bellyLateralMaxAlt - up there the cross
                        // reading is phantom-grade noise AND thin air gives
                        // a roll tap no authority, so f118 pulsed at max
                        // cadence for 2.5 min chasing readings that swung
                        // 592->1050->115->976 (user: 高空一直在抽搐,
                        // 高层大气稀薄这样校准没用). Gating ENGAGE (not just
                        // fire) also stops the engage/re-arm log churn on
                        // the noise deadband crossings
                        bool latAllowed = false;
                        if ((!lateralAbort) && (y < bellyLateralMaxAlt))
                        {
                            double cm = errCross.magnitude;
                            if (cm > bellyLateralDeadband)
                            {
                                if (lateralEngageCross < 0)
                                {
                                    lateralEngageCross = cm;
                                    Log.Info(string.Format("[TrajCal] LATERAL ENGAGE t={0:F1} alt={1:F0} cross={2:F0}", t, y, cm));
                                    latAllowed = true;
                                }
                                else if (cm > lateralEngageCross + 2000)
                                {
                                    // cross-track GREW >2km while correcting =
                                    // the f64 positive feedback is back
                                    lateralAbort = true;
                                    Log.Info(string.Format("[TrajCal] LATERAL ABORT t={0:F1} alt={1:F0} cross={2:F0} engageCross={3:F0} - lateral correction cut for this flight", t, y, cm, lateralEngageCross));
                                }
                                else
                                    latAllowed = true;
                            }
                            else
                                lateralEngageCross = -1; // re-arm inside the deadband
                        }
                        // f87/f88 lateral SIGN (still governs the pulse
                        // direction): the correction tips the whole belly
                        // frame toward the error (steer = -vel - corr
                        // below), which is right along-track (tips the AoA)
                        // but backwards cross-track: yawing the frame toward
                        // the error yaws the LIFT vector toward the error,
                        // the path curves toward the error and the impact
                        // runs away (f88 flight: cross 577m -> 2577m under
                        // correction, watchdog cut it at 18km). The lateral
                        // component must enter with the OPPOSITE sign - yaw
                        // the frame AWAY from the error so the lift leans
                        // the path back toward the target. If the next
                        // flight's watchdog trips again, this hull's lateral
                        // response is not lift-dominated at all
                        // f98: the along-track error goes through the pid
                        // ALONE (a leashed lateral share of errSteer was
                        // geometrically swamped by any big along error)
                        errSteer = vhDir * errAlong;
                        // f106/f114: the lateral term is a discrete PULSE
                        // (user technique) instead of a held proportional
                        // tilt - tap, hands off, re-read the settled mark.
                        // f114: the pulse now ROLLS (bellyRollPulseDeg bumps
                        // the rollOffset the core passes to starshipAtt)
                        // instead of yawing the steer frame 4 deg: at belly
                        // AoA a roll rotates the whole lift vector (banked
                        // turn - the user's own corrections are pure roll
                        // taps), the 4-deg yaw only sideslipped.
                        // Sign (world-frame derivation): +dRollOffset rotates
                        // the target frame about noseT (qTgt = L*AngleAxis(
                        // -ro,up)), so the physical belly normal swings
                        // toward wingT = bellyT x noseT; the wind pushes the
                        // wetted belly OPPOSITE its outward normal (flat
                        // plate: F ~ -k*n), so +dRoll accelerates the ship
                        // toward -wingT. To curve the path toward the target
                        // (-errCross, since error = impact - target) take
                        // sign(dRoll) = sign(dot(wingT, errCross)) - the same
                        // lean-the-belly-away-from-the-target physics as the
                        // f88/f90/f91-verified yaw sign. The f87 watchdog
                        // above remains the backstop if the hull disagrees
                        if (t < lateralPulseUntil)
                            bellyRollPulseDeg = lateralRollSign * bellyLateralRollDeg; // fade applied below with the along correction's
                        else if ((latAllowed) && (y < bellyLateralMaxAlt) && (errCross.magnitude > 1) && (t >= lateralCooldownUntil))
                        {
                            Vector3d pBT, pNT;
                            StarshipAttitudeController.BellyFrame(Vector3d.Normalize(vel_air), up, bellyAoACurrent, out pBT, out pNT);
                            if (pNT.magnitude > 0.05) // near-vertical wind: no bank reference - retry next tick
                            {
                                lateralRollSign = (Vector3d.Dot(Vector3d.Cross(pBT, pNT), errCross) > 0) ? 1 : -1;
                                lateralPulseUntil = t + lateralPulseTime;
                                lateralCooldownUntil = lateralPulseUntil + lateralPulseGap;
                                bellyRollPulseDeg = lateralRollSign * bellyLateralRollDeg;
                                Log.Info(string.Format("[TrajCal] LATERAL ROLL PULSE t={0:F1} alt={1:F0} cross={2:F0} bank={3:F0} dur={4:F1}s", t, y, errCross.magnitude, bellyRollPulseDeg, lateralPulseTime));
                            }
                        }
                    }
                    double ang = pid_belly.Update(errSteer.magnitude, dt);
                    double fade = HGUtils.Clamp((yG - flipAltMax) / Math.Max(1, bellyCorrectionFadeAlt - flipAltMax), 0, 1);
                    // f84: also fade the correction as the horizontal
                    // airspeed dies - at near-vertical fall the steering
                    // frame is degenerate (f83 wallow) and there is no
                    // directional lift left to steer with anyway
                    fade *= HGUtils.Clamp(vhSteer.magnitude / Math.Max(1, bellyCorrectionVhFull), 0, 1);
                    // f84 user directive: 低空允许舵面继续校准落点，但合成后
                    // 实际攻角不能为负. The correction tilts the whole belly
                    // frame off the velocity vector, so the pitch-plane part
                    // of the tilt subtracts from the commanded AoA directly;
                    // cap the total correction angle 2 deg short of the
                    // scheduled AoA (the nose floor keeps that AoA >= the
                    // path steepness, so this cap only binds on shallow
                    // slow approaches - exactly where a negative-AoA dig
                    // would otherwise live)
                    double corrCap = Math.Min(bellyFlopMaxAoA * fade, Math.Max(0, bellyAoACurrent - 2));
                    Vector3d corr = GetSteerCorrection(errSteer, ang, corrCap, omegaLat);
                    bellyRollPulseDeg *= fade; // f114: the roll pulse rides the same aero-authority fades as the along correction
                    // T3 sign hypothesis #2 (flight 45): with the correction
                    // ADDED the impact error grew monotonically 10->153km
                    // while attitude tracking was good - the broadside
                    // plate's lift response is opposite the falcon9
                    // nose-tilt convention, so SUBTRACT. If the next flight
                    // diverges faster instead, flip this back
                    steer = -Vector3d.Normalize(vel_air) - corr * starshipCorrectionGain;
                    }
                }
                // Flip trigger (design D8, revised after flight 44): the old
                // rule credited (flipTime + igniteDelay) of fall at the
                // VERTICAL descent rate only - a tumbling horizontal arrival
                // (vy ~ 0, 240 m/s sideways) got ignition 2s before impact.
                // Now also fire on remaining path distance to the predicted
                // impact point vs the kinematic stopping distance for the
                // TOTAL speed plus the unpowered rotation/ignition lead.
                // predBodyRelPos comes from the 1Hz prediction sim; its
                // zero-default makes pathLeft huge, never a false trigger.
                // Recomputed every control tick (~0.1s)
                double landingMinThrust, landingMaxThrust;
                KSPUtils.ComputeMinMaxThrust(vessel, out landingMinThrust, out landingMaxThrust, false, landingBurnEngines);
                double newLandingBurnAMax = landingMaxThrust / totalMass;
                if ((Math.Abs(landingBurnAMax - newLandingBurnAMax) > 0.5) || ((y < 15000) && (Math.Abs(groundAlt - landingBurnHeightGroundRef) > 100)))
                {
                    landingBurnAMax = newLandingBurnAMax; // update so we don't continually recalc
                    landingBurnHeightGroundRef = groundAlt;
                    landingBurnHeight = Simulate.CalculateLandingBurnHeight(groundAlt, r, v, vessel, totalMass, landingMinThrust, landingMaxThrust, aeroModel, vessel.mainBody, this, 100, "", suicideFactor);
                }
                double flipAlt = HGUtils.Clamp((-vy) * (flipTime + igniteDelay) + Math.Max(0, landingBurnHeight), flipAltMin, flipAltMax);
                double gHere = body.gravParameter / (r.magnitude * r.magnitude);
                double avStop = Math.Max(1, landingBurnAMax * suicideFactor - gHere);
                double vTot = vel_air.magnitude;
                double stopDist = vTot * vTot / (2 * avStop) + vTot * (flipTime + igniteDelay);
                bool verticalRule = yG <= flipAlt;
                // Real flight only: the sim flies the flip instantly
                // (att=steer each step) and its predBodyRelPos is one
                // prediction tick stale, so the path rule adds bias there
                // without buying anything - the catastrophe it prevents
                // (horizontal arrival, vy~0) cannot happen in the sim
                // f89: TWO guards so a vertical-ish arrival never fires this
                // rule early (f89 crash: the violent brake dropped vy to
                // -50 at 3.3km and the rule flipped 2km above the vertical
                // rule's altitude; the suicide law then saw a ship falling
                // far SLOWER than its profile and withheld thrust for the
                // whole 34s fall while the hull glide-accelerated to
                // 120 m/s horizontal):
                //  1) signature gate - the rule exists for the tumbling
                //     HORIZONTAL arrival (flight 44: vy~0, 240 m/s
                //     sideways); require vh to actually dominate the fall
                //  2) under TrajCal the own sim is not the guidance
                //     reference (its predBodyRelPos is not steered by the
                //     red mark) - measure pathLeft against the Traj mark
                double vhNow = Vector3d.Exclude(up, vel_air).magnitude;
                Vector3d pathTgt = ((TrajCalActive()) && (trajImpactValid)) ? trajImpact : predBodyRelPos;
                double pathLeftEff = (pathTgt - r).magnitude;
                bool pathRule = (!simulate) && (yG <= flipAltMax) && (vhNow > 2 * Math.Abs(vy)) && (pathLeftEff <= stopDist);
                if ((vy < 0) && (verticalRule || pathRule))
                {
                    phase = BLControllerPhase.Flip;
                    flipStartT = t; // sim flip-fall gate + chute stall timer
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToFlip");
                }
            }

            // FLIP (starship, design D8, retargeted after flights 51-53):
            // NOSE-UP pitch to the local vertical with a small anti-vh lean -
            // the real Starship maneuver. The rotation itself is the
            // horizontal brake: the broadside hull keeps catching the wind
            // while pitching up, trading vh for body lift instead of
            // propellant (the old falcon-style target -vel_air swung the
            // nose 180 deg around to point BACKWARD along the flight path -
            // "直接调转机头和屁股" - and threw away the lift-kill). The
            // attitude is flown by the falcon9 SAS path (Fly excludes Flip
            // from the belly PD). Throttle stays zero until the nose is
            // within 45 deg of the near-vertical target, then the shared
            // LandingBurn takes over. No abort: a failed flip crashes and
            // the log says why.
            // In simulation the rotation is instant (att=steer each step) -
            // make the sim pay the real cost instead: flipTime+igniteDelay
            // seconds of unpowered fall before the burn may start, so the
            // predicted cross INCLUDES the flip fall instead of crediting a
            // fantasy terminal glide below 1 km
            if (phase == BLControllerPhase.Flip)
            {
                Vector3d vhFlip = Vector3d.Exclude(up, vel_air);
                steer = (vhFlip.magnitude > 1)
                    ? Vector3d.Normalize(up - flipUprightLean * Vector3d.Normalize(vhFlip))
                    : Vector3d.Normalize(up);
                attitudeError = HGUtils.angle_between(att, steer);
                // Airbrake pitch-up assist + burn-during-flip (see the field
                // comment): light once the nose is clearly swinging up
                // instead of waiting out the full rotation unpowered
                airbrakeWanted = true;
                throttle = (attitudeError < flipBurnMaxAngle) ? flipBurnThrottle : 0;
                if (flipStartT < 0)
                    flipStartT = t; // sim copies start un-set; the trigger sets it
                bool rotated = simulate
                    ? (t - flipStartT >= flipTime + igniteDelay)
                    : (attitudeError < 45);
                if (rotated)
                {
                    lowestY = KSPUtils.FindLowestPointOnVessel(vessel);
                    phase = BLControllerPhase.LandingBurn;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToLandingBurn");
                }
            }

            // Last-resort parachutes (user request after flight 51): the
            // flip cannot always pull the nose around (f48 needed manual
            // chute assist) and a landing burn that has lost attitude low
            // down cannot recover - deploy stock chutes rather than crater.
            // Speed guard: stock chutes shred above ~300 m/s
            if ((!simulate) && (parachuteBackup) && (!chutesDeployed))
            {
                if (phase == BLControllerPhase.Flip)
                {
                    if (flipStartT < 0)
                        flipStartT = t;
                    if ((attitudeError > 45) && (t - flipStartT > flipTime + 10) && (vel_air.magnitude < 300))
                    {
                        KSPUtils.DeployParachutes(vessel);
                        chutesDeployed = true;
                        msg = "Flip stalled - deploying parachutes as last resort";
                    }
                }
                else if ((phase == BLControllerPhase.LandingBurn) && (attitudeError > 70) && (yG < 2000) && (vy < -30) && (vel_air.magnitude < 300))
                {
                    KSPUtils.DeployParachutes(vessel);
                    chutesDeployed = true;
                    // throttle NOT cut: chutes and the suicide burn stack
                    // (user flew exactly that combination and it worked)
                    msg = "Attitude lost in landing burn - deploying parachutes as last resort";
                }
            }

            // BOOSTBACK
            if (phase == BLControllerPhase.BoostBack)
            {
                // Aim to close max of 20% of error in 1 second
                steer = -Vector3d.Normalize(error);
                // Safety checks in inverse cosine
                attitudeError = HGUtils.angle_between(att, steer);
                double dv = error.magnitude / targetT; // estimated delta V needed
                double ba = 0;
                if (attitudeError < 10 + dv * 0.5) // more accuracy needed when close to target
                    ba = Math.Max(0.3 * dv, 10 / targetT);
                throttle = Mathf.Clamp((float)((ba - amin) / (0.01 + amax - amin)), minThrottle, 1);
                // Stop if error has grown significantly. Use a smoothed error
                // for the minimum (the raw prediction jumps >1km tick-to-tick
                // near the cutoff, and min-of-noisy-series makes the 1.5x
                // threshold trigger on pure sim noise) and require the growth
                // to persist 1s before giving up - a converging burn must
                // never be cut by a single-tick prediction spike (flight 17:
                // cut at 883m while closing 30m/0.1s, coasted with 1960m)
                smError = (smError < 0) ? targetError : smError + (targetError - smError) * HGUtils.Clamp(dt / 1.5, 0, 1);
                minError = Math.Min(smError, minError);
                if ((smError > minError * 1.5) && (targetError < 5000))
                    boostbackGrowTime += dt;
                else
                    boostbackGrowTime = 0;
                if ((boostbackGrowTime > 1.0) || (targetError < 10))
                {
                    phase = BLControllerPhase.Coasting;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToCoasting");
                }
                if ((y < reentryBurnAlt) && (vy < 0)) // falling
                {
                    // f97 low-trajectory fix: this handoff exists for HIGH
                    // flights, where the boostback is long done before the
                    // fall below reentryBurnAlt. On a LOW flight (apoapsis
                    // below reentryBurnAlt - the syl booster turned around at
                    // 14 km) it fired at apex and cut an INCOMPLETE boostback
                    // with the impact still ~7 km off target; the reentry
                    // "burn" then saw a slower-than-target velocity and fell
                    // straight through to AeroDescent (user: 还没有把落点推到
                    // 目标就取消执行了反而继续了下一步滑翔, landed 6.7 km off).
                    // A converging burn keeps burning while falling - the
                    // done/diverging exits above (error <10m, or 1s of growth
                    // inside 5km -> Coasting) still fire first. Only the
                    // ground forces this handoff: the landing burn still
                    // needs its height
                    if (yG < Math.Max(2500, 2.5 * landingBurnHeight))
                    {
                        phase = BLControllerPhase.ReentryBurn;
                        msg = Localizer.Format("#BoosterGuidance_SwitchedToReentryBurn");
                    }
                }

                // TODO - Check for steer in 180 degrees as interpolation wont work
                steer = Vector3d.Normalize(att * 0.75 + steer * 0.25); // simple interpolation to damp rapid oscillations
            }

            // COASTING
            if (phase == BLControllerPhase.Coasting)
            {
                if ((y < reentryBurnAlt) && (vy < 0))
                {
                    phase = BLControllerPhase.ReentryBurn;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToReentryBurn");
                }
            }

            // RE-ENTRY BURN
            // (f83 cleanup: the starship aim-burn branch that used to hijack
            // this phase is deleted - starship never enters ReentryBurn now;
            // EntryCoast/BellyFlop go straight to the glide and TrajCal owns
            // all impact correction. What remains is the falcon9 speed-
            // control burn)
            if (phase == BLControllerPhase.ReentryBurn)
            {
                {
                double errv = vel_air.magnitude - reentryBurnTargetSpeed;

                if (errv > 0)
                {
                    double smooth = HGUtils.LinearMap((double)y, (double)reentryBurnAlt, (double)reentryBurnAlt - 4000, 0, 1);
                    // Limit maximum de-acceleration to make the simulation accuracy when dt=2 or 4 secs
                    double da = g + Math.Min(Math.Max(errv * 0.3, 10), 50); // attempt to cancel 30% of extra velocity in 1 sec and min of 10m/s/s
                                                                            // Use of dt prevents too high throttle when simulating re-entry burn with dt=2 or 4 secs.
                    double newThrottle = smooth * (da - amin) / (0.01 + amax - amin);
                    throttle = HGUtils.Clamp(newThrottle, minThrottle, 1);
                }
                else
                {
                    phase = BLControllerPhase.AeroDescent;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToAeroDescent");
                }

                if (!simulate)
                {
                    pid_reentry.kp = reentryBurnSteerKp * CalculateSteerGain(throttle, vel_air, r, y, totalMass, false);
                    steerGain = pid_reentry.kp;
                    double ang = pid_reentry.Update(error.magnitude, Time.deltaTime);
                    steer = -Vector3d.Normalize(vel_air) + GetSteerCorrection(error, ang, EffectiveMaxAoA(reentryBurnMaxAoA, y), omegaLat);
                }
                }
            }

            // desired velocity - used in AERO DESCENT and LANDING BURN
            double dvy = -touchdownSpeed;
            double av = Math.Max(0.1, landingBurnAMax - g);

            // AERO DESCENT
            if (phase == BLControllerPhase.AeroDescent)
            {
                if (!simulate)
                {
                    pid_aero.kp = aeroDescentSteerKp * CalculateSteerGain(0, vel_air, r, y, totalMass, false);
                    steerGain = pid_aero.kp;
                    double ang = pid_aero.Update(error.magnitude, dt);
                    steer = -Vector3d.Normalize(vel_air) + GetSteerCorrection(error, ang, EffectiveMaxAoA(aeroDescentMaxAoA, y), omegaLat);
                }

                double landingMinThrust, landingMaxThrust;
                KSPUtils.ComputeMinMaxThrust(vessel, out landingMinThrust, out landingMaxThrust, false, landingBurnEngines);
                double newLandingBurnAMax = landingMaxThrust / totalMass;

                if ((Math.Abs(landingBurnAMax - newLandingBurnAMax) > 0.5) || ((y < 15000) && (Math.Abs(groundAlt - landingBurnHeightGroundRef) > 100)))
                {
                    landingBurnAMax = landingMaxThrust / totalMass; // update so we don't continually recalc
                    landingBurnHeightGroundRef = groundAlt;
                    landingBurnHeight = Simulate.CalculateLandingBurnHeight(groundAlt, r, v, vessel, totalMass, landingMinThrust, landingMaxThrust, aeroModel, vessel.mainBody, this, 100, "", suicideFactor);
                }

                // Never enter the landing burn while still ascending: landingBurnHeight
                // is meaningless against an ascending trajectory and the transition
                // would fire immediately (enable-during-ascent bug)
                // f127 (user-picked A): landingBurnHeight prices killing the
                // TOTAL speed energetically, but the lateral kill is
                // AUTHORITY-limited: below aoaRampLowAlt the cap is lowAoACap
                // (7 deg) so aLat <= ~amax*0.12 and a light booster's whole
                // sub-4km burn lasts only ~9 s - f127's 3.75m arrived with
                // vh=110-117 (burn start 3.3-3.7 km) where killing it is
                // mathematically impossible (measured 110->44 = cap-limited;
                // the ship swept past the pad still carrying 42 m/s
                // tangentially). When the natural burn is short, ignite
                // higher so the horizontal kill gets time in the higher-cap
                // band above the line. Heavy boosters unaffected (their
                // 10-16 km vertical height always exceeds the extension);
                // tame arrivals (vh<~40) add <1 km; starship ignites via its
                // own flip logic and never reaches this branch
                double igniteH = landingBurnHeight;
                if (igniteH < aoaRampLowAlt)
                {
                    double vhIgn = Vector3d.Exclude(up, vel_air).magnitude;
                    igniteH = Math.Max(igniteH, aoaRampLowAlt + vhIgn * vhIgniteGain);
                }
                if ((vy < 0) && (yG - vel_air.magnitude * igniteDelay <= igniteH)) // Switch to landing burn N secs earlier to allow RO engine start up time
                {
                    lowestY = KSPUtils.FindLowestPointOnVessel(vessel);
                    phase = BLControllerPhase.LandingBurn;
                    msg = Localizer.Format("#BoosterGuidance_SwitchedToLandingBurn");
                }
                // Interpolate to avoid rapid swings
                steer = Vector3d.Normalize(att * 0.75 + steer * 0.25); // simple interpolation to damp rapid oscillations
            }

            // LANDING BURN (suicide burn)
            if (phase == BLControllerPhase.LandingBurn)
            {
                // f106: the engine-set enforcement that lived here moved to
                // BoosterGuidanceCore.Fly - it now fires ONCE at Flip/LB entry
                // (with a too-weak-thrust fallback) instead of the old
                // once-per-enable SetActiveEngines here PLUS the per-tick
                // core loop, so engines the user adds mid-burn stay lit
                av = Math.Max(0.1, amax - g); // wrong on first iteration
                // Profile-slope feedforward: the deceleration required just to *stay* on the
                // target descent profile (d(dvy)/dt via chain rule; equals (1+sf)*av/2 when
                // tracking). A pure proportional throttle law cannot track a sloped profile -
                // it settles into a steady-state speed deficit of tau*slope (~11 m/s) that is
                // carried all the way down (flight test: impact at -24 m/s).
                // The suicide/taper handoff happens inside at taperSwitchY - see
                // SuicideBurnThrottle for why the suicide branch must not run to y=0
                // (flight 25 hover trap)
                double dvyDt;
                if (amax > 0)
                {
                    throttle = SuicideBurnThrottle(yG, vy, av, g, amin, amax, suicideFactor,
                        touchdownSpeed, touchdownMargin, dt, minThrottle, taperSwitchY,
                        out dvy, out dvyDt, Vector3d.Exclude(up, vel_air).magnitude);

                    // If badly tilted (>~70 deg from vertical) while still falling fast,
                    // a burn cannot decelerate the fall at all - it only flings the
                    // impact point downrange (flight 12: full-thrust horizontal at
                    // 1.3km, landed 1.5km off). Cut to minimum thrust until attitude
                    // is recovered. Gated on vy so the slow terminal hover (where the
                    // logged attitude axis has proven unreliable) is never starved
                    if ((Vector3d.Dot(att, up) < 0.35) && (vy < -50))
                        throttle = minThrottle;
                    else
                    {
                        // compensate if not vertical as need more vertical component of thrust
                        // (floor of 0.5 so a bad control-point reference can at most double thrust)
                        throttle = HGUtils.Clamp(throttle / Math.Max(0.5, Vector3d.Dot(att, up)), minThrottle, 1);
                        // f126 (user-picked A, 3.75m regression): the floors
                        // below RAISE throttle, but at burn start the hull can
                        // still be ~60 deg off the steer (flip transient -
                        // radial-1 both attempts: thr=1.00 spike at
                        // att_err=60), and a raised throttle then shoves the
                        // ship sideways instead of braking along the steer.
                        // Hold the floors off until the hull is within 25 deg
                        // of the last commanded steer. Falcon only: the
                        // starship belly frame's attitude error is not
                        // nose-vs-steer, and its f124 floor is validated
                        // as-is. In the sim the attitude tracking is instant,
                        // so the gate is always open there and the prediction
                        // still sees the same law
                        bool floorsAttOk = (recoveryProfile == "starship") || (attErrPrevTick <= 25);
                        // f89: the suicide law manages VERTICAL speed only.
                        // Entering the burn slower than the profile (early
                        // flip after a violent brake, or a genuine horizontal
                        // arrival) it coasts at throttle~0 by design - but
                        // then a big horizontal speed has NO thrust behind
                        // the retrograde lean to kill it, and the hull
                        // glide-accelerates downrange instead (f89: vh
                        // 64 -> 121 m/s during a 34s unpowered "landing
                        // burn", splat 1.3km past the target). While the
                        // ship is well above the ground and fast sideways,
                        // hold a modest floor so the lean has bite; the
                        // suicide law resumes once vh is under control
                        // f124 (user-picked A+B, starship): the fixed 0.3
                        // floor sits UNDER the f104 translateAuth threshold
                        // (0.35), so steer stayed pure VERTICAL the whole
                        // float - the post-flip sail had no powered lateral
                        // kill at all (88->2 m/s took 30s of aero only =
                        // 2.7km of uncommanded translation from a 62m
                        // setup). Scale the floor with vh so the f99 v2g
                        // lean gets real thrust behind it: full translation
                        // authority from vh~60 up, fading back to the old
                        // floor below. Engages at vh>25 so the corridor's
                        // ~40 m/s flip delivery is still caught. Falcon
                        // unchanged
                        double vhMagF = Vector3d.Exclude(up, vel_air).magnitude;
                        double f89Floor = 0.3;
                        double f89VhOn = 40;
                        if (recoveryProfile == "starship")
                        {
                            f89Floor = HGUtils.Clamp(0.35 + vhMagF / 150.0, 0.3, 0.8);
                            f89VhOn = 25;
                        }
                        if ((!simulate) && floorsAttOk && (yG > 300) && (vhMagF > f89VhOn) && (throttle < f89Floor))
                        {
                            throttle = f89Floor;
                            if (t - lastVhFloorLogT > 5)
                            {
                                lastVhFloorLogT = t;
                                Log.Info(string.Format("[LandingBurn] vh-floor: vh={0:F0} vy={1:F1} y={2:F0} thr={3:F2} - holding throttle to kill horizontal speed", vhMagF, vy, yG, throttle));
                            }
                        }
                        // f115 horizontal-kill floor: the f89 floor above only
                        // guarantees SOME thrust behind the lean. A heavy fast
                        // arrival (f115: 481 t brick, vy=-601 vh=380 at burn
                        // start 18.5 km) has a suicide profile with vertical
                        // margin to spare, so the law coasts at 0.3 through the
                        // high band while the horizontal kill gets only
                        // sin(cap)*0.3*amax - mathematically unable to stop.
                        // Below aoaRampLowAlt the AoA cap collapses to
                        // lowAoACap and lateral authority dies with it, so the
                        // kill MUST be done before that line: hold whatever
                        // throttle kills vh down to the residual the formula
                        // below allows (0 = full brake) while
                        // falling to aoaRampLowAlt. Runs in the sim too so the
                        // prediction (and the burn-start height it feeds) sees
                        // the same law the live loop flies.
                        // f117 retarget (user-approved 方案A): the band used to
                        // be yG > aoaRampTopAlt (kill done by 15 km) - f117's
                        // burn STARTED at 10.5 km, entirely under the gate, so
                        // the floor never fired (not once in 5 attempts; the
                        // shared log limiter would have hidden it anyway) and
                        // the ship carried vh=420 into the burn, ~100 over the
                        // pad at 1 km, 356 m past. The authority that matters
                        // is the band ABOVE aoaRampLowAlt; demand the kill
                        // there whatever height the burn started at
                        // f120 (user: 改完水平刹得有点猛,基本都是前移 - picked
                        // 距离感知残值): the fixed 80 residual ignored how far
                        // the target still was - vh IS the reach. f120 flight 1
                        // killed vh 338->50 by 4.2 km while still 2.6 km out,
                        // then crawled the 7-deg band and landed ~1.5-2 km
                        // short; flights 2/3: 321/252 m short (f117 without the
                        // floor was 356 m LONG - the fixed floor traded
                        // overshoot for undershoot). Solve for the arrival
                        // speed at the 4 km line that covers what remains:
                        //   dist = (vhNow+vhRes)/2*tBand + vhRes*vhKillTau
                        // and fire only on the EXCESS (vhNow > vhRes) - a
                        // short-for-reach arrival keeps its speed, while the
                        // f115 overshoot geometry (dist small, numerator
                        // negative) collapses to the floor = same deep kill
                        // f121 (user: 两次过冲严重 - picked 带符号剩余距离+下限0):
                        // the f120 formula used the distance MAGNITUDE - blind
                        // to direction. Flight 2 was already 14.3 km PAST the
                        // target and receding at 378 m/s when the burn started;
                        // "far away" ordered res=300 (keep speed) on an ESCAPE
                        // velocity -> 17.6 km overshoot, worst ever. Flight 1
                        // overflew the target at 11 km (dist=130, closing +18)
                        // but the 80 floor kept 51-80 m/s -> 3 km past. The
                        // residual must be the speed needed to cover the SIGNED
                        // remaining distance along the velocity direction:
                        // approaching with road ahead -> keep speed (f120
                        // behavior unchanged); overflown/receding -> rem<=0 ->
                        // res=0 = full brake. Floor 80 -> 0 for the same reason
                        // f123 (user: 还有点过冲 - picked 加精确停站项): the band
                        // term paces the kill across the WHOLE band above the
                        // 4 km line - right far out, but f123b converged to
                        // dist=97 at 7.2 km and then drifted 312m PAST while
                        // the kill idled at aLatReq=1.5 for 27s (vh 40->1),
                        // landing 266m long. Add the stop-at-target term
                        // vh^2/(2*rem) - the same law ConeGuard already uses
                        // (aNeed=closing^2/2dist): it dominates near the target
                        // (kill fast, no drift-past), is negligible far out
                        // (the band term keeps the f120 reach plan), and
                        // rem<=25 (reached/past) means brake as hard as the
                        // attitude cap allows
                        // f128 (user: 两次过冲,二次点火段水平没杀干净 - picked
                        // 方案A 延伸地板): the yG > aoaRampLowAlt gate assumed the
                        // kill could always be finished above the 4 km line -
                        // true for the heavy (16 km burn start, arrives at the
                        // line with vh=1.7) but not for short-burn boats: f128's
                        // 3.75m ignited at 7.2 km (f127 extension), the floor
                        // paced vh 57->20 by the line with res=9 unmet, and
                        // BELOW the line the suicide law coasted (throttle ~0
                        // -> aLat=0.1, steering impotent) while nothing paced
                        // vh until v2g's brake woke at ~900 m - crossed the pad
                        // at 25 m/s, stopped 59/74 m past on both flights.
                        // Extend the same signed-remainder pacing down to the
                        // upright latch: the floor raising throttle IS what
                        // restores lateral authority in the coast. The 999
                        // hard-brake branch must NOT reach low altitude (full
                        // throttle spike at vh=2 near the ground - f126
                        // family), so below the line the stop term is clamped
                        // to vh^2/50. Heavy regression checked numerically:
                        // vh=1.7/dist=38 at the line -> demand ~minThrottle =
                        // no-op under its suicide burn; starship untouched
                        double vhNow = Vector3d.Exclude(up, vel_air).magnitude;
                        if ((yG > uprightHeight) && (amax > 0) && floorsAttOk)
                        {
                            double tBand = Math.Max((yG - aoaRampLowAlt) / Math.Max(50, -vy), vhKillTBandMin);
                            Vector3d vhVecK = Vector3d.Exclude(up, vel_air);
                            Vector3d tgtHK = Vector3d.Exclude(up, tgt_r - r);
                            double distHK = tgtHK.magnitude;
                            double remHK = (vhNow > 1) ? Vector3d.Dot(tgtHK, vhVecK / vhNow) : distHK;
                            // f135's descent-synced tau REVERTED per user
                            // order (退回前天稳定批次): fixed vhKillTau
                            // restored - the 200T brick's fast fall below the
                            // line re-exposes the f135 921m-short physics
                            double vhRes = HGUtils.Clamp((remHK - vhNow * tBand / 2) / (tBand / 2 + vhKillTau), 0, vhKillResidCap);
                            double aLatFull = amax * Math.Sin(EffectiveMaxAoA(landingBurnMaxAoA, yG) * deg2rad);
                            if (vhNow > vhRes)
                            {
                                // f130 (heavy CRASH, user: 高空乱点火落不下去燃料耗尽):
                        // the 999 "brake as hard as the cap allows" branch was
                        // an f123 lazy-infinity - full throttle for ANY vh>0
                        // when rem<=25. The f128 extension plus this flight's
                        // early homing (dist=21 at 8.4 km) put the heavy into
                        // the rem~+-25 zone for ~70 s: every drift-past ate a
                        // thr=1.00 slam (TWR~3.3 -> 7.7g vertical CLIMB
                        // vy=+22, and ~6 m/s2 lateral that shot vh THROUGH
                        // zero: 1 -> 33 m/s the other way, dist 33 -> 485 =
                        // ping-pong), even vh=1 triggered full thrust
                        // (rem<0 -> res=0, 1>0 fires). ~100 t burned hovering
                        // at 7.7 km, flameout at 5.9 km, cratered 250 m out.
                        // Proportional stop everywhere: vh^2/(2*max(rem,25))
                        // is IDENTICAL to the old formula for rem>25, equals
                        // the f128 below-line clamp for rem<=25, and keeps the
                        // f123 slam for genuinely fast drift (vh=40 -> 32 >
                        // aLatFull -> full brake) while a 1-10 m/s drift gets
                        // a gentle 0.02-2 m/s2 instead of a 7.7g slam
                        double aLatStop = (vhNow * vhNow) / (2 * Math.Max(remHK, 25));
                                double aLatReq = Math.Max((vhNow - vhRes) / tBand, aLatStop);
                                // f131 (user: 精度好但有点废燃料 - picked 方案A
                                // 死区): small demands (0.1-2.5) are residuals the
                                // terminal v2g absorbs free under the final burn's
                                // own throttle. Genuine misses demand 6-42 m/s2:
                                // 1.5 splits them
                                // f132 (3.75m HIT THE LAUNCH TOWER, user: 点火抽搐
                                // 落点不准): the single-threshold deadband was a
                                // bang-bang switch - f132's demand sat at 1.5+-0.1
                                // for most of the burn (KSP.log samples pinned at
                                // 1.5), so every tick flipped raise/release: 202
                                // throttle flips in 39 s (~5 Hz visible engine
                                // twitch, half the coast at min throttle = half
                                // the lateral authority). Homing slowed, arrived
                                // 44 m off at 255 m still falling -134, hit the
                                // tower (dist=34) at y=89, vy=-70. HYSTERESIS:
                                // engage at 1.5, hold until demand < 0.6 - noise
                                // at the threshold can no longer drop the floor
                                // mid-kill; genuinely tiny demands (never reach
                                // 1.5) stay suppressed, so the f131 fuel saving
                                // survives. Sim keeps the plain threshold: the
                                // latch is real-flight state, a sim run must not
                                // mutate it (f108 shared-latch lesson)
                                bool vhKillDbPass;
                                if (simulate)
                                    vhKillDbPass = aLatReq >= vhKillDeadband;
                                else
                                {
                                    if (aLatReq >= vhKillDeadband)
                                        vhKillLatched = true;
                                    else if (aLatReq < vhKillDeadbandRelease)
                                        vhKillLatched = false;
                                    vhKillDbPass = vhKillLatched;
                                }
                                if (vhKillDbPass)
                                {
                                double vhKillFloor = HGUtils.Clamp(aLatReq / Math.Max(0.1, aLatFull), minThrottle, 1);
                                if (throttle < vhKillFloor)
                                {
                                    throttle = vhKillFloor;
                                    if ((!simulate) && (t - lastVhKillLogT > 5))
                                    {
                                        lastVhKillLogT = t;
                                        Log.Info(string.Format("[LandingBurn] vh-kill floor: vh={0:F0} vy={1:F1} y={2:F0} dist={3:F0} rem={4:F0} res={5:F0} tBand={6:F1}s aStop={7:F1} aLatReq={8:F1}/{9:F1} -> thr={10:F2} (dist-paced horizontal kill, f128 extends below the {11:F0}m line)", vhNow, vy, yG, distHK, remHK, vhRes, tBand, aLatStop, aLatReq, aLatFull, throttle, aoaRampLowAlt));
                                    }
                                }
                                }
                            }
                        }
                        else if (!simulate)
                        {
                            vhKillLatched = false;
                        }
                        // f92 coast translation: while the suicide law is
                        // coasting (vy under the profile -> throttle ~0) there
                        // is no thrust behind the steer tilt, so the residual
                        // offset just rides along until the dense low band.
                        // High and thin is where translation is cheapest -
                        // hold a small floor when real position error remains.
                        // Clamped under 0.85*g/amax so it can never hover; the
                        // suicide law takes back over the moment it wants more
                        if ((!simulate) && (recoveryProfile == "starship") && (coastTransThrottle > 0) && (yG > coastTransMinAlt) && (!coastTransGiveUp))
                        {
                            double pErrMag = Vector3d.Exclude(up, r - tgt_r).magnitude;
                            double tGoC = 2 * yG / Math.Max(5, -vy);
                            if ((pErrMag > coastTransMinErr) && (tGoC > 8))
                            {
                                if (coastTransStartT < 0)
                                {
                                    coastTransStartT = t;
                                    coastTransStartErr = pErrMag;
                                }
                                else if ((t - coastTransStartT > 15) && (pErrMag > 0.9 * coastTransStartErr))
                                {
                                    coastTransGiveUp = true;
                                    Log.Info(string.Format("[LandingBurn] trans-floor GIVE-UP: y={0:F0} posErr={1:F0} was {2:F0} 15s ago - the floor cannot win this one, saving the fuel for the suicide burn", yG, pErrMag, coastTransStartErr));
                                }
                                double transFloor = Math.Min(coastTransThrottle, 0.85 * g / Math.Max(1, amax));
                                if ((!coastTransGiveUp) && (throttle < transFloor))
                                {
                                    throttle = transFloor;
                                    if (t - lastCoastTransLogT > 5)
                                    {
                                        lastCoastTransLogT = t;
                                        Log.Info(string.Format("[LandingBurn] trans-floor: y={0:F0} posErr={1:F0} vy={2:F1} tGo={3:F0} thr={4:F2}", yG, pErrMag, vy, tGoC, throttle));
                                    }
                                }
                            }
                            else
                            {
                                // converged (or out of time) - the next error
                                // gets a fresh 15 s window and a fresh give-up
                                coastTransStartT = -1;
                                coastTransGiveUp = false;
                            }
                        }
                    }
                }
                // Forced upright terminal attitude (also modeled in simulation via this shared path):
                // once low and slow enough sideways, command pure vertical and accept the residual
                // position error rather than touch down tilted and tip over.
                // Latched with hysteresis: a plain threshold toggled the mode at ~1 Hz as
                // horizontal speed hovered at the limit (flight 15, final 250 m) - each
                // flip to upright removed the retrograde lean so the drift regrew, which
                // flipped it back, shaking the vessel all the way down
                double horizSpeed = Vector3d.Exclude(up, vel_air).magnitude;
                if ((uprightHeight <= 0) || (yG >= uprightHeight))
                    uprightLatched = false;
                else if (horizSpeed < uprightMaxHorizSpeed)
                    uprightLatched = true;
                else if (horizSpeed > uprightMaxHorizSpeed + 3)
                    uprightLatched = false;
                bool forceUpright = (uprightHeight > 0) && (yG < uprightHeight) && uprightLatched;
                if (forceUpright)
                {
                    // Pure vertical, no lateral correction: homing during the
                    // latch was tried and reverted (flights 32-34) - the
                    // correction either pumped the horizontal speed past the
                    // latch-release threshold (f32, 13-degree touchdown) or,
                    // speed-capped, could not kill the residual tangential
                    // drift below ~60m anyway because the suicide burn is at
                    // full thrust there and the attitude lags the rotating
                    // correction by ~50 degrees (f33/34, 5-6 degree
                    // touchdowns). Upright touchdown beats the offset it
                    // costs: legs break from tilt, not from drift
                    steer = up;
                    if ((!uprightReported) && (!simulate))
                    {
                        msg = Localizer.Format("#BoosterGuidance_ForcedUpright");
                        uprightReported = true;
                    }
                }
                else if ((!simulate) && (yG > noSteerHeight))
                {
                    double maxAoA = EffectiveMaxAoA(landingBurnMaxAoA, yG);
                    if ((uprightHeight > 0) && (yG < uprightHeight))
                    {
                        // Still too fast sideways to go fully upright: ramp the allowed angle
                        // down as we descend so the booster is nearly vertical by noSteerHeight
                        double frac = HGUtils.Clamp((yG - noSteerHeight) / Math.Max(1, uprightHeight - noSteerHeight), 0, 1);
                        maxAoA = 2 + frac * (maxAoA - 2);
                    }
                    if (recoveryProfile == "starship")
                    {
                        // f99: run the terminal velocity-law for the WHOLE
                        // starship burn, not just below the 1500m handoff. The
                        // falcon scheme (live retro-lean + capped impact-law
                        // corr) twisted the nose through the entire burn - the
                        // lean azimuth chases the velocity azimuth, which the
                        // correction itself keeps rotating (f98: ~130 deg
                        // sweep; f99: ~120 deg, att_err ~0 all the way down -
                        // the ship flew the command exactly; user: "扭转倾斜"
                        // / "机身左倾严重"), and the 7-degree-capped corr
                        // mostly fought the lean instead of the 1.1-1.6 km
                        // entry error (landed 1358m off). The f98 azimuth
                        // freeze never even engaged (vh stayed under its 40
                        // m/s capture threshold the whole burn). The v2g
                        // velocity-law below 1500m is the part that always
                        // worked - vDes points AT the target and shrinks with
                        // altitude, (vDes - v) damps the drift, the azimuth is
                        // stable by construction and the command converges to
                        // vertical as the ship arrives. Falcon path untouched
                        Vector3d posErrS = Vector3d.Exclude(up, r - tgt_r);
                        double budgetS = Math.Max(maxAoA, V2gMaxAoAEff * TerminalAoAFade(yG));
                        // f104: translation authority is the THROTTLE, not the
                        // tilt. In the fast-descent low-throttle part of the burn
                        // the hull's aero normal force opposes any nose tilt
                        // (weathervane) and beats the thrust component: f104
                        // tracked a perfect 17-deg tilt TOWARD the target at ~20%
                        // throttle and the hull was pushed AWAY (thrust +1.7 vs
                        // aero -5 m/s2; the 1.3km flip error grew to 3.1km;
                        // user: "左偏的落点却还要左倾" - the correction acted
                        // reversed). Below ~35% throttle any tilt does negative
                        // work, so hold vertical there and let the low-altitude
                        // high-throttle segment (thrust beat aero at 60%+) do
                        // the translating
                        double translateAuth = HGUtils.Clamp((throttle - 0.35) / 0.25, 0, 1);
                        steer = up + translateAuth * VelocityToGoCorrection(posErrS, vel_air, up, yG, vy, budgetS, omegaLat);
                        steerGain = v2gKp;
                    }
                    else
                    {
                    // Velocity-damped homing (PD), slow regime only: subtract
                    // landingBurnVelDamp * horizontal velocity from the steer target, but
                    // never more than half the real error. Above 50 m/s (or uncapped) the
                    // term is a phantom hundreds of metres wide that reverses the homing
                    // direction and hunts the attitude loop (flight 12 crash; flight 15
                    // final-phase wobble with velDamp=3 persisted from that build)
                    Vector3d errEff = error;
                    if ((landingBurnVelDamp > 0) && (horizSpeed < 50) && (error.magnitude > 1))
                    {
                        Vector3d phantom = landingBurnVelDamp * Vector3d.Exclude(up, vel_air);
                        double maxPhantom = 0.5 * error.magnitude;
                        if (phantom.magnitude > maxPhantom)
                            phantom = Vector3d.Normalize(phantom) * maxPhantom;
                        errEff = error - phantom;
                    }
                    double ang;
                    double sgCalc = CalculateSteerGain(throttle, vel_air, r, y, totalMass, false);
                    if (recoveryProfile == "starship")
                    {
                        // f90: on a big lifting hull the aero/thrust blend
                        // gain is ill-conditioned exactly where it matters -
                        // sideFA ~= sideFT at terminal fall speeds fades the
                        // gain to ~0 (a 500m error commanded only 2-4 deg, and
                        // nothing at all below 264m), so the 26s burn closed
                        // just ~170m of a ~720m offset. Keep only the SIGN of
                        // the blend (aero mode when the hull force clearly
                        // dominates - that window genuinely steered f90 toward
                        // the target; thrust mode otherwise) with fixed
                        // authority: full deflection beyond ~100m of error,
                        // linear below. Zero/parity defaults to thrust mode -
                        // this is, by definition, the powered phase.
                        // f92: magnitude decoupled from the GUI knob - the
                        // save's falcon-era landingBurnSteerKp (~0.3) had
                        // scaled the fixed authority down to +/-0.003
                        pid_landing.kp = ((sgCalc > 1e-6) ? 1 : -1) * 0.1;
                    }
                    else
                        pid_landing.kp = landingBurnSteerKp * sgCalc;
                    steerGain = pid_landing.kp;
                    ang = pid_landing.Update(errEff.magnitude, Time.deltaTime);
                    Vector3d corr = GetSteerCorrection(errEff, ang, maxAoA, omegaLat);
                    // Velocity-to-go blend (DISABLED at v2gHeight=0 after the
                    // flight-20 flyby regression - see field comment; the
                    // impact-error law above is the proven scheme)
                    double w = HGUtils.Clamp((v2gHeight - yG) / Math.Max(1, v2gBlend), 0, 1);
                    if (w > 0)
                    {
                        double v2gBudget = maxAoA + w * Math.Max(0, v2gMaxAoA - maxAoA);
                        Vector3d posErr = Vector3d.Exclude(up, r - tgt_r);
                        corr = (1 - w) * corr + w * VelocityToGoCorrection(posErr, vel_air, up, yG, vy, v2gBudget, omegaLat);
                    }
                    // Early terminal handoff (v2gTermHeight, default 600m):
                    // blend the velocity braking in above noSteerHeight so
                    // the residual speed is killed by ~200m and the forced-
                    // upright latch can engage early - flight 21 engaged only
                    // at 200m, was still translating under a 12-degree
                    // correction in the final 50m and broke a landing leg.
                    // Safe here: throttle is up (braking authority exists,
                    // unlike the min-throttle band of the flight-20 flyby)
                    // and the remaining distances are small
                    double wt = (v2gTerminal) ? HGUtils.Clamp((V2gTermHeightEff - yG) / Math.Max(1, V2gTermHeightEff - noSteerHeight), 0, 1) : 0;
                    if (wt > 0)
                    {
                        double v2gBudgetT = Math.Max(maxAoA, V2gMaxAoAEff * TerminalAoAFade(yG));
                        Vector3d posErr = Vector3d.Exclude(up, r - tgt_r);
                        corr = (1 - wt) * corr + wt * VelocityToGoCorrection(posErr, vel_air, up, yG, vy, v2gBudgetT, omegaLat);
                    }
                    // f113 cone guard (user-approved f111 方案A): a fast dive
                    // AT the pad crosses overhead still carrying the
                    // horizontal component (f111: 255m off, vh=5.4; f113: 14m
                    // overhead at 678m carrying vh=110 -> 238m off, vh=6.1) -
                    // the terminal v2g handoff below 600m with the faded
                    // budget engages too late to stop the sail-past. When the
                    // lateral decel needed to null the closing speed within
                    // the remaining horizontal distance exceeds what the
                    // current attitude budget delivers, take over NOW with
                    // the full (ramp-limited) budget; hysteresis release once
                    // the closing speed is back inside the terminal law's
                    // reach.
                    {
                        Vector3d posErrH = Vector3d.Exclude(up, r - tgt_r);
                        double distH = posErrH.magnitude;
                        Vector3d velH = Vector3d.Exclude(up, vel_air);
                        double closing = (distH > 1) ? -Vector3d.Dot(velH, posErrH) / distH : 0; // >0 = approaching the pad
                        double aLat = Math.Max(0.1, (amin + throttle * (amax - amin)) * Math.Sin(maxAoA * deg2rad));
                        double aNeed = (closing > 0) ? closing * closing / (2 * Math.Max(distH, 50)) : 0;
                        // f126 (user-picked B 加强版): both release thresholds
                        // were distance-blind. closing<8 let radial-1 go at
                        // dist=18 still carrying vh=20 (sail-past - the error
                        // grew 16->67m after release), and the aNeed branch
                        // released attempt 1 at dist=30/closing=10 because the
                        // authority looked ample - but the plain law that
                        // takes over does NOT actually brake (dead PID gain +
                        // diluted v2g mid-band). Scale the closing release
                        // with the distance (8 far out, 2 at the pad) and
                        // forbid the aNeed release inside 80 m. Heavy-booster
                        // neutral far out (the clamp ceiling IS the old 8 m/s,
                        // and its releases were all aNeed-driven at dist>64);
                        // near the pad it simply brakes a touch longer
                        double cgRelClosing = HGUtils.Clamp(distH / 8, 2, 8);
                        // f127 (user-picked B): the release watched only the
                        // RADIAL closing speed - blind to tangential motion.
                        // f127 flight 2 swept past the pad at dist=19 carrying
                        // vh=42 tangentially (closing~0 -> released -> drifted
                        // out to 50m); flight 1 released at dist=3 with
                        // closing=-44 (already past, still fast). Inside 80 m
                        // additionally require the FULL horizontal speed to be
                        // under a distance-scaled threshold before letting go.
                        // Heavy-booster neutral: its historical releases were
                        // at dist>=66 with vh~closing (radial motion), which
                        // pass the new vh test too
                        double cgRelVh = HGUtils.Clamp(distH / 4, 5, 20);
                        double vhH = velH.magnitude;
                        if ((!coneGuard) && (closing > 10) && (aNeed > 0.7 * aLat))
                        {
                            coneGuard = true;
                            Log.Info(string.Format("[ConeGuard] ON t={0:F1} alt={1:F0} dist={2:F0} closing={3:F0} aNeed={4:F1} aLat={5:F1} - full-budget v2g takeover", t, y, distH, closing, aNeed, aLat));
                        }
                        else if ((coneGuard) && ((closing < cgRelClosing) || ((aNeed < 0.4 * aLat) && (distH > 80))) && ((vhH < cgRelVh) || (distH > 80)))
                        {
                            coneGuard = false;
                            Log.Info(string.Format("[ConeGuard] OFF t={0:F1} alt={1:F0} dist={2:F0} closing={3:F0} vh={4:F0} rel={5:F1}/{6:F1}", t, y, distH, closing, vhH, cgRelClosing, cgRelVh));
                        }
                        if (coneGuard)
                            corr = VelocityToGoCorrection(posErrH, vel_air, up, yG, vy, maxAoA, omegaLat);
                    }
                    // Steer retrograde with added up component to damp oscillations at slow speed near ground
                    steer = -Vector3d.Normalize(vel_air - 20 * up) + corr;
                    }
                }
                else
                {
                    if ((recoveryProfile == "starship") && (!simulate))
                    {
                        // f99: same full-burn velocity-law as above - no
                        // retro-lean azimuth chase in the last metres either
                        Vector3d posErr = Vector3d.Exclude(up, r - tgt_r);
                        steer = up + VelocityToGoCorrection(posErr, vel_air, up, yG, vy, Math.Max(EffectiveMaxAoA(landingBurnMaxAoA, yG), V2gMaxAoAEff * TerminalAoAFade(yG)), omegaLat);
                        steerGain = v2gKp;
                    }
                    else
                    {
                    // Just cancel velocity with significant upwards component to stay upright
                    steer = -Vector3d.Normalize(vel_air - 20 * up);
                    if ((!simulate) && (v2gTerminal))
                    {
                        // Below noSteerHeight (v2gTerminal): velocity braking
                        // on clean local signals. The trajectory line is
                        // already aligned by the impact-error law above, so
                        // chasing v_des = -posErr/tGo cannot cause the
                        // flight-20 flyby - it just kills the residual drift
                        // that otherwise runs unopposed to touchdown
                        // (flight 17/18: 35m of drift below 200m). Budget
                        // fades near the ground to protect touchdown attitude
                        Vector3d posErr = Vector3d.Exclude(up, r - tgt_r);
                        steer += VelocityToGoCorrection(posErr, vel_air, up, yG, vy, Math.Max(EffectiveMaxAoA(landingBurnMaxAoA, yG), V2gMaxAoAEff * TerminalAoAFade(yG)), omegaLat);
                    }
                    }
                }
                // Hard tilt cap near the ground: as vy collapses at the end of
                // the suicide profile the retro-lean term atan(vh/(20-vy))
                // plus any residual correction can command 30-45 degrees of
                // tilt in the last seconds - flight 22 was flung sideways at
                // ~18 m/s^2 under such a command at 37m and tipped over.
                // (When the forced-upright latch is engaged steer is already
                // pure up and this is a no-op.)
                // NOTE: do NOT use Vector3d.Slerp here - it is declared
                // InternalCall in Assembly-CSharp and its native implementation
                // is not registered in KSP's Mono runtime, so the first
                // execution throws MissingMethodException (flight 23: guidance
                // dead from the first tick, since the prediction sim passes
                // y<100 every tick). Vector3d.Angle/Project are managed and safe.
                if (yG < lowTiltCapHeight)
                {
                    double tilt = Vector3d.Angle(steer, up);
                    if (tilt > lowTiltCap)
                    {
                        // Rescale the perpendicular component so that
                        // tan(new tilt) = tan(lowTiltCap) - an exact cap using
                        // only managed vector ops (|perp|/|alongUp| = tan(tilt))
                        Vector3d alongUp = Vector3d.Project(steer, up);
                        Vector3d perp = steer - alongUp;
                        if ((perp.magnitude > 1e-9) && (Vector3d.Dot(steer, up) > 0))
                            steer = alongUp + perp * (alongUp.magnitude * Math.Tan(lowTiltCap * deg2rad) / perp.magnitude);
                        else
                            steer = up; // degenerate (already vertical, or pointing down)
                    }
                }
                if ((yG < noSteerHeight) && (!noSteerReported))
                {
                    msg = string.Format(Localizer.Format("#BoosterGuidance_NoSteerHeightReached"));
                    noSteerReported = true;
                }

                // Decide to shutdown engines for final touch down? (within 3 secs)
                // Criteria should be if
                // height
                double minHeight = KSPUtils.MinHeightAtMinThrust(yG, vy, amin, g);
                // Criteria for shutting down engines
                // - we could not reach ground at minimum thrust (would ascend)
                // - falling less than touchdown speed (otherwise can decide to shutdown engines when still high and travelling fast)
                // This is particulary done to stop the simulation never hitting the ground and making pretty circles through the sky
                // until the maximum time is exceeded. The predicted impact position will vary widely and this was incur a lot of time to calculate
                // - this is very tricky to get right for the actual vessel since in RO engines take time to throttle down, so it needs to be done
                //   early, allowing for the fact the residual engine thrust will slow the rocket more for the next 2-3 secs
                // - the engine will restart again if landing doesn't happen with 2-3 secs
                bool cant_reach_ground = (minHeight > 0) && (vy > -50);
                if ((cant_reach_ground) && (bailOutLandingBurn))
                    throttle = 0;

                // Interpolate to avoid rapid swings
                steer = Vector3d.Normalize(att * 0.75 + steer * 0.25); // simple interpolation to damp rapid oscillations
            }

            // Belly-attitude error for the Actual.dat att_err column: the
            // starship belly phases never set attitudeError in their steer
            // branches, so flight 41 logged a useless 0 through the whole
            // descent. Done here (before the log row) instead of in the core
            // Fly, which runs after the row is written
            if ((!simulate) && (recoveryProfile == "starship")
                && ((phase == BLControllerPhase.EntryCoast) || (phase == BLControllerPhase.BellyFlop)))
            {
                // Belly target at the scheduled AoA (not simply -steer: at
                // 60 deg cruise AoA the belly tilts up off the wind axis)
                Vector3d bT, nT;
                StarshipAttitudeController.BellyFrame(-steer, Vector3d.Normalize(r), bellyAoACurrent, out bT, out nT);
                attitudeError = HGUtils.angle_between(StarshipAttitudeController.BellyAxisWorld(vessel, bellyRollOffset + bellyRollPulseDeg), bT);
                bellyAttErrLive = attitudeError; // for the aero-cal gate: attitudeError gets overwritten at method end (nose-vs-steer ~160deg in belly flight = f60 cal silence)
            }
            // LandingBurn att_err (flight 46 logged a useless 0.0 through the
            // whole burn - the SAS-vs-steer angle is exactly the signal that
            // showed the ship falling in its tail-first trim with no control)
            if ((!simulate) && (phase == BLControllerPhase.LandingBurn))
                attitudeError = HGUtils.angle_between(att, steer);

            // Logging (real flight only - simulation copies must not pollute the actual log)
            if (Utils.LoggingActive && !simulate)
            {
                if (logTransform == null)
                    SetUpLogTransform(FlightGlobals.ActiveVessel.name);

                if (!actualRowLogged)
                {
                    actualRowLogged = true;
                    Log.Info("First actual row: phase=" + phase + " alt=" + vessel.altitude);
                }

                // Record the guidance settings whenever they change (including
                // mid-flight GUI edits by the operator) so the flight log is
                // self-describing and adjustments can be correlated with the
                // trajectory. Lines start with '#' so data parsers skip them
                // f83 cleanup: the deleted aim-burn/overshoot-brake knobs are
                // replaced by the TrajCal set so the log stays self-describing
                string settingsLine = string.Format("# settings maxAoA(l/a/r)={0:F1}/{1:F1}/{2:F1} lowAoACap={3:F1}deg@{4:F0}-{5:F0}m steerDamp={6:F2} velDamp={7:F2} noSteerH={8:F0} uprightH={9:F0} uprightV={10:F1} tdMargin={11:F0} suicide={12:F2} reentryAlt={13:F0} igniteDelay={14:F1} v2g={18:F1}deg/m/s@{15:F0}-{19:F0}m v2gVmax={20:F0} v2gAoA={22:F0} v2gTH={23:F0} v2gBO={24:F0} tiltCap={25:F0}deg@{26:F0}m flipT={27:F1} flipLean={28:F2} trajCal={29} tcOver={30:F0} tcAltFrac={31:F2} tcBrakeAlt={32:F0} tcShort={33:F0} tcBurnQ={34:F0} tcBurnAlt={35:F0}/{36:F0} tcBurnDv={37:F0} tcBurnThr={38:F2} trimQ={39:F0} bellyAoA={40:F0}/{41:F0}@{42:F0}/{43:F0} corrGain={44:F1} tvh={45:F0}/{46:F0} tAoA={47:F0} fb={48:F2}/{49:F0} abLiftRatio={50:F2} vb={51} vAlt={52:F0} vDist={53:F0} vQ={54:F0} tcExit={55:F0} am={56} amDist={57:F0} amG={58:F3} tgt={16:F4},{17:F4},{21:F0} bt={59} btAlt={60:F0} btErr={61:F0} btQ={62:F0} btThr={63:F2} btDv={64:F0} btExit={65:F0} btTau={66:F1} latMax={67:F1}",
                    landingBurnMaxAoA, aeroDescentMaxAoA, reentryBurnMaxAoA, lowAoACap, aoaRampLowAlt, aoaRampTopAlt, steerDamping, landingBurnVelDamp, noSteerHeight, uprightHeight, uprightMaxHorizSpeed, touchdownMargin, suicideFactor, reentryBurnAlt, igniteDelay, v2gHeight, tgtLatitude, tgtLongitude, v2gKp, v2gBlend, v2gMaxSpeed, tgtAlt, v2gMaxAoA, v2gTermHeight, v2gBrakeOnlyRadius, lowTiltCap, lowTiltCapHeight, flipTime, flipUprightLean, starshipTrajCal ? 1 : 0, trajCalOverError, trajCalOverAltFrac, trajCalBrakeMaxAlt, trajCalShortError, trajCalBurnMaxQ, trajCalBurnMinAlt, trajCalBurnMaxAlt, trajCalBurnMaxDv, trajCalBurnThrottle, bellyTrimFlipQ, bellyBrakeAoA, bellyGlideAoA, bellyQFullBrake, bellyQFullGlide, starshipCorrectionGain, bellyTerminalVh, bellyTerminalVhBlend, bellyTerminalMaxAoA, flipBurnThrottle, flipBurnMaxAngle, airbrakeMaxLiftRatio, trajCalViolentBrake ? 1 : 0, trajCalViolentAlt, trajCalViolentDist, trajCalViolentMaxQ, trajCalBurnExitShort, trajCalAoaMod ? 1 : 0, trajCalAoaModDist, trajCalAoaModGain, trajCalBigTrim ? 1 : 0, trajCalBigTrimMinAlt, trajCalBigTrimError, trajCalBigTrimMaxQ, trajCalBigTrimThrottle, trajCalBigTrimMaxDv, trajCalBigTrimExit, trajCalBigTrimTau, bellyLateralMaxDeg);
                if (settingsLine != lastSettingsLine)
                {
                    lastSettingsLine = settingsLine;
                    Utils.Log(Utils.LogType.actual, settingsLine);
                    Log.Info("Settings: " + settingsLine);
                }

                Vector3d a = att * (amin + throttle * (amax - amin)); // this assumes engine is ignited though
                Vector3d tr = logTransform.InverseTransformPoint(r + body.position);
                Vector3d tv = logTransform.InverseTransformVector(vel_air);
                Vector3d ta = logTransform.InverseTransformVector(a);

                // f136 dual-prediction logging: keep Trajectories computing
                // (it idles with its window closed) and append ITS predicted
                // impact in the same target-centered frame as tr, so the
                // row carries both predictions side by side. Never feeds back
                // into any control law
                if (!trajLogForceDone)
                {
                    trajLogForceDone = true;
                    TrajAPI.SetAlwaysUpdate(true);
                }
                Vector3d? impLog = TrajAPI.GetImpactPosition();
                if (impLog.HasValue)
                {
                    trajLogImpact = impLog.Value;
                    trajLogImpactT = t;
                }
                double trajX = double.NaN, trajZ = double.NaN;
                if ((trajLogImpactT >= 0) && (t - trajLogImpactT < trajImpactMaxAge))
                {
                    Vector3d impL = trajLogImpact;
                    double impLAge = t - trajLogImpactT;
                    if (impLAge > 0.01) // rotate forward with the body spin between Trajectories' refreshes (same as the TrajCal read)
                        impL = (Vector3d)(Quaternion.AngleAxis((float)(impLAge * body.angularVelocity.magnitude * Mathf.Rad2Deg), body.angularVelocity.normalized) * (Vector3)trajLogImpact);
                    Vector3d tImp = logTransform.InverseTransformPoint(impL + body.position);
                    trajX = tImp.x;
                    trajZ = tImp.z;
                }

                Utils.Log(Utils.LogType.actual, String.Format("{0:F1} {1} {2:F1} {3:F1} {4:F1} {5:F1} {6:F1} {7:F1} {8:F1} {9:F1} {10:F1} {11:F1} {12:F1} {13:F1} {14:F3} {15:F1} {16:F2} {17:F1} {18:F1}",
                    t - logStartTime, phase, tr.x, tr.y, tr.z, tv.x, tv.y, tv.z, ta.x, ta.y, ta.z, attitudeError, amin, amax, steerGain, targetError, totalMass, trajX, trajZ));
                logLastTime = t;
            }

            // PILOT BURN ATTITUDE (see the field comment): a GUI-button-
            // driven manual override, placed after every phase block so it
            // owns the final steer in the calm phases. Guidance's own burns
            // keep priority (they set throttle > 0, which skips this AND
            // makes Fly ignore the pilot throttle), and the sim never sees
            // it (simulate / clone fields stay at defaults)
            if ((!simulate) && (recoveryProfile == "starship"))
            {
                bool calmPhase = (phase == BLControllerPhase.AwaitDeorbit)
                    || (phase == BLControllerPhase.EntryCoast)
                    || (phase == BLControllerPhase.Coasting)
                    || (phase == BLControllerPhase.BellyFlop);
                pilotBurnAttitude = false;
                swingDir = 0;
                pilotRefuseWhy = "";
                double qPilot = DynamicPressure(y, vel_air.magnitude, body);

                // f104: the f85 experimental high-alt FINE trim that was
                // described here is removed (user: "5w5米以上的实验性发动机
                // 调控功能去掉吧不好用") - high altitude is the phantom-mark
                // regime, a fine engine trim could only chase phantoms.
                // f97 BIG trim: engages for |along| > trajCalBigTrimError at
                // high altitude (the user wants the big errors fixed where
                // dV is cheapest). Manual buttons outrank it instantly
                // (pilotAttitudeMode != 0 suppresses the burn branch below)
                int bigDir = 0;
                // f102 (user picked option 2): the honest zone below
                // trajCalBigTrimMinAlt gets the point-first prograde burn for BIG
                // shorts (along < -15km) - the f102 星舰货运 flight landed 19.9km
                // short with every short-tool exhausted (tcBurn spent its whole
                // 150 m/s budget for ~1km of mark: a belly-attitude burn at q=9k
                // heats the air instead of extending the glide). Retro stays
                // above 30km: the long side below 30km belongs to the
                // boards/governor/violent brake. Low-alt prograde rides the wider
                // tcBurn q gate (20 kPa; flip burns run 18-24 kPa routinely).
                // f103: it releases at the same 10km as everything else now
                // (user: "只要误差小于10km就行").
                bool bigTrimAltOk = (y > trajCalBigTrimMinAlt) || ((y > 8000) && ((bigTrimDir > 0) || (trajAlong < -15000)));
                if ((trajCalBigTrim) && (pilotAttitudeMode == 0) && TrajCalActive()
                    && (phase == BLControllerPhase.BellyFlop) && bigTrimAltOk
                    && (!glideLost)
                    && (trajImpactValid) && (t - trajImpactT < trajImpactMaxAge)
                    && (bigTrimSpentDv < trajCalBigTrimMaxDv)
                    && (t - bigTrimAbortT > 60) // cooldown after a runaway abort
                    && ((dvAvailable < 0) || (dvAvailable > 1.25 * landingReserveDv))) // f96 fuel floor: never burn into the landing reserve
                {
                    bool lowHonest = (y <= trajCalBigTrimMinAlt);
                    // f103 root cause: the low-alt prograde never fired because the
                    // ENGAGE threshold stayed trajCalBigTrimError (30km) even below
                    // 30km - bigTrimAltOk let the block evaluate for along<-15km,
                    // but the direction gate still demanded -30km, and the f103
                    // mark plateaued at -20.5km. Every other gate was green
                    // (q~7400 Pa at the -15km crossing, far under the 20k limit).
                    // The honest zone now engages at the designed/labelled -15km;
                    // release is 10km for everything (user: "只要误差小于10km就行").
                    double bErr = (bigTrimDir != 0) ? ((bigTrimBellyBurn) ? 5000 : trajCalBigTrimExit) : ((lowHonest) ? 15000 : trajCalBigTrimError);
                    if ((trajAlong > bErr) && (qPilot < trajCalBigTrimMaxQ) && (!lowHonest) && (y < 40000))
                        bigDir = -1; // red mark LONG -> retrograde burn pulls it back (tail-first swing = natural trim)
                    else if ((trajAlong < -bErr) && (qPilot < (lowHonest ? trajCalBurnMaxQ : pilotBurnProgradeMaxQ)) && (y < 45000))
                        bigDir = +1; // red mark SHORT -> prograde extends it (nose-first keeps the tighter q guard)
                        // f101: prograde banned above 45 km (user-approved). A short reading up there is the
                        // phantom zone (the mark moved 16 km in 10 s with zero thrust this flight); three
                        // prograde burns at 44-55 km added ~104 m/s to a trajectory that was genuinely LONG
                        // below 30 km -> made the real overshoot worse (f86 positive-feedback regime).
                        // f107: retro now banned above 40 km too (user-approved, same phantom disease,
                        // third data point) - "braking is the safe failure direction" is DISPROVEN:
                        // f103 burned 125.6 m/s retro at 45 km chasing a phantom-long +70k into a really
                        // short trajectory; f107 burned 126.3 m/s retro at 55/44 km on +36k/+30k readings
                        // that collapsed to a REAL -19km short at 20 km (leverage 146 m per m/s x 126 m/s
                        // = 18.4 km - the retro burns WERE the shortfall; the user then had to re-add
                        // 160 m/s manually at 19 km). The 30-40 km window keeps retro for marks that
                        // stay long into the semi-honest zone; above 40 km we hold and let the honest
                        // zone own it.
                    if ((bigDir != 0) && (bigTrimDir != 0) && (bigDir != bigTrimDir) && (t - bigTrimFlipT < trajCalBigTrimFlipCd))
                        bigDir = 0; // a direction reversal waits out the cooldown unpowered
                }
                // f98 diagnostic: the first enable of f98 sat 68 s on what the
                // user read as a huge mark with no burn, and the log could not
                // say why (on paper every gate was green - suspicion is the
                // high-alt mark itself read <30km, and the "re-enable fixed
                // it" was a quickload to a lower point where it read 47km).
                // Log the gate state, rate-limited, so the next silent window
                // names its own blocker
                if ((trajCalBigTrim) && (phase == BLControllerPhase.BellyFlop) && (y > 8000)
                    && (bigTrimDir == 0) && (t - lastBigTrimDiagT > 10)) // f103: log down to the low-alt floor too - the low-alt prograde never fired in f103 and the 30km-limited diag was blind to why
                {
                    lastBigTrimDiagT = t;
                    Log.Info(string.Format("[BigTrim] gate t={0:F1} alt={1:F0} along={2:F0} cross={12:F0} q={3:F0} impactValid={4} impactAge={5:F1} glideLost={6} dvAvail={7:F0} spentDv={8:F1} pilotMode={9} trajCal={10} bigDir={11}",
                        t, y, trajAlong, qPilot, trajImpactValid, t - trajImpactT, glideLost, dvAvailable, bigTrimSpentDv, pilotAttitudeMode, TrajCalActive(), bigDir, trajCross)); // f106: cross was unlogged below 43km - the user's 9km roll-tap lateral work was invisible
                }

                if ((pilotAttitudeMode != 0) && (throttle <= 0) && (!bellyBraking) && (!glideLost) && calmPhase)
                {
                    if ((pilotAttitudeMode > 0) && (qPilot > pilotBurnProgradeMaxQ))
                    {
                        // nose-first refused at this q: keep the belly, and with
                        // pilotBurnAttitude false Fly blocks the throttle too
                        pilotRefuseWhy = "prograde refused by dynamic pressure (q limit)"; // f85: refusals must say WHY
                        if (t - pilotBurnRefuseLogT > 5)
                        {
                            pilotBurnRefuseLogT = t;
                            Log.Info(string.Format("[PilotBurn] prograde refused: q={0:F0} > {1:F0} Pa, holding belly", qPilot, pilotBurnProgradeMaxQ));
                        }
                    }
                    else
                    {
                        steer = (pilotAttitudeMode > 0) ? Vector3d.Normalize(vel_air) : -Vector3d.Normalize(vel_air);
                        pilotBurnAttitude = true;
                        swingDir = (pilotAttitudeMode > 0) ? 1 : -1;
                    }
                }
                else if (pilotAttitudeMode != 0)
                {
                    // button ON but refused - f85: the refusal used to be
                    // indistinguishable from "button off" in both the log and
                    // the on-screen message, and the user pushed the throttle
                    // for a minute against an invisible gate
                    if (!calmPhase)
                        pilotRefuseWhy = "not allowed in this phase (coast/re-entry/glide only)";
                    else if (throttle > 0)
                        pilotRefuseWhy = "guidance burn in progress - manual suppressed";
                    else if (bellyBraking)
                        pilotRefuseWhy = "calibration burn in progress";
                    else if (glideLost)
                        pilotRefuseWhy = "glide-lost protection active";
                }
                else if ((bigDir != 0) && (throttle <= 0) && (!bellyBraking) && calmPhase)
                {
                    // f97 big trim burn: same point-first swing path as the
                    // pilot buttons / fine trim - pilotBurnAttitude flies the
                    // swing and Core's 10-deg steer gate holds mainThrottle at
                    // 0 until the nose is actually on +/-vel_air (user: 一定
                    // 要朝向调控准确之后再启动油门).
                    // f98 MJ-style smooth throttle (user: "参考mj那种平滑的
                    // 点火吧,把限制去掉" - the fixed 10% cap was too slow and
                    // crude): size the throttle from the dV the mark error
                    // needs - along-excess over the online leverage estimate,
                    // delivered over ~tau seconds, mapped through the engines'
                    // delivered-accel curve. Full thrust on a big error,
                    // tapering smoothly to zero at the release band
                    // f107: pick the burn STYLE once at burn start. A
                    // short-mark burn starting in the honest zone flies the
                    // belly/glide steer sustained at up to FULL throttle
                    // until along > -5km (the user's proven manual save) -
                    // the nose-first swing pulses gave ~15 m/s where ~150
                    // was needed. The style must not flip mid-burn if the
                    // ship balloons back above 30km (f107: +1.5km on the
                    // user's burn).
                    if (bigTrimDir != bigDir)
                        bigTrimBellyBurn = (bigDir > 0) && (y <= trajCalBigTrimMinAlt);
                    if (!bigTrimBellyBurn)
                    {
                        steer = (bigDir > 0) ? Vector3d.Normalize(vel_air) : -Vector3d.Normalize(vel_air);
                        pilotBurnAttitude = true;
                        swingDir = bigDir;
                    }
                    // belly burn: steer stays on the glide attitude from the
                    // branch above; the 10-deg steer gate still holds the
                    // throttle until the belly attitude is actually tracked
                    double btRelease = (bigTrimBellyBurn) ? 5000 : trajCalBigTrimExit;
                    double needDv = Math.Max(0, Math.Abs(trajAlong) - btRelease) / Math.Max(50, bigTrimLeverage);
                    double aNeed = needDv / Math.Max(1, trajCalBigTrimTau);
                    throttle = HGUtils.Clamp((aNeed - amin) / Math.Max(0.1, amax - amin), 0, (bigTrimBellyBurn) ? 1.0 : trajCalBigTrimThrottle);
                    if (bigTrimDir != bigDir)
                    {
                        Log.Info(string.Format("[BigTrim] BURN {0}{1} t={2:F1} alt={3:F0} along={4:F0} q={5:F0} dvAvail={6:F0} spentDv={7:F1} thr={8:F2} lev={9:F0}",
                            (bigDir > 0) ? "PROGRADE" : "RETRO", (bigTrimBellyBurn) ? "(belly)" : "", t, y, trajAlong, qPilot, dvAvailable, bigTrimSpentDv, throttle, bigTrimLeverage));
                        bigTrimDir = bigDir;
                        bigTrimFlipT = t;
                        bigTrimStartAlong = trajAlong; // runaway reference (re-based at arming)
                        bigTrimArmed = false;
                        bigTrimRunawayT = -1;
                    }
                    // f96 fuel floor: the whole point of this burn is to SAVE
                    // the landing fuel by fixing the error early - never let
                    // it eat the reserve itself
                    else if ((dvAvailable >= 0) && (dvAvailable < 1.1 * landingReserveDv))
                    {
                        Log.Info(string.Format("[BigTrim] FUEL CUT t={0:F1} alt={1:F0} along={2:F0} dvAvail={3:F0} reserve={4:F0}",
                            t, y, trajAlong, dvAvailable, landingReserveDv));
                        bigTrimDir = 0;
                        bigTrimBellyBurn = false;
                        pilotBurnAttitude = false;
                        swingDir = 0;
                        throttle = 0;
                    }
                    else
                    {
                        // f98: count dV only while the steer gate is actually
                        // OPEN (Core holds mainThrottle at 0 until the nose is
                        // within 10 deg of the target) - at full throttle the
                        // swing-in would otherwise burn the 300 m/s budget on
                        // paper before producing any thrust
                        if (HGUtils.angle_between(att, steer) < 10)
                            bigTrimSpentDv += (amin + throttle * (amax - amin)) * dt;
                        // f99: arm the runaway guard only once real thrust has
                        // been delivered (>30 m/s) - before that any wrong-way
                        // mark motion is the phantom mark swinging on its own
                        // during the swing-in, and aborting then just burns the
                        // 60 s cooldown while the error stays huge
                        if ((!bigTrimArmed) && (bigTrimSpentDv > 30))
                        {
                            bigTrimArmed = true;
                            bigTrimStartAlong = trajAlong; // re-baseline: swing-in drift is not the burn's doing
                        }
                        if ((bigTrimArmed) && ((trajAlong - bigTrimStartAlong) * bigTrimDir < -10000))
                        {
                            if (bigTrimRunawayT < 0)
                                bigTrimRunawayT = t;
                            else if (t - bigTrimRunawayT >= 3)
                            {
                                Log.Info(string.Format("[BigTrim] ABORT runaway t={0:F1} alt={1:F0} along={2:F0} startAlong={3:F0} spentDv={4:F1}",
                                    t, y, trajAlong, bigTrimStartAlong, bigTrimSpentDv));
                                bigTrimDir = 0;
                                bigTrimBellyBurn = false;
                                bigTrimAbortT = t;
                                pilotBurnAttitude = false;
                                swingDir = 0;
                                throttle = 0;
                            }
                        }
                        else
                            bigTrimRunawayT = -1;
                        // Online leverage estimate (m of mark shift per m/s of
                        // dV): cumulative right-way progress per spent dV.
                        // Wrong-way samples are the phantom mark drifting on
                        // its own (f98: the error GREW 47k->53k through the
                        // first 19s of a working retro burn) - skip those
                        double progress = Math.Abs(bigTrimStartAlong) - Math.Abs(trajAlong);
                        if ((bigTrimSpentDv > 5) && (progress > 100))
                            bigTrimLeverage = HGUtils.Clamp(bigTrimLeverage + 0.1 * (progress / bigTrimSpentDv - bigTrimLeverage), 50, 1000);
                    }
                }
                else if ((bigTrimDir != 0) && (bigDir == 0))
                {
                    Log.Info(string.Format("[BigTrim] BURN OFF t={0:F1} alt={1:F0} along={2:F0} spentDv={3:F1} dvAvail={4:F0} lev={5:F0}", t, y, trajAlong, bigTrimSpentDv, dvAvailable, bigTrimLeverage));
                    bigTrimDir = 0;
                    bigTrimBellyBurn = false;
                    bigTrimArmed = false;
                    bigTrimRunawayT = -1;
                }
                // f83 cleanup: the overshootBrakeSwing branch that used to
                // follow here is deleted with the legacy brake; f104 deletes
                // the f85 HighTrim branch that sat here too
            }

            lastt = t;
            attPrev = att;
            attPrevValid = true;
            steer = Vector3d.Normalize(steer);
            attitudeError = HGUtils.angle_between(att, steer);

            throttle = HGUtils.Clamp(throttle, 0, 1);
            if (!simulate)
                prevThrottleOut = throttle;

            // Log simulate to ground when phase changes (real flight only, avoid recursive sim logging)
            // So the logging is done at the start of the new phase
            if ((lastPhase != phase) && (Utils.LoggingActive) && (!simulate))
                LogSimulation();

            elapsed_secs = timer.ElapsedMilliseconds * 0.001;

            // Set info message
            // Panel error source (f83 user directive: 面板落点误差以红十字为
            // 准): in TrajCal mode show the RAW Trajectories impact error -
            // literally what the red map mark is doing this instant. The
            // steering value (targetError) is the EMA-smoothed version and
            // lags the mark by design; showing it made the panel disagree
            // with the mark. Below noSteerHeight/landed trajErrRawValid is
            // false, so the touchdown message keeps the ACTUAL miss distance
            double errDisp = ((trajErrRawValid) && (TrajCalActive())) ? trajErrRawMag : targetError;
            string tgtErrStr;
            if (errDisp > 1000)
                if (errDisp > 100000)
                    tgtErrStr = string.Format("{0:F0}km", errDisp * 0.001);
                else
                {
                    if (errDisp > 10000)
                        tgtErrStr = string.Format("{0:F1}km", errDisp * 0.001);
                    else
                        tgtErrStr = string.Format("{0:F2}km", errDisp * 0.001);
                }
            else
                tgtErrStr = string.Format("{0:F0}m", errDisp);
            if (vessel.checkLanded())
            {
                info = string.Format(Localizer.Format("#BoosterGuidance_LandedXFromTarget", tgtErrStr));
            }
            else
            {
                string s1 = tgtErrStr;
                string s2 = string.Format("{0:F0}", attitudeError);
                string s3 = string.Format("{0:F0}", targetT);
                string s4 = string.Format("{0:F0}", elapsed_secs * 1000);
                if (showCpuTime)
                    info = string.Format(Localizer.Format("#BoosterGuidance_ErrorXTimeXCPUX", s1, s2, s3, s4));
                else
                    info = string.Format(Localizer.Format("#BoosterGuidance_ErrorXTimeX", s1, s2, s3));
            }

            if (msg != "")
                info = msg;

            return msg;
        }
    }
}
