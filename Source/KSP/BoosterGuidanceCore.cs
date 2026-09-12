using System;
using System.IO;
using System.Linq;
using System.Collections.Generic;
using KSP.Localization;
using UnityEngine;
using UnityEngine.Profiling;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    public class BoosterGuidanceCore : PartModule
    {
        // Saved settings

        [KSPField(isPersistant = true, guiActive = false)]
        public bool tgtSet = false;

        [KSPField(isPersistant = true, guiActive = false)]
        public double tgtLatitude = 0;

        [KSPField(isPersistant = true, guiActive = false)]
        public double tgtLongitude = 0;

        [KSPField(isPersistant = true, guiActive = false)]
        public double tgtAlt = 0;

        [KSPField(isPersistant = true, guiActive = false)]
        public double reentryBurnAlt = 55000;

        [KSPField(isPersistant = true, guiActive = false)]
        public double reentryBurnTargetSpeed = 700;

        [KSPField(isPersistant = true, guiActive = false)]
        public float reentryBurnSteerKp = 0.01f;

        [KSPField(isPersistant = true, guiActive = false)]
        public float reentryBurnMaxAoA = 20;

        [KSPField(isPersistant = true, guiActive = false)]
        public float aeroDescentSteerKp = 10;

        [KSPField(isPersistant = true, guiActive = false)]
        public float aeroDescentMaxAoA = 10;

        [KSPField(isPersistant = true, guiActive = false)]
        public float landingBurnSteerKp = 10;

        [KSPField(isPersistant = true, guiActive = false)]
        public float landingBurnMaxAoA = 10;

        [KSPField(isPersistant = true, guiActive = false)]
        public int touchdownMargin = 20;

        [KSPField(isPersistant = true, guiActive = false)]
        public float touchdownSpeed = 2;

        [KSPField(isPersistant = true, guiActive = false)]
        public int noSteerHeight = 200;

        // Below this height (and slow enough sideways) the landing burn
        // commands a pure vertical attitude. 0 disables forced uprighting.
        [KSPField(isPersistant = true, guiActive = false)]
        public int uprightHeight = 250;

        // Forced uprighting only engages once horizontal speed is below this
        [KSPField(isPersistant = true, guiActive = false)]
        public float uprightMaxHorizSpeed = 5;

        // Steer rate damping (seconds): subtracts steerDamping * lateral angular
        // rate from steer corrections to suppress oscillation. 0 disables.
        [KSPField(isPersistant = true, guiActive = false)]
        public double steerDamping = 1.0;

        // Terminal homing velocity damping (seconds). 0 disables (default) - see
        // BLController.landingBurnVelDamp; re-enable only after flight validation.
        // Field RENAMED (was landingBurnVelDamp) so the 3.0 persisted by the
        // crash-era build in existing saves is ignored - same trick as
        // aoaRampHighAlt->aoaRampTopAlt (KSPField persistence carries old
        // values across builds, silently overriding new defaults)
        [KSPField(isPersistant = true, guiActive = false)]
        public double landingBurnVelDampSec = 0.0;

        // Low-altitude AoA schedule: effective max AoA is capped at lowAoACap
        // degrees (absolute) below aoaRampLowAlt, full authority at
        // aoaRampTopAlt. Automates manually lowering the steer gain at low
        // altitude to stop oscillation. The ramp top field was renamed when
        // it moved 6000 -> 15000 (flight 16 shook from ~14km down to 5km) so
        // the persisted 6000 in existing saves is ignored
        [KSPField(isPersistant = true, guiActive = false)]
        public double lowAoACap = 7;

        [KSPField(isPersistant = true, guiActive = false)]
        public double aoaRampLowAlt = 4000;

        [KSPField(isPersistant = true, guiActive = false)]
        public double aoaRampTopAlt = 15000;

        // Coast autowarp (MechJeb-style): physics-warp through the Coasting
        // phase (the long hands-off wait after boostback) and drop back to 1x
        // for every active phase. Physics warp keeps FixedUpdate running so
        // guidance never loses authority; only ever engages for the active
        // vessel and only raises the rate (never stomps a higher manual rate)
        [KSPField(isPersistant = true, guiActive = false)]
        public bool coastAutoWarp = true;

        [KSPField(isPersistant = true, guiActive = false)]
        public int coastAutoWarpRate = 2; // physics warp index: 1=2x, 2=3x, 3=4x
        private bool autoWarpEngaged = false;

        // 自动加速 (user request): armed from the GUI, owns the warp rate
        // outright - every OnUpdate picks the highest SAFE rate and actively
        // drops to 1x the instant conditions degrade (unlike the raise-only
        // coast autowarp). Starship profile only, guidance-enabled only
        [KSPField(isPersistant = true, guiActive = false)]
        public bool autoWarp = false;
        private bool autoWarpActive = false;
        // 气动校准 kL/kD (flight 59): the active controller EMAs these during
        // the glide. Persisting them on the core feeds the deorbit scope the
        // SAME realized lift/drag (pre-enable cross == guided cross = "唯一
        // 落点") and survives disable/enable + quicksave/quickload cycles
        [KSPField(isPersistant = true, guiActive = false)]
        public double aeroCalLift = 1;
        [KSPField(isPersistant = true, guiActive = false)]
        public double aeroCalDrag = 1;

        // f64 q-banded cal values (semantics owned by BLController):
        // file-persisted only (revert-proof, see below), synced back from
        // the controller every tick so a mid-flight Changed() push never
        // clobbers the live learning
        public double[] aeroCalBinsLift = null;
        public double[] aeroCalBinsDrag = null;
        public int[] aeroCalBinsSamples = null;
        // f67: boards-out lift bands (boards measurably CHANGE lift - on
        // 星舰货运 kL 1.58 -> 3.0 with boards out - so one band set cannot
        // serve both states without poisoning the seed it saves)
        public double[] aeroCalBinsLiftAb = null;
        public int[] aeroCalBinsSamplesAb = null;
        // The file's global kL/kD at seed time this flight - SaveAeroCalToFile
        // falls back to these when band 0 (the cruise regime) has too few
        // samples, so a flight that only sampled ONE regime (f66: 464
        // samples all at q>10kPa) cannot drag the saved global to a value
        // that poisons the next flight's low-q prediction (1.54 -> 0.39)
        private double aeroCalSeedLift = 1;
        private double aeroCalSeedDrag = 1;

        // PluginData aero-cal file (f63): the per-save KSPFields above die
        // with every revert-to-quicksave - the quicksave predates the cal, so
        // each retry relearns the glide from scratch, the high-altitude
        // prediction starts ~30 km short-biased, the user trims THAT, and
        // then watches the cross walk forward as kL converges mid-glide
        // ("大气滑翔导致预测落点一直在前移"). Persist per-vessel to a file
        // that survives reverts; the file wins over the KSPField at enable
        private static Dictionary<string, double[]> aeroCalFile = null; // vessel name -> [kL, kD, samples] (+ f64: kL0..2, kD0..2, n0..2 when present)
        private static bool aeroCalFileLoaded = false;
        private double lastAeroCalFileSave = -100;
        private int lastAeroCalSavedN = -1;

        private static string AeroCalPath()
        {
            return KSPUtil.ApplicationRootPath + "GameData/BoosterGuidance/PluginData/aerocal.cfg";
        }

        private static double[] ParseCalList(string s, double fill)
        {
            string[] toks = s.Split(',');
            double[] r = new double[toks.Length];
            for (int i = 0; i < toks.Length; i++)
            {
                double d;
                r[i] = double.TryParse(toks[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out d) ? d : fill;
            }
            return r;
        }

        private static void LoadAeroCalFile()
        {
            aeroCalFile = new Dictionary<string, double[]>();
            aeroCalFileLoaded = true;
            try
            {
                string path = AeroCalPath();
                if (!File.Exists(path))
                    return;
                foreach (string line in File.ReadAllLines(path))
                {
                    string[] parts = line.Split('\t');
                    if (parts.Length < 4)
                        continue;
                    double kL, kD, n;
                    if (double.TryParse(parts[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out kL)
                        && double.TryParse(parts[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out kD)
                        && double.TryParse(parts[3], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out n))
                    {
                        double[] vals = new double[] { kL, kD, n };
                        // f64 banded extension: kL0,kL1,kL2 <tab> kD0..2 <tab> n0..2
                        if (parts.Length >= 7)
                        {
                            double[] lq = ParseCalList(parts[4], 1);
                            double[] dq = ParseCalList(parts[5], 1);
                            double[] nq = ParseCalList(parts[6], 0);
                            if ((lq.Length == 3) && (dq.Length == 3) && (nq.Length == 3))
                            {
                                Array.Resize(ref vals, 12);
                                for (int i = 0; i < 3; i++)
                                {
                                    vals[3 + i] = lq[i];
                                    vals[6 + i] = dq[i];
                                    vals[9 + i] = nq[i];
                                }
                                // f67 boards-out lift bands: kLab0..2 <tab> nab0..2
                                if (parts.Length >= 9)
                                {
                                    double[] lab = ParseCalList(parts[7], 1);
                                    double[] nab = ParseCalList(parts[8], 0);
                                    if ((lab.Length == 3) && (nab.Length == 3))
                                    {
                                        Array.Resize(ref vals, 18);
                                        for (int i = 0; i < 3; i++)
                                        {
                                            vals[12 + i] = lab[i];
                                            vals[15 + i] = nab[i];
                                        }
                                    }
                                }
                            }
                        }
                        aeroCalFile[parts[0]] = vals;
                    }
                }
            }
            catch (Exception e)
            {
                Log.Info("aerocal.cfg load failed: " + e.Message);
            }
        }

        // Seed the core's cal fields from the file BEFORE ConfigureController
        // pushes them into the fresh controller (and the deorbit scope reads
        // the same fields, so the pre-enable cross gets the realized aero too)
        private void SeedAeroCalFromFile()
        {
            if (!aeroCalFileLoaded)
                LoadAeroCalFile();
            if ((vessel == null) || (aeroCalFile == null))
                return;
            double[] vals;
            if (aeroCalFile.TryGetValue(vessel.name, out vals))
            {
                aeroCalLift = vals[0];
                aeroCalDrag = vals[1];
                if (vals.Length >= 18)
                {
                    aeroCalBinsLift = new double[] { vals[3], vals[4], vals[5] };
                    aeroCalBinsDrag = new double[] { vals[6], vals[7], vals[8] };
                    aeroCalBinsSamples = new int[] { (int)vals[9], (int)vals[10], (int)vals[11] };
                    aeroCalBinsLiftAb = new double[] { vals[12], vals[13], vals[14] };
                    aeroCalBinsSamplesAb = new int[] { (int)vals[15], (int)vals[16], (int)vals[17] };
                    Log.Info(string.Format("[AeroCal] seeded from file: vessel={0} kL={1:F2} kD={2:F2} n={3:F0} kLq={4:F2}/{5:F2}/{6:F2} kDq={7:F2}/{8:F2}/{9:F2} nq={10}/{11}/{12} kLqab={13:F2}/{14:F2}/{15:F2} nqab={16}/{17}/{18}",
                        vessel.name, aeroCalLift, aeroCalDrag, vals[2],
                        vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], (int)vals[9], (int)vals[10], (int)vals[11],
                        vals[12], vals[13], vals[14], (int)vals[15], (int)vals[16], (int)vals[17]));
                }
                else if (vals.Length >= 12)
                {
                    aeroCalBinsLift = new double[] { vals[3], vals[4], vals[5] };
                    aeroCalBinsDrag = new double[] { vals[6], vals[7], vals[8] };
                    aeroCalBinsSamples = new int[] { (int)vals[9], (int)vals[10], (int)vals[11] };
                    // v2 migration: these bands may blend brakes-in and
                    // brakes-out samples (f67's band 0 read 2.88 from that
                    // mix) - carry them as boards-IN, start the boards-out
                    // bands from the same values with no samples (thin ->
                    // fallback), and let the next flight re-split them
                    aeroCalBinsLiftAb = new double[] { vals[3], vals[4], vals[5] };
                    aeroCalBinsSamplesAb = new int[] { 0, 0, 0 };
                    Log.Info(string.Format("[AeroCal] seeded from file: vessel={0} kL={1:F2} kD={2:F2} n={3:F0} kLq={4:F2}/{5:F2}/{6:F2} kDq={7:F2}/{8:F2}/{9:F2} nq={10}/{11}/{12}",
                        vessel.name, aeroCalLift, aeroCalDrag, vals[2],
                        vals[3], vals[4], vals[5], vals[6], vals[7], vals[8], (int)vals[9], (int)vals[10], (int)vals[11]));
                }
                else
                {
                    // f66: a legacy 3-field entry left the bands at the 1.0
                    // defaults - the next save then wrote those defaults over
                    // regimes the global already knew (kL 1.54 at low q).
                    // Replicate the legacy global into every band instead
                    aeroCalBinsLift = new double[] { aeroCalLift, aeroCalLift, aeroCalLift };
                    aeroCalBinsDrag = new double[] { aeroCalDrag, aeroCalDrag, aeroCalDrag };
                    aeroCalBinsSamples = new int[] { 0, 0, 0 };
                    aeroCalBinsLiftAb = new double[] { aeroCalLift, aeroCalLift, aeroCalLift };
                    aeroCalBinsSamplesAb = new int[] { 0, 0, 0 };
                    Log.Info(string.Format("[AeroCal] seeded from file: vessel={0} kL={1:F2} kD={2:F2} n={3:F0}", vessel.name, aeroCalLift, aeroCalDrag, vals[2]));
                }
            }
            // Whatever the file said (or did not), THIS is the global the
            // save may fall back to when band 0 never gets thick this flight
            aeroCalSeedLift = aeroCalLift;
            aeroCalSeedDrag = aeroCalDrag;
        }

        private void SaveAeroCalToFile(string why)
        {
            if ((controller == null) || (vessel == null))
                return;
            if (controller.calSamples < 20) // too few samples to trust
                return;
            if (!aeroCalFileLoaded)
                LoadAeroCalFile();
            try
            {
                double[] vals = new double[18];
                bool binsOk = (controller.aeroCalLiftQ != null) && (controller.aeroCalLiftQ.Length == 3)
                    && (controller.aeroCalDragQ != null) && (controller.aeroCalDragQ.Length == 3)
                    && (controller.aeroCalSamplesQ != null) && (controller.aeroCalSamplesQ.Length == 3);
                // f66: the global the file keeps must stay LOW-q knowledge.
                // A flight that sampled only ONE regime (464 samples all at
                // q>10kPa) dragged the saved global 1.54 -> 0.39 and the next
                // flight's whole glide predicted short from that poisoned
                // seed. Band 0 is the cruise regime: when it stands on its
                // own it IS the best global; otherwise keep what the file
                // already knew at seed time
                if (binsOk)
                {
                    vals[0] = (controller.aeroCalSamplesQ[0] >= controller.calBinMinSamples) ? controller.aeroCalLiftQ[0] : aeroCalSeedLift;
                    vals[1] = (controller.aeroCalSamplesQ[0] >= controller.calBinMinSamples) ? controller.aeroCalDragQ[0] : aeroCalSeedDrag;
                }
                else
                {
                    vals[0] = controller.aeroCalLift;
                    vals[1] = controller.aeroCalDrag;
                }
                vals[2] = controller.calSamples;
                if (binsOk)
                    for (int i = 0; i < 3; i++)
                    {
                        vals[3 + i] = controller.aeroCalLiftQ[i];
                        vals[6 + i] = controller.aeroCalDragQ[i];
                        vals[9 + i] = controller.aeroCalSamplesQ[i];
                        // f67 boards-out lift bands; a corrupt/short boards-
                        // out array degrades to the boards-in band + 0
                        // samples (thin -> fallback) rather than a throw
                        vals[12 + i] = ((controller.aeroCalLiftAbQ != null) && (controller.aeroCalLiftAbQ.Length == 3)) ? controller.aeroCalLiftAbQ[i] : controller.aeroCalLiftQ[i];
                        vals[15 + i] = ((controller.aeroCalSamplesAbQ != null) && (controller.aeroCalSamplesAbQ.Length == 3)) ? controller.aeroCalSamplesAbQ[i] : 0;
                    }
                else
                    Array.Resize(ref vals, 3); // f64 guard: legacy 3-field line rather than a throw
                aeroCalFile[vessel.name] = vals;
                System.Text.StringBuilder sb = new System.Text.StringBuilder();
                foreach (var kv in aeroCalFile)
                {
                    sb.Append(kv.Key.Replace('\t', ' ')).Append('\t')
                        .Append(kv.Value[0].ToString("F4", System.Globalization.CultureInfo.InvariantCulture)).Append('\t')
                        .Append(kv.Value[1].ToString("F4", System.Globalization.CultureInfo.InvariantCulture)).Append('\t')
                        .Append(kv.Value[2]);
                    if (kv.Value.Length >= 12)
                    {
                        sb.Append('\t');
                        for (int i = 3; i < 6; i++)
                        { if (i > 3) sb.Append(','); sb.Append(kv.Value[i].ToString("F4", System.Globalization.CultureInfo.InvariantCulture)); }
                        sb.Append('\t');
                        for (int i = 6; i < 9; i++)
                        { if (i > 6) sb.Append(','); sb.Append(kv.Value[i].ToString("F4", System.Globalization.CultureInfo.InvariantCulture)); }
                        sb.Append('\t');
                        for (int i = 9; i < 12; i++)
                        { if (i > 9) sb.Append(','); sb.Append((int)kv.Value[i]); }
                    }
                    if (kv.Value.Length >= 18)
                    {
                        sb.Append('\t');
                        for (int i = 12; i < 15; i++)
                        { if (i > 12) sb.Append(','); sb.Append(kv.Value[i].ToString("F4", System.Globalization.CultureInfo.InvariantCulture)); }
                        sb.Append('\t');
                        for (int i = 15; i < 18; i++)
                        { if (i > 15) sb.Append(','); sb.Append((int)kv.Value[i]); }
                    }
                    sb.Append('\n');
                }
                File.WriteAllText(AeroCalPath(), sb.ToString());
                Log.Info(string.Format("[AeroCal] saved to file ({0}): vessel={1} kL={2:F2} kD={3:F2} n={4}", why, vessel.name, vals[0], vals[1], controller.calSamples));
            }
            catch (Exception e)
            {
                Log.Info("aerocal.cfg save failed: " + e.Message);
            }
        }

        // Below this height above the target, LandingBurn commands RCS
        // translation against the horizontal velocity. DISABLED by default
        // (0): with top-mounted RCS (the usual booster layout) translation
        // thrust acts far from the centre of mass and the induced torque can
        // tip the vessel over near the ground. Set >0 only if your RCS sits
        // near the CoM. Renamed when defaulted off so a persisted 350 in
        // existing saves is ignored
        [KSPField(isPersistant = true, guiActive = false)]
        public double rcsTranslateAlt = 0;

        [KSPField(isPersistant = true, guiActive = false)]
        public bool deployLandingGear = true;

        [KSPField(isPersistant = true, guiActive = false)]
        public int deployLandingGearHeight = 500;

        [KSPField(isPersistant = true, guiActive = false)]
        public string landingBurnEngines = "current";

        [KSPField(isPersistant = true, guiActive = false)]
        public float igniteDelay = 3;

        [KSPField(isPersistant = true, guiActive = false)]
        public string phase = "Unset";

        // Recovery profile: "falcon9" (default, tail-first retrograde) or
        // "starship" (belly flop + flip). Never guessed - switched
        // explicitly in the GUI
        [KSPField(isPersistant = true, guiActive = false)]
        public string recoveryProfile = "falcon9";

        // Belly axis roll offset (degrees, about ReferenceTransform.up):
        // belly axis = ReferenceTransform.forward rolled by this. Calibrated
        // once in the GUI on the first starship flight (design D4)
        [KSPField(isPersistant = true, guiActive = false)]
        public double bellyRollOffset = 0;

        // Belly-flop correction master gain (0-1, design D7): scales the
        // impact-error correction live, editable mid-flight. 0 = dumb glide
        // (pure attitude hold) for the T2 authority test
        [KSPField(isPersistant = true, guiActive = false)]
        public double starshipCorrectionGain = 1.0;
        // Belly-flop AoA schedule ends (GUI-tunable; the q thresholds stay at
        // the flight-46-derived 300/3000 Pa in BLController)
        public double bellyBrakeAoA = 85;
        public double bellyGlideAoA = 18;

        // f83 cleanup: the standalone impact prediction (deorbit scope) and
        // the aim burn were deleted - the validated Trajectories-driven
        // framework (TrajCal) replaced them; the Trajectories red cross is
        // the single authoritative impact marker

        // Passive glide (user request after f70-f72): pure aero-brake descent
        // - belly AoA schedule + straight trajectory, NO impact prediction,
        // NO aero calibration, NO airbrake/engine overshoot brake, NO aim
        // burn, no prediction cross, no deorbit scope. The pilot compares
        // MJ's landing prediction against the unperturbed real trajectory.
        // Flip + landing burn still fly (they are not prediction). DEFAULT
        // ON per the user request; persisted per-save
        [KSPField(isPersistant = true, guiActive = false)]
        public bool starshipPassiveGlide = true;
        // Traj-driven calibration (user request after f76): calibration
        // reads TRAJECTORIES' predicted impact (reflection, optional dep)
        // instead of our own sim - overshoot -> airbrakes, short -> modest
        // extend burn, lateral rides the belly correction. Combines with
        // passive glide; DEFAULT ON per the user request
        [KSPField(isPersistant = true, guiActive = false)]
        public bool starshipTrajCal = true;
        // f85 暴力减速 (violent brake): below 10 km, if the Traj red mark is
        // still 1+ km long, force the full-brake AoA for max drag (details on
        // the controller's trajCalViolent* fields). DEFAULT ON per the user
        // request; persisted per-save
        [KSPField(isPersistant = true, guiActive = false)]
        public bool starshipViolentBrake = true;
        // f104: the f85 experimental 高空校准点火 (>55km engine fine-trim)
        // is REMOVED (user: "5w5米以上的实验性发动机调控功能去掉吧不好用") -
        // the high-alt mark is the phantom regime, a fine trim could only
        // chase phantoms. The persisted field is gone with it (KSP ignores
        // unknown fields in old saves)
        // f97 高空大偏差反推: when the high-alt Traj mark is more than 30 km
        // out, a gentle falcon9-style retro/pro burn fixes it where dV is
        // cheapest - throttle HARD-CAPPED at 10%, point-first (the 10-deg
        // steer gate holds the thrust until aligned), landing-reserve fuel
        // floor. DEFAULT ON per the user request; persisted per-save
        [KSPField(isPersistant = true, guiActive = false)]
        public bool starshipBigTrim = true;
        // f95 比例抬头减速 (proportional AoA governor): below 5 km the Traj
        // mark's overshoot is killed by raising the nose proportionally
        // instead of the bang-bang 85-deg violent brake - the ship keeps
        // steerable vh (f95: the slam left it an unguided leaf, 266m ->
        // 1095m). DEFAULT ON per the user design; persisted per-save
        [KSPField(isPersistant = true, guiActive = false)]
        public bool starshipAoaMod = true;

        // Pilot burn attitude buttons (f62 user redesign, GUI 顺向/逆向):
        // +1 = prograde (前进), -1 = retro (刹车), 0 = off. Written by the
        // MainWindow toggles, pushed to the controller every Fly tick. NOT
        // persisted - a momentary in-flight tool, always starts OFF
        public int pilotAttitudeMode = 0;

        // Emergency land-anywhere (GUI 应急着陆 button): enable guidance
        // immediately with the target anchored under the vessel; the
        // controller then replaces all homing with horizontal-drift braking
        // and lands vertically wherever the ship is. The real target is
        // saved here and restored when guidance is disabled
        public bool emergencyLanding = false;
        private double savedTgtLat = 0;
        private double savedTgtLon = 0;
        private double savedTgtAlt = 0;

        // Manual-flight input recorder (user flies the reentry by hand,
        // falcon landing does the rest): logs pilot inputs + attitude
        // response at 20 Hz even with guidance DISABLED, so the flown AoA
        // profile can be replayed against the schedule afterwards. Toggle
        // in the GUI; writes <vessel>.Input.dat next to Actual.dat
        [KSPField(isPersistant = true, guiActive = false)]
        public bool flightRecorderEnabled = false;
        private System.IO.StreamWriter inputLog = null;
        private float lastInputLog = -1;

        // Starship attitude PD (writes FlightCtrlState directly - SAS has no
        // roll channel) and the attitude-tracking failure detector (design
        // D9: warn only, no fallback)
        private StarshipAttitudeController starshipAtt = new StarshipAttitudeController();
        private double attFailTime = 0;
        private bool attFailWarned = false;
        private bool starshipWarpCutNotified = false;
        private bool nanCanaryFired = false;
        private bool velCanaryFired = false;
        private bool starshipSasForceLogged = false;
        private float lastTorqueDiag = -10;

        [KSPField(isPersistant = true, guiActive = false)]
        public float aeroMult = 1;

        [KSPField(isPersistant = true, guiActive = false)]
        public bool enableRCS = true;
        [KSPField(isPersistant = true, guiActive = false)]
        public int actionGroup = 0;
        [KSPField(isPersistant = true, guiActive = false)]
        public float degreeRange = 10f;

        // Hotkey (user request 2026-09-12: 一键设目标+开引导). KeyCode name
        // as a plain string so a bad/empty entry just disables the binding;
        // hotkeySiteName is the landing-site preset the hotkey aims at (star
        // button in the sites list). NO emergency hotkey: it conflicted with
        // the user's camera tool (End pressed for the camera fired
        // EmergencyLand mid-descent twice - f134 ocean splash) - the corner
        // red button stays the only emergency trigger
        [KSPField(isPersistant = true, guiActive = false)]
        public string hotkeyGuidance = "Backspace";
        [KSPField(isPersistant = true, guiActive = false)]
        public string hotkeySiteName = "";

        public double last_t = 0;
        public double last_throttle = 0;
        public Vector3d last_steer = Vector3d.zero;
        // f99: the steer-angle gate below ran every frame but steerAngle was
        // a per-call LOCAL reset to 0 - and GetControlOutputs only re-runs at
        // ~10 Hz above tgtAlt+500, so on every skipped frame the gate saw
        // angle 0 and passed the last commanded throttle straight through.
        // During a big-trim swing-in that leaked full commanded thrust while
        // the nose was still tens of degrees off target (f99: 35 m/s burned
        // in 3.6 s with the controller-side pointing gate never opening;
        // user: "还没转到合适的地方就开始点火"). Persist the angle between
        // updates like throttle/steer so the gate holds between updates
        private double last_steerAngle = 0;
        private bool attDiagnosticLogged = false;
        // f96 landing-fuel reserve watch: f96 ended in a fuel-out crash at
        // 1.6 km because the pilot chased the swinging high-altitude red mark
        // with three manual retro burns (~17.5 t of propellant, ~1 km/s) and
        // nobody told them the landing reserve was gone. Once a second we
        // compute the usable dV (KSPUtils.GetAverageIsp + ComputeUsable
        // PropellantKg) and warn when it closes on the landing reserve.
        // dvAvailable is shown in the window so the reserve is always visible
        public double dvAvailable = -1; // m/s, -1 = not computed yet
        public double landingReserveDv = 350; // m/s - a full flip+landing burn costs ~200-300; 350 adds margin
        private double lastDvCheckT = -1;
        private double lastDvWarnT = -99;
        public bool logging = false;
        public bool useFAR = false;
        public bool debug = false;
        public string logFilename = "unset";
        private string info = "Disabled";
        private bool reportedLandingGear = false;

        // Flight controller with copy of these settings
        BLController controller = null;

        // List of all active controllers
        public static List<BLController> controllers = new List<BLController>();

        public void OnDestroy()
        {
            if (controller != null)
                DisableGuidance();
            if (inputLog != null)
            {
                inputLog.Close();
                inputLog = null;
            }
        }

        public void OnCrash()
        {
            // TODO - does this work?
            if ((controller != null) && (controller.vessel = FlightGlobals.ActiveVessel) && (controller.enabled))
                GuiUtils.ScreenMessage("Vessel crashed - Try increasing touchdown margin in the advanced tab");
        }

        // Find first BoosterGuidanceCore module for vessel
        static public BoosterGuidanceCore GetBoosterGuidanceCore(Vessel vessel)
        {
            foreach (var part in vessel.Parts)
            {
                foreach (var mod in part.Modules)
                {
                    if (mod.GetType() == typeof(BoosterGuidanceCore))
                    {
                        //Log.Info("vessel=" + vessel.name + " part=" + part.name + " module=" + mod.name + " modtype=" + mod.GetType());
                        return (BoosterGuidanceCore)mod;
                    }
                }
            }
            //Log.Info("No BoosterGuidanceCore module for vessel " + vessel.name);
            return null;
        }

        public void AttachVessel(Vessel vessel)
        {
            // Sets up Aero forces function again respected useFAR flag
            controller.AttachVessel(vessel, useFAR);
        }

        public void AddController(BLController controller)
        {
            controllers.Add(controller);
        }

        public void RemoveController(BLController controller)
        {
            controllers.Remove(controller);
        }

        public void CopyToOtherCore(BoosterGuidanceCore other)
        {
            other.deployLandingGear = deployLandingGear;
            other.deployLandingGearHeight = deployLandingGearHeight;
            other.landingBurnEngines = landingBurnEngines;
            other.landingBurnMaxAoA = landingBurnMaxAoA;
            other.landingBurnSteerKp = landingBurnSteerKp;
            other.aeroDescentMaxAoA = aeroDescentMaxAoA;
            other.aeroDescentSteerKp = aeroDescentSteerKp;
            other.reentryBurnAlt = reentryBurnAlt;
            other.reentryBurnMaxAoA = reentryBurnMaxAoA;
            other.reentryBurnSteerKp = reentryBurnSteerKp;
            other.reentryBurnTargetSpeed = reentryBurnTargetSpeed;
            other.tgtAlt = tgtAlt;
            other.tgtLatitude = tgtLatitude;
            other.tgtSet = tgtSet;
            other.tgtLongitude = tgtLongitude;
            other.touchdownMargin = touchdownMargin;
            other.touchdownSpeed = touchdownSpeed;
            other.noSteerHeight = noSteerHeight;
            other.uprightHeight = uprightHeight;
            other.uprightMaxHorizSpeed = uprightMaxHorizSpeed;
            other.steerDamping = steerDamping;
            other.landingBurnVelDampSec = landingBurnVelDampSec;
            other.lowAoACap = lowAoACap;
            other.aoaRampLowAlt = aoaRampLowAlt;
            other.aoaRampTopAlt = aoaRampTopAlt;
            other.coastAutoWarp = coastAutoWarp;
            other.coastAutoWarpRate = coastAutoWarpRate;
            other.autoWarp = autoWarp;
            other.aeroCalLift = aeroCalLift;
            other.aeroCalDrag = aeroCalDrag;
            other.aeroCalBinsLift = ((aeroCalBinsLift != null) && (aeroCalBinsLift.Length == 3)) ? (double[])aeroCalBinsLift.Clone() : null;
            other.aeroCalBinsDrag = ((aeroCalBinsDrag != null) && (aeroCalBinsDrag.Length == 3)) ? (double[])aeroCalBinsDrag.Clone() : null;
            other.aeroCalBinsSamples = ((aeroCalBinsSamples != null) && (aeroCalBinsSamples.Length == 3)) ? (int[])aeroCalBinsSamples.Clone() : null;
            other.aeroCalBinsLiftAb = ((aeroCalBinsLiftAb != null) && (aeroCalBinsLiftAb.Length == 3)) ? (double[])aeroCalBinsLiftAb.Clone() : null;
            other.aeroCalBinsSamplesAb = ((aeroCalBinsSamplesAb != null) && (aeroCalBinsSamplesAb.Length == 3)) ? (int[])aeroCalBinsSamplesAb.Clone() : null;
            other.rcsTranslateAlt = rcsTranslateAlt;
            other.igniteDelay = igniteDelay;
            other.phase = phase;
            other.recoveryProfile = recoveryProfile;
            other.bellyRollOffset = bellyRollOffset;
            other.starshipCorrectionGain = starshipCorrectionGain;
            other.bellyBrakeAoA = bellyBrakeAoA;
            other.bellyGlideAoA = bellyGlideAoA;
            other.starshipPassiveGlide = starshipPassiveGlide;
            other.starshipTrajCal = starshipTrajCal;
            other.starshipViolentBrake = starshipViolentBrake;
            other.starshipAoaMod = starshipAoaMod;
            other.starshipBigTrim = starshipBigTrim;
            other.emergencyLanding = emergencyLanding;
            other.flightRecorderEnabled = flightRecorderEnabled;
        }

        public void SetTarget(double latitude, double longitude, double alt)
        {
            tgtSet = true;
            tgtLatitude = latitude;
            tgtLongitude = longitude;
            tgtSet = true;

            tgtAlt = (int)alt;
            if (controller != null)
                controller.SetTarget(latitude, longitude, alt);
        }

        public void SetPhase(BLControllerPhase phase)
        {
            if (controller != null)
                controller.SetPhase(phase);
        }

        public BLControllerPhase Phase()
        {
            if (controller != null)
                return controller.phase;
            else
                return BLControllerPhase.Unset;
        }

        public string SetLandingBurnEngines()
        {
            List<ModuleEngines> activeEngines = KSPUtils.GetActiveEngines(vessel);
            // get string
            List<string> s = new List<string>();
            int num = 0;
            foreach (var engine in KSPUtils.GetAllEngines(vessel))
            {
                if (activeEngines.Contains(engine))
                {
                    s.Add("1");
                    num++;
                }
                else
                    s.Add("0");
            }
            landingBurnEngines = String.Join(",", s.ToArray());
            Log.Info("landingBurnEngines=" + landingBurnEngines);
            if (controller != null)
                controller.SetLandingBurnEnginesFromString(landingBurnEngines);
            return num.ToString();
        }

        public string UnsetLandingBurnEngines()
        {
            Log.Info("UnsetLandingBurnEngines");
            landingBurnEngines = "current";
            if (controller != null)
                controller.SetLandingBurnEnginesFromString(landingBurnEngines);
            return landingBurnEngines;
        }

        public double LandingBurnHeight()
        {
            if (controller != null)
                return controller.landingBurnHeight;
            else
                return 0;
        }

        public bool Enabled()
        {
            return (controller != null) && (controller.enabled);
        }

        // BoosterGuidanceCore params changed so update controller
        public void Changed()
        {
            if (controller != null)
                ConfigureController(controller);
            CopyToOtherCores();
        }

        // Push all persisted guidance settings into a controller (the active
        // guidance controller or the background return-fuel hint controller)
        private void ConfigureController(BLController c)
        {
            c.InitReentryBurn(reentryBurnSteerKp, reentryBurnMaxAoA, reentryBurnAlt, reentryBurnTargetSpeed);
            c.InitAeroDescent(aeroDescentSteerKp, aeroDescentMaxAoA);
            c.InitLandingBurn(landingBurnSteerKp, landingBurnMaxAoA);
            c.SetTarget(tgtLatitude, tgtLongitude, tgtAlt);
            c.touchdownMargin = touchdownMargin;
            c.touchdownSpeed = touchdownSpeed;
            c.noSteerHeight = noSteerHeight;
            c.uprightHeight = uprightHeight;
            c.uprightMaxHorizSpeed = uprightMaxHorizSpeed;
            c.steerDamping = steerDamping;
            c.landingBurnVelDamp = landingBurnVelDampSec;
            c.lowAoACap = lowAoACap;
            c.aoaRampLowAlt = aoaRampLowAlt;
            c.aoaRampTopAlt = aoaRampTopAlt;
            c.deployLandingGear = deployLandingGear;
            c.deployLandingGearHeight = deployLandingGearHeight;
            c.igniteDelay = igniteDelay;
            c.recoveryProfile = recoveryProfile;
            c.starshipCorrectionGain = starshipCorrectionGain;
            c.bellyRollOffset = bellyRollOffset;
            c.bellyBrakeAoA = bellyBrakeAoA;
            c.bellyGlideAoA = bellyGlideAoA;
            c.starshipPassiveGlide = starshipPassiveGlide;
            c.starshipTrajCal = starshipTrajCal;
            c.emergencyLanding = emergencyLanding;
            c.aeroCalLift = aeroCalLift;
            c.aeroCalDrag = aeroCalDrag;
            if ((aeroCalBinsLift != null) && (aeroCalBinsLift.Length == 3)
                && (aeroCalBinsDrag != null) && (aeroCalBinsDrag.Length == 3)
                && (aeroCalBinsSamples != null) && (aeroCalBinsSamples.Length == 3))
            {
                c.aeroCalLiftQ = (double[])aeroCalBinsLift.Clone();
                c.aeroCalDragQ = (double[])aeroCalBinsDrag.Clone();
                c.aeroCalSamplesQ = (int[])aeroCalBinsSamples.Clone();
            }
            if ((aeroCalBinsLiftAb != null) && (aeroCalBinsLiftAb.Length == 3)
                && (aeroCalBinsSamplesAb != null) && (aeroCalBinsSamplesAb.Length == 3))
            {
                c.aeroCalLiftAbQ = (double[])aeroCalBinsLiftAb.Clone();
                c.aeroCalSamplesAbQ = (int[])aeroCalBinsSamplesAb.Clone();
            }
            c.SetLandingBurnEnginesFromString(landingBurnEngines);
        }

        public void CopyToOtherCores()
        {
            foreach (var part in vessel.Parts)
            {
                foreach (var mod in part.Modules)
                {
                    if (mod.GetType() == typeof(BoosterGuidanceCore))
                    {
                        var other = (BoosterGuidanceCore)mod;
                        CopyToOtherCore(other);
                    }
                }
            }
        }

        [KSPAction("Toggle BoosterGuidance")]
        public void ToggleGuidance(KSPActionParam param)
        {
            if (controller == null)
                EnableGuidance(param);
            else
                DisableGuidance(param);
        }

        public void EnableGuidance()
        {
            KSPActionParam param = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);
            EnableGuidance(param);

            FireEvent(actionGroup);
            if (enableRCS)
                FireEvent(12);
        }

        public static Dictionary<int, String> KM_dictAGNames = new Dictionary<int, String> {
            { 0,  "Stage" },
            { 1,  "Custom01" },
            { 2,  "Custom02" },
            { 3,  "Custom03" },
            { 4,  "Custom04" },
            { 5,  "Custom05" },
            { 6,  "Custom06" },
            { 7,  "Custom07" },
            { 8,  "Custom08" },
            { 9,  "Custom09" },
            { 10, "Custom10" },
            { 11, "Light" },
            { 12, "RCS" },
            { 13, "SAS" },
            { 14, "Brakes" },
            { 15, "Abort" },
            { 16, "Gear" },
            { 17, "Beep" },
        };

        public static Dictionary<int, KSPActionGroup> KM_dictAG = new Dictionary<int, KSPActionGroup> {
            { 0,  KSPActionGroup.None },
            { 1,  KSPActionGroup.Custom01 },
            { 2,  KSPActionGroup.Custom02 },
            { 3,  KSPActionGroup.Custom03 },
            { 4,  KSPActionGroup.Custom04 },
            { 5,  KSPActionGroup.Custom05 },
            { 6,  KSPActionGroup.Custom06 },
            { 7,  KSPActionGroup.Custom07 },
            { 8,  KSPActionGroup.Custom08 },
            { 9,  KSPActionGroup.Custom09 },
            { 10, KSPActionGroup.Custom10 },
            { 11, KSPActionGroup.Light },
            { 12, KSPActionGroup.RCS },
            { 13, KSPActionGroup.SAS },
            { 14, KSPActionGroup.Brakes },
            { 15, KSPActionGroup.Abort },
            { 16, KSPActionGroup.Gear }
        };

        public void FireEvent(int eventID)
        {
            if (eventID > 0)
            {
                Log.Info("Fire Event " + KM_dictAGNames[eventID]);
                vessel.ActionGroups.ToggleGroup(KM_dictAG[eventID]);
            }
        }



        [KSPAction("Enable BoosterGuidance")]
        public void EnableGuidance(KSPActionParam param)
        {
            SeedAeroCalFromFile(); // f63: revert-proof per-vessel kL/kD - the quicksave restores PRE-cal KSPFields, the file remembers
            controller = new BLController(vessel, useFAR);
            reportedLandingGear = false;
            waitingForDescent = vessel.checkLanded(); // armed on the pad: activate at apex
            Changed(); // updates controller

            if ((tgtLatitude == 0) && (tgtLongitude == 0) && (tgtAlt == 0))
            {
                Log.Info("EnableGuidance refused: no target set (vessel=" + vessel.name + ")");
                GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_NoTargetSet"));
                return;
            }
            if (!controller.enabled)
            {
                Log.Info("Enabled Guidance for vessel " + FlightGlobals.ActiveVessel.name + " (module vessel=" + this.vessel.name + " landed=" + this.vessel.checkLanded() + " waitingForDescent=" + waitingForDescent + ")");
                Vessel vessel = FlightGlobals.ActiveVessel;
                Targets.RedrawTarget(vessel.mainBody, tgtLatitude, tgtLongitude, tgtAlt);
                // The starship profile flies its own attitude PD in Fly;
                // enabling SAS too would double-command the controls
                if (recoveryProfile != "starship")
                    vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.StabilityAssist);
                if (logging)
                    StartLogging();
                // f105: EnableGuidance recreates `controller` above, so the
                // old !controller.enabled check always passed and a second
                // enable (action group + GUI, f105: 16:20:38 AND 16:20:48)
                // registered Fly TWICE - the controller double-stepped all
                // flight and every downstream per-frame edge detector broke
                if (!flyRegistered)
                {
                    flyRegistered = true;
                    vessel.OnFlyByWire += new FlightInputCallback(Fly);
                }
            }
            else
                Log.Info("EnableGuidance: controller already enabled");
            controller.SetPhase(BLControllerPhase.Unset);
            controller.enabled = true;
            AddController(controller);
        }

        public void StartLogging()
        {
            if (controller != null)
                controller.StartLogging(logFilename);
        }

        public void StopLogging()
        {
            if (controller != null)
                controller.StopLogging();
        }

        public void DisableGuidance()
        {
            KSPActionParam param = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);
            DisableGuidance(param);
        }

        // Emergency land-anywhere: save the real target, anchor the target
        // under the vessel (the controller refreshes the anchor at 1 Hz from
        // here on) and enable guidance with the profile's normal phase
        // auto-pick - belly flop / flip / landing burn run as usual, only
        // the homing is replaced by horizontal-drift braking
        public void EmergencyLand()
        {
            Vessel av = FlightGlobals.ActiveVessel;
            if (av == null)
                return;
            if (!emergencyLanding)
            {
                savedTgtLat = tgtLatitude;
                savedTgtLon = tgtLongitude;
                savedTgtAlt = tgtAlt;
            }
            emergencyLanding = true;
            SetTarget(av.latitude, av.longitude, Math.Max(0, av.altitude - av.radarAltitude));
            Log.Info("EmergencyLand: anchor=" + av.latitude.ToString("F4") + "," + av.longitude.ToString("F4") + " alt=" + tgtAlt);
            GuiUtils.ScreenMessage("EMERGENCY LANDING");
            if (!Enabled())
                EnableGuidance();
            else
            {
                Changed(); // push emergencyLanding into the running controller
                SetPhase(BLControllerPhase.Unset); // re-pick: emergency skips the belly flop
            }
        }

        // Airbrake overshoot control (f60 user request): the controller's
        // overshoot-brake arm condition also deploys stock airbrakes - free
        // drag, no rotation, no propellant, no dV-budget gate. Stock
        // airbrakes are ModuleAeroSurface with useInternalDragModel; control
        // surfaces are the ModuleControlSurface SUBCLASS and must NOT be
        // touched (those are the glide flaps), hence the exact-type test.
        // The prediction sim does NOT model the deployed drag - the live
        // aero calibration's kD EMA absorbs it
        private List<ModuleAeroSurface> airbrakeModules = null;
        private bool airbrakesDeployed = false;
        private void ApplyAirbrakes(bool want)
        {
            if ((vessel == null) || (!vessel))
                return;
            if (airbrakeModules == null)
            {
                airbrakeModules = new List<ModuleAeroSurface>();
                foreach (var part in vessel.Parts)
                    foreach (var m in part.FindModulesImplementing<ModuleAeroSurface>())
                        if ((m.GetType() == typeof(ModuleAeroSurface)) && (m.useInternalDragModel))
                            airbrakeModules.Add(m);
                Log.Info("[Airbrake] scan: " + airbrakeModules.Count + " stock airbrake module(s) on " + vessel.Parts.Count + " parts");
            }
            if ((airbrakeModules.Count == 0) || (want == airbrakesDeployed))
                return;
            foreach (var m in airbrakeModules)
                m.deploy = want;
            airbrakesDeployed = want;
            Log.Info("[Airbrake] " + (want ? "DEPLOY" : "RETRACT") + " n=" + airbrakeModules.Count
                + " terr=" + ((controller != null) ? controller.targetError.ToString("F0") : "?"));
        }

        [KSPAction("Disable BoosterGuidance")]
        public void DisableGuidance(KSPActionParam param)
        {
            Log.Info("DisableGuidance: vessel=" + ((vessel != null) ? vessel.name : "null") + " active=" + ((FlightGlobals.ActiveVessel != null) ? FlightGlobals.ActiveVessel.name : "null") + " controller=" + ((controller != null) ? "set" : "null"));
            if (controller != null)
            {
                SaveAeroCalToFile("disable"); // keep the revert-proof file current (f63)
                RemoveController(controller);
                controller.StopLogging();
            }
            if ((vessel) && vessel.enabled) // extra checks
            {
                if (flyRegistered)
                {
                    flyRegistered = false;
                    vessel.OnFlyByWire -= new FlightInputCallback(Fly);
                }
                vessel.Autopilot.Disable();
            }
            // f63: the player's own throttle lever stays where they left it
            // (the pilot-burn passthrough reads it, guidance masks it) - the
            // instant guidance lets go, the stale lever applies. A full lever
            // relaunched the ship right after touchdown, TWICE. Zero the
            // input state so guidance hands back a dead stick
            FlightInputHandler.state.mainThrottle = 0;
            ApplyAirbrakes(false); // never leave the brakes out after guidance lets go
            GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_DisabledGuidance"));
            if (autoWarpEngaged)
            {
                TimeWarp.SetRate(0, false);
                autoWarpEngaged = false;
            }
            if (autoWarpActive)
            {
                TimeWarp.SetRate(0, true); // 自动加速 owned the rate
                autoWarpActive = false;
            }
            controller = null;
            pilotAttitudeMode = 0; // attitude buttons off with guidance (GUI toggle state resets too)
            // Emergency over: restore the real target saved at the button
            if (emergencyLanding)
            {
                emergencyLanding = false;
                SetTarget(savedTgtLat, savedTgtLon, savedTgtAlt);
            }
        }

        /// //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Following two methods come from the following thread:
        // https://forum.kerbalspaceprogram.com/index.php?/topic/130485-problems-getting-relative-pitch-and-yaw-from-vessel-heading/

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ActiveVessel"></param>
        /// <param name="targetVector"></param>
        /// <returns></returns>
        private double[] getOffsetFromHeading(Vessel ActiveVessel, Vector3d targetVector)
        {
            Vector3d yawComponent = Vector3d.Exclude(ActiveVessel.GetTransform().forward, targetVector);
            Vector3d yawCross = Vector3d.Cross(yawComponent, ActiveVessel.GetTransform().right);
            double yaw = SignedVectorAngle(yawComponent, ActiveVessel.GetTransform().up, yawCross);

            Vector3d pitchComponent = Vector3d.Exclude(ActiveVessel.GetTransform().right, targetVector);
            Vector3d pitchCross = Vector3d.Cross(pitchComponent, ActiveVessel.GetTransform().forward);
            double pitch = SignedVectorAngle(pitchComponent, ActiveVessel.GetTransform().up, pitchCross);

            if (Math.Abs(yaw) > 90)
            {
                yaw = -yaw;
                // This condition makes sure progradePitch doesn't wrap from -x to 360-x
                if (pitch > 0)
                {
                    pitch = pitch - 180;
                }
                else
                {
                    pitch = pitch + 180;
                }
            }
            return new double[] { pitch, yaw };
        }

        private double SignedVectorAngle(Vector3d referenceVector, Vector3d otherVector, Vector3d normal)
        {
            Vector3d perpVector;
            double angle;
            //Use the geometry object normal and one of the input vectors to calculate the perpendicular vector
            perpVector = Vector3d.Cross(normal, referenceVector);
            //Now calculate the dot product between the perpendicular vector (perpVector) and the other input vector
            angle = Vector3d.Angle(referenceVector, otherVector);
            angle *= Math.Sign(Vector3d.Dot(perpVector, otherVector));

            return angle;
        }

        /// //////////////////////////////////////////////////////////////////////////////////////////////////////

        // True when guidance was armed while landed (e.g. on the pad before launch):
        // Fly stays hands-off until the vessel is airborne and past its apex, so the
        // touchdown check doesn't instantly disable guidance and we don't fight the
        // ascent autopilot
        private bool waitingForDescent = false;
        private bool flyTickLogged = false;
        private bool pilotBurnActive = false; // [PilotBurn] edge-detect
        private float lastPilotBurnRefuseLog = -100; // Time.time, rate-limits the no-passthrough log
        private bool pilotSwingActive = false; // f85 swing feedback edge-detect
        private bool pilotSwingReady = false; // swing-aligned announcement fired this swing
        // f104 low-alt manual override state (user: "2w米以下的末端调控允许
        // 玩家手动介入调整飞船姿态和油门,当玩家不操作的时候回到自动操作")
        private float prevPilotThr = -1; // lever-move detection for the throttle latch
        private bool lowAltManualThr = false; // throttle latch: manual until the lever returns to zero
        private bool lowAltManualStick = false; // stick override live this tick (the Input.dat recorder logs it as the man column)
        private float lastManualLogT = -100; // Time.time, 1Hz rate limit on the takeover log (f105: the edge flag alone spammed every frame when a double-registered Fly fought another callback over ctrlState)
        private bool lowAltManualStickLoggedPrev = false; // previous-tick stick state (edge detection for the takeover screen message)
        private bool lowAltManualCutLogged = false; // edge logging for the vh<50 forced-auto cut
        private bool flyRegistered = false; // f105: EnableGuidance recreates the controller so !controller.enabled is no guard - track the OnFlyByWire registration explicitly
        // f106: the landing-engine set is enforced ONCE at Flip/LandingBurn
        // entry, not every tick - the per-tick Shutdown() killed the user's
        // manual sea-level engine starts all through the burn ("大气引擎无法
        // 启动"). After the one-shot the user may add engines in an emergency
        private bool landingEngineSetEnforced = false;

        public void Fly(FlightCtrlState state)
        {
            if (vessel == null)
                return;

            if (!flyTickLogged)
            {
                flyTickLogged = true;
                Log.Info("Fly first tick: vessel=" + vessel.name + " active=" + ((FlightGlobals.ActiveVessel != null) ? FlightGlobals.ActiveVessel.name : "null") + " logging=" + Utils.LoggingActive + " phase=" + ((controller != null) ? controller.phase.ToString() : "null"));
            }

            if (waitingForDescent)
            {
                if (vessel.checkLanded())
                    return; // still on the pad
                Vector3d radialUp = Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position);
                if (Vector3d.Dot(vessel.GetObtVelocity(), radialUp) >= 0)
                    return; // still ascending - leave controls to the ascent autopilot
                waitingForDescent = false;
                controller.SetPhase(BLControllerPhase.Unset); // pick the right phase for current altitude
                Log.Info("Pad-armed guidance taking over at apex: alt=" + vessel.altitude);
                GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_EnabledGuidance"));
            }
            double throttle = last_throttle;
            Vector3d steer = last_steer;
            double minThrust;
            double maxThrust;
            float pilotThr = state.mainThrottle; // player's own input; we override below
            // f104: the player's raw stick axes for the <20km manual override
            // at the end of Fly. f105: read FlightInputHandler.state (the raw
            // input source the recorder also logs), NOT state.pitch at Fly
            // entry - by the time Fly runs, state has been through every
            // earlier fly-by-wire callback (SAS PID output included), so the
            // entry value is contaminated exactly when SAS is enabled
            float pilotPitch = FlightInputHandler.state.pitch;
            float pilotYaw = FlightInputHandler.state.yaw;
            float pilotRoll = FlightInputHandler.state.roll;
            controller.pilotThrottle = pilotThr; // live-only pilot-burn attitude (BLController); clones never see it
            controller.pilotAttitudeMode = pilotAttitudeMode; // GUI 顺向/逆向 buttons -> controller, every tick
            controller.starshipPassiveGlide = starshipPassiveGlide; // GUI 被动滑翔 toggle -> controller, every tick
            controller.starshipTrajCal = starshipTrajCal; // GUI Traj校准 toggle -> controller, every tick
            controller.trajCalViolentBrake = starshipViolentBrake; // GUI 暴力减速 toggle -> controller, every tick (f85)
            controller.trajCalAoaMod = starshipAoaMod; // GUI 比例抬头减速 toggle -> controller, every tick (f95)
            controller.trajCalBigTrim = starshipBigTrim; // GUI 高空大偏差反推 toggle -> controller, every tick (f97)
            controller.dvAvailable = dvAvailable; // f96 fuel watch -> controller: the big trim burn has a landing-reserve floor
            controller.landingReserveDv = landingReserveDv;

            KSPUtils.ComputeMinMaxThrust(vessel, out minThrust, out maxThrust);

            Vector3d tgt_r = vessel.mainBody.GetWorldSurfacePosition(tgtLatitude, tgtLongitude, tgtAlt);

            string msg = "";
            bool landingGear = false;
            bool bailOutLandingBurn = true; // cut thrust if near ground and have too much thrust to reach ground
            double elapsedTimeSinceLastUpdate = vessel.missionTime - last_t;
            if ((elapsedTimeSinceLastUpdate > 0.1) || (vessel.altitude < tgtAlt + 500))
            {
                // Attitude from the ReferenceTransform (control point). NOTE: engine
                // thrust transforms were tried here (KSPUtils.GetThrustAxis) but they
                // rotate with the gimbal, injecting gimbal motion into the rate
                // damping; the one-time diagnostic showed ReferenceTransform.up ==
                // GetTransform().up == thrust axis for decoupler-rooted vessels anyway
                Vector3d att = (vessel.ReferenceTransform != null) ? vessel.ReferenceTransform.up : vessel.transform.up;
                if (!attDiagnosticLogged)
                {
                    attDiagnosticLogged = true;
                    Transform vt = vessel.GetTransform();
                    Transform rt = vessel.ReferenceTransform;
                    Vector3d upAxis = FlightGlobals.getUpAxis(vessel.GetWorldPos3D());
                    Log.Info("Attitude axes vs up=" + upAxis + ": thrustAxis=" + KSPUtils.GetThrustAxis(vessel)
                        + " GetTransform.up=" + ((vt != null) ? (Vector3d)vt.up : Vector3d.zero)
                        + " ReferenceTransform.up=" + ((rt != null) ? (Vector3d)rt.up : Vector3d.zero)
                        + " rootTransform.up=" + (Vector3d)vessel.transform.up);
                }
                msg = controller.GetControlOutputs(vessel, vessel.GetTotalMass(), vessel.GetWorldPos3D() - vessel.mainBody.position, vessel.GetObtVelocity(), att, minThrust, maxThrust,
                controller.vessel.missionTime, vessel.mainBody, false, out throttle, out steer, out landingGear, bailOutLandingBurn, debug);
                last_throttle = throttle;
                last_steer = steer;
                last_t = vessel.missionTime;
                var steerAngleOffsets = getOffsetFromHeading(vessel, steer);
                last_steerAngle = Math.Sqrt(steerAngleOffsets[0] * steerAngleOffsets[0] + steerAngleOffsets[1] * steerAngleOffsets[1]);
                ApplyAirbrakes(controller.airbrakeWanted);

                // f96 landing-fuel reserve watch (1 Hz - the propellant scan
                // walks every part): keep dvAvailable fresh for the window,
                // and warn when the remaining dV closes on the landing
                // reserve. Two levels: burning into 1.5x reserve gets a
                // notice; crossing the reserve itself gets the alarm
                if (vessel.missionTime - lastDvCheckT >= 1)
                {
                    lastDvCheckT = vessel.missionTime;
                    double fuelKg = KSPUtils.ComputeUsablePropellantKg(vessel);
                    double mNow = vessel.GetTotalMass();
                    double mDry = mNow - fuelKg / 1000.0; // propellant kg -> t
                    dvAvailable = ((fuelKg > 1) && (mDry > 0) && (mNow > mDry))
                        ? KSPUtils.GetAverageIsp(vessel) * 9.81 * Math.Log(mNow / mDry)
                        : 0;
                    if ((vessel == FlightGlobals.ActiveVessel) && (vessel.missionTime - lastDvWarnT > 10))
                    {
                        if (dvAvailable < landingReserveDv)
                        {
                            lastDvWarnT = vessel.missionTime;
                            GuiUtils.ScreenMessage("DANGER: fuel below the safe-landing reserve! (dV left " + dvAvailable.ToString("F0") + " m/s, landing needs ~" + landingReserveDv.ToString("F0") + " m/s)");
                            Log.Info(string.Format("[FuelWatch] DANGER: dvAvail={0:F0} < reserve={1:F0}", dvAvailable, landingReserveDv));
                        }
                        else if ((dvAvailable < 1.5 * landingReserveDv) && (throttle > 0.01))
                        {
                            lastDvWarnT = vessel.missionTime;
                            GuiUtils.ScreenMessage("WARNING: this burn is eating the landing reserve (dV left " + dvAvailable.ToString("F0") + " m/s, landing needs ~" + landingReserveDv.ToString("F0") + " m/s)");
                            Log.Info(string.Format("[FuelWatch] WARN: burning into the margin: dvAvail={0:F0} reserve={1:F0}", dvAvailable, landingReserveDv));
                        }
                    }
                }
            }

            if ((landingGear) && (!reportedLandingGear))
            {
                KSPUtils.DeployLandingGear(vessel);
                if (vessel == FlightGlobals.ActiveVessel)
                    GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_DeployingLandingGear"));
            }

            if ((msg != "") && (vessel == FlightGlobals.ActiveVessel))
                GuiUtils.ScreenMessage(msg);

            if (vessel.checkLanded())
            {
                DisableGuidance();
                state.mainThrottle = 0;
                return;
            }

            // Set active engines in landing burn. FLIP too (f62 burn-
            // during-flip): the engines must be LIT when the swing-through
            // throttle comes, not ignited after the rotation completes -
            // real Starship lights them at flip start
            // f106: enforce ONCE at phase entry (B) - the per-tick version
            // shut down every engine the user lit by hand all through the
            // burn ("大气引擎无法启动"), and check the selected set can
            // actually stop the ship (A): f106 rode full throttle into the
            // ground on 2 weak engines (TWR<1) while the strong pair sat
            // locked out by the user's own landingBurnEngines=1,1,0,0
            if ((controller.phase == BLControllerPhase.LandingBurn)
                || (controller.phase == BLControllerPhase.Flip))
            {
                if ((controller.landingBurnEngines != null) && (!landingEngineSetEnforced))
                {
                    landingEngineSetEnforced = true;
                    double selectedThrust = 0;
                    foreach (ModuleEngines engine in controller.landingBurnEngines)
                        selectedThrust += engine.maxThrust * engine.thrustPercentage / 100; // kN, rated x limiter
                    double weight = vessel.GetTotalMass() * 9.81; // tonnes -> kN
                    Log.Info(string.Format("[EngineCheck] landing set: {0} engines, rated {1:F0} kN vs weight {2:F0} kN (TWR~{3:F2})",
                        controller.landingBurnEngines.Count, selectedThrust, weight, selectedThrust / Math.Max(1, weight)));
                    if (selectedThrust < 1.15 * weight)
                    {
                        // hopeless even at full throttle - release the whole
                        // set rather than die on the bitmask (f106)
                        controller.landingBurnEngines = null;
                        foreach (ModuleEngines engine in KSPUtils.GetAllEngines(vessel))
                            if (!engine.isOperational)
                                engine.Activate();
                        GuiUtils.ScreenMessage("Selected engines too weak to land (TWR<1.15) - ALL engines released!");
                        Log.Info("[EngineCheck] selected engines too weak - released ALL engines for the landing burn");
                    }
                    else
                    {
                        foreach (ModuleEngines engine in KSPUtils.GetAllEngines(vessel))
                        {
                            if (controller.landingBurnEngines.Contains(engine))
                            {
                                if (!engine.isOperational)
                                    engine.Activate();
                            }
                            else
                            {
                                if (engine.isOperational)
                                    engine.Shutdown();
                            }
                        }
                    }
                }
            }
            else
                landingEngineSetEnforced = false; // re-arm outside the burn phases

            // Draw predicted position if controlling that vessel. In starship
            // AwaitDeorbit the prediction sim is skipped (waiting for the
            // deorbit burn), so predBodyRelPos is stale - the deorbit scope
            // owns the red cross there instead of drawing garbage. Passive
            // glide: no prediction runs at all, so no cross (MJ owns the
            // impact display). TrajCal: Fly's own sim is skipped - the cross
            // shows TRAJECTORIES' impact (what the calibration steers/brakes
            // against), including in passive glide and AwaitDeorbit; hidden
            // while Traj has no data.
            // f97: the passive-glide gate must use the PROFILE-GATED
            // PassiveGlide(), not the raw field - the field defaults true and
            // sits on every core (including falcon boosters, where it means
            // nothing), which suppressed the falcon's own-sim cross entirely
            // (user: 返回加速的时候落点显示消失了 - with BG's cross hidden,
            // the only mark was Trajectories', which itself hides under
            // thrust, so the burn flew with NO impact display)
            if ((vessel == FlightGlobals.ActiveVessel)
                && ((!controller.PassiveGlide()) || (controller.TrajCalActive()))
                && !((recoveryProfile == "starship") && (controller.phase == BLControllerPhase.AwaitDeorbit) && (!controller.TrajCalActive())))
            {
                double lat, lon, alt;
                if (controller.TrajCalActive())
                {
                    TrajAPI.SetAlwaysUpdate(true); // once-guarded inside
                    Vector3d? trajImp = TrajAPI.GetImpactPosition();
                    if (trajImp.HasValue)
                    {
                        vessel.mainBody.GetLatLonAlt(trajImp.Value + vessel.mainBody.position, out lat, out lon, out alt);
                        alt = vessel.mainBody.TerrainAltitude(lat, lon); // Make on surface
                        Targets.RedrawPrediction(vessel.mainBody, lat, lon, alt + 1); // 1m above ground
                    }
                    else if (Targets.predictedCross != null)
                        Targets.predictedCross.enabled = false;
                }
                else
                {
                    // prediction is for position of planet at current time compensating for
                    // planet rotation
                    vessel.mainBody.GetLatLonAlt(controller.predBodyRelPos + controller.vessel.mainBody.position, out lat, out lon, out alt);
                    alt = vessel.mainBody.TerrainAltitude(lat, lon); // Make on surface
                    Targets.RedrawPrediction(vessel.mainBody, lat, lon, alt + 1); // 1m above ground
                }

                Targets.DrawSteer(vessel.vesselSize.x * Vector3d.Normalize(steer), null, Color.green);
            }
            // RCS provides the torque for the turn-around flip - make sure it stays
            // on during that phase even if the operator had it off during ascent
            if ((controller.phase == BLControllerPhase.TurnAround) && (enableRCS))
                vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);

            // Coast autowarp: the post-boostback coast is a long wait with no
            // control input needed; physics-warp through it and drop back to 1x
            // as soon as an active phase begins. Physics warp (never rails)
            // keeps FixedUpdate running so guidance keeps full authority. Only
            // for the active vessel, and only ever raises the rate so a higher
            // manual setting is left alone. Starship EntryCoast gets the same
            // treatment - it is the same long unpowered wait and the belly PD
            // keeps running under physics warp. Flight 49: BellyFlop below the
            // schedule's full-brake q is thin-air max-drag with huge control
            // margins - physics-warp that too (q rising past the threshold
            // drops warp on the next tick via the else branch)
            if ((coastAutoWarp) && (vessel == FlightGlobals.ActiveVessel))
            {
                int rate = Mathf.Clamp(coastAutoWarpRate, 1, 3);
                double qNow = 0.5 * vessel.atmDensity * vessel.srf_velocity.sqrMagnitude;
                bool lowQBelly = (recoveryProfile == "starship") && (controller.phase == BLControllerPhase.BellyFlop) && (qNow < controller.bellyQFullBrake);
                bool coastPhase = (controller.phase == BLControllerPhase.Coasting)
                    || ((recoveryProfile == "starship") && (controller.phase == BLControllerPhase.EntryCoast))
                    || lowQBelly;
                if (coastPhase)
                {
                    if (TimeWarp.CurrentRateIndex < rate)
                    {
                        if (TimeWarp.fetch.Mode != TimeWarp.Modes.LOW) // LOW = physics rates (1-4x)
                            TimeWarp.fetch.Mode = TimeWarp.Modes.LOW;
                        TimeWarp.SetRate(rate, false);
                        if (!autoWarpEngaged)
                            GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_CoastAutoWarp"));
                        autoWarpEngaged = true;
                    }
                }
                else if (autoWarpEngaged)
                {
                    TimeWarp.SetRate(0, false);
                    autoWarpEngaged = false;
                }
            }

            // Terminal RCS translation braking (OPT-IN, rcsTranslateAlt > 0):
            // brakes sideways independently of attitude, but with top-mounted
            // RCS the translation torque can tip the vessel over near the
            // ground - off by default
            if ((enableRCS) && (rcsTranslateAlt > 0) && (controller.phase == BLControllerPhase.LandingBurn) && (vessel.altitude - tgtAlt < rcsTranslateAlt))
            {
                Vector3d upAxis = Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position);
                Vector3d vhVec = Vector3d.Exclude(upAxis, vessel.GetSrfVelocity());
                double horizSpeed = vhVec.magnitude;
                if (horizSpeed > 0.5)
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.RCS, true);
                    double mag = Math.Min(1, horizSpeed / 3);
                    Vector3d local = vessel.ReferenceTransform.InverseTransformDirection(-vhVec * (mag / horizSpeed));
                    state.X = (float)local.x;
                    state.Y = (float)local.y;
                    state.Z = (float)local.z;
                }
            }

            // Pilot throttle passthrough (f61 user request, "开启引导后玩家
            // 仍然可以介入顺向或者逆向启动引擎来校准落点"): in the calm
            // starship phases guidance's own throttle is 0, so hand the
            // player's throttle through for manual impact trimming with the
            // prediction cross as feedback. Guidance still wins whenever it
            // commands thrust (aim burn, overshoot brake, flip, landing),
            // and the 1 Hz prediction re-sim makes the cross honestly
            // reflect the burn's result as the state changes.
            // f61 follow-up: only pass through when the controller accepted
            // the burn (pilotBurnAttitude) and has swung the ship onto
            // +/-vel_air via the SAS path - at the locked belly attitude the
            // thrust is perpendicular to the flight path and a "correction"
            // burn would do nothing along-track
            if ((throttle <= 0) && (pilotThr > 0) && (recoveryProfile == "starship")
                && ((controller.phase == BLControllerPhase.AwaitDeorbit)
                    || (controller.phase == BLControllerPhase.EntryCoast)
                    || (controller.phase == BLControllerPhase.Coasting)
                    || (controller.phase == BLControllerPhase.BellyFlop)))
            {
                if (controller.pilotBurnAttitude)
                {
                    throttle = pilotThr;
                    if (!pilotBurnActive)
                    {
                        pilotBurnActive = true;
                        Log.Info("[PilotBurn] ON thr=" + pilotThr.ToString("F2") + " dir=" + ((controller.pilotAttitudeMode > 0) ? "prograde" : "retro")
                            + " phase=" + controller.phase + " terr=" + controller.targetError.ToString("F0"));
                    }
                }
                else if (Time.time - lastPilotBurnRefuseLog > 5)
                {
                    // refused (no 顺向/逆向 button on, or prograde above the
                    // q guard): log instead of silently eating the player's
                    // throttle input, AND tell them on screen - flight 63
                    // held the throttle for four straight minutes against
                    // this block with no idea why nothing happened.
                    // f85: a pressed-but-refused button logged the same
                    // "press 逆向/顺向 first" line as no-button - the user
                    // pushed the throttle for a minute against an invisible
                    // gate. The controller now reports the real reason
                    lastPilotBurnRefuseLog = Time.time;
                    string why = controller.pilotRefuseWhy;
                    if (why == "")
                        why = "press RETRO/PROGRADE first, then throttle up (prograde may be refused at high q)";
                    Log.Info("[PilotBurn] no passthrough: " + why + " phase=" + controller.phase + " thr=" + pilotThr.ToString("F2"));
                    GuiUtils.ScreenMessage("Throttle locked by guidance: " + why);
                }
            }
            else if (pilotBurnActive)
            {
                pilotBurnActive = false;
                Log.Info("[PilotBurn] OFF phase=" + controller.phase + " terr=" + controller.targetError.ToString("F0"));
            }

            // Flip burn (f62): the flip steer is UP while the nose is still
            // swinging, so the usual 10deg gate would hold the burn until
            // the rotation is DONE - exactly the unpowered fall the burn
            // exists to remove. The controller only outputs the flip
            // throttle below flipBurnMaxAngle, so gate on that instead
            double steerGate = ((controller.phase == BLControllerPhase.Flip) && (throttle > 0))
                ? controller.flipBurnMaxAngle : degreeRange;
            double gateAngle = last_steerAngle;
            // f112 belly-burn gate: belly-attitude burns (tcBurn, bigTrim
            // belly style) HOLD the belly attitude through the burn, where
            // nose-vs-steer is ~160 deg by construction - the nose gate
            // below strangled every belly burn ever commanded to zero
            // throttle (f112: three prograde burns, 47s+29s+29s, all
            // paper-only spentDv, zero fuel burned). Gate on the live belly
            // attitude error instead: 20 deg admits normal tracking
            // (0.3-5 deg, transients ~18) and still blocks a departed
            // attitude. Swing-style burns (pilotBurnAttitude=true slew the
            // NOSE onto +/-vel_air) keep the nose gate.
            if ((throttle > 0) && (!controller.pilotBurnAttitude) && (recoveryProfile == "starship")
                && ((controller.phase == BLControllerPhase.EntryCoast)
                    || (controller.phase == BLControllerPhase.BellyFlop)))
            {
                steerGate = 20;
                gateAngle = controller.bellyAttErrLive;
            }
            if (gateAngle <= steerGate)
                state.mainThrottle = (float)throttle;
            else
                state.mainThrottle = 0;
            bool starshipAttitude = (recoveryProfile == "starship")
                && ((controller.phase == BLControllerPhase.EntryCoast)
                    || (controller.phase == BLControllerPhase.BellyFlop));
            if (starshipAttitude)
            {
                // Back from a pilot burn (or any SAS-flown excursion): the
                // autopilot must not double-command against the belly PD
                if (vessel.Autopilot.Enabled)
                    vessel.Autopilot.Disable();
                // NaN canary: flights 42/43 died to a NaN orbit in EntryCoast
                // with a completely clean Actual.dat. If the poison passes
                // through this pipeline, catch the FIRST non-finite value
                // here with context; if the vessel velocity is already
                // exploding on entry (external kraken), say so instead
                Vector3d vnow = vessel.GetObtVelocity();
                if (((double.IsNaN(vnow.x) || double.IsNaN(vnow.y) || double.IsNaN(vnow.z)) && (!velCanaryFired))
                    || ((vnow.magnitude > 20000) && (!velCanaryFired)))
                {
                    velCanaryFired = true;
                    Log.Info("CANARY: vessel orbital velocity invalid on Fly entry: v=" + vnow + " phase=" + controller.phase
                        + " alt=" + vessel.altitude.ToString("F0") + " warp=" + TimeWarp.CurrentRateIndex + "/" + TimeWarp.fetch.Mode);
                }
                // Custom PD with roll authority (SAS has none) holds the
                // belly to the wind. Flip and LandingBurn use the falcon9
                // SAS path: the flip rotation to nose-UP (local vertical +
                // anti-vh lean) IS the flip, and the ship is upright after
                // it (retargeted from nose-retrograde after flights 51-53).
                // f83: pilot 逆向/顺向 and overshoot-brake swings (steer =
                // +/-vel_air) also fly through the PD, PITCH-ONLY with the
                // wing axis locked level - the SAS path had no roll channel
                // and the shortest-arc swing yawed the hull sideways
                // ("不要出现左右转向来掰机身")
                if (controller.pilotBurnAttitude)
                {
                    // f85 swing progress feedback: at the atmosphere edge the
                    // swing takes ~15 s on an RCS-less hull (wheels + weak
                    // fins vs the rising wind) and the f85 user read the slow
                    // rotation as "没反应", giving up after a 3 s throttle
                    // tap. Announce the swing and the moment it is aligned
                    Transform rtS = vessel.ReferenceTransform;
                    double swingErr = (rtS != null) ? Vector3d.Angle((Vector3d)rtS.up, steer) : 180; // nose vs +/-vel_air target
                    if (!pilotSwingActive)
                    {
                        pilotSwingActive = true;
                        pilotSwingReady = false;
                        Log.Info("[PilotBurn] swing start dir=" + ((controller.swingDir > 0) ? "prograde" : "retro") + " err=" + swingErr.ToString("F0") + " alt=" + vessel.altitude.ToString("F0"));
                        if (vessel == FlightGlobals.ActiveVessel)
                            GuiUtils.ScreenMessage("Swinging to " + ((controller.swingDir > 0) ? "PROGRADE" : "RETRO") + " attitude (may take ~10s) - throttle up when aligned");
                    }
                    else if ((!pilotSwingReady) && (swingErr < 15))
                    {
                        pilotSwingReady = true;
                        Log.Info("[PilotBurn] swing aligned err=" + swingErr.ToString("F1"));
                        if (vessel == FlightGlobals.ActiveVessel)
                            GuiUtils.ScreenMessage("Attitude aligned - throttle up");
                    }
                    starshipAtt.UpdateSwing(vessel, bellyRollOffset, steer,
                        Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position), state);
                }
                else
                {
                    pilotSwingActive = false;
                    // f114: bellyRollPulseDeg is the lateral roll-pulse bump
                    // (0 outside the pulse) - the calibration itself is never
                    // touched
                    starshipAtt.Update(vessel, bellyRollOffset + controller.bellyRollPulseDeg, steer, controller.bellyAoACurrent, state);
                }
                if ((double.IsNaN(state.pitch) || double.IsNaN(state.yaw) || double.IsNaN(state.roll)
                    || double.IsInfinity(state.pitch) || double.IsInfinity(state.yaw) || double.IsInfinity(state.roll)))
                {
                    if (!nanCanaryFired)
                    {
                        nanCanaryFired = true;
                        Log.Info("CANARY: non-finite control output from starship PD: steer=" + steer
                            + " phase=" + controller.phase + " alt=" + vessel.altitude.ToString("F0"));
                    }
                    state.pitch = 0; state.yaw = 0; state.roll = 0; // never feed poison to KSP
                }
                // Attitude tracking failure detector (design D9): warn and
                // keep flying - v1 measures whether the craft can hold it.
                // Skipped during a pilot/overshoot-brake swing: the ship is
                // deliberately leaving the belly frame, so the belly error
                // would false-positive the whole maneuver
                Vector3d bT, nT;
                double bellyErr = 0;
                if (!controller.pilotBurnAttitude)
                {
                    StarshipAttitudeController.BellyFrame(-steer, Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position), controller.bellyAoACurrent, out bT, out nT);
                    // f114: measure against the PULSED reference - a lateral
                    // roll pulse is a commanded attitude, not a tracking
                    // failure (the watchdog would otherwise charge every tap)
                    bellyErr = Vector3d.Angle(StarshipAttitudeController.BellyAxisWorld(vessel, bellyRollOffset + controller.bellyRollPulseDeg), bT);
                }
                if (bellyErr > 20)
                    attFailTime += TimeWarp.fixedDeltaTime;
                else
                {
                    attFailTime = 0;
                    attFailWarned = false;
                }
                if ((attFailTime > 3) && (!attFailWarned))
                {
                    attFailWarned = true;
                    Log.Info("Attitude tracking failure: belly error " + bellyErr.ToString("F1") + " deg for " + attFailTime.ToString("F1") + "s in phase " + controller.phase);
                    if (vessel == FlightGlobals.ActiveVessel)
                        GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_AttitudeTrackingWarn"));
                }
                // Authority diagnostic (flight 45: attitude drifted to a
                // stable 84deg-off trim at high q and the user suspected the
                // elevons never moved). Every 2s log the commanded deflection
                // plus the TOTAL available control torque split by provider
                // type: saturated commands + big surface torque + growing
                // error = disturbance beats authority; ~0 surface torque =
                // the surfaces really are not being driven
                if (Time.time - lastTorqueDiag > 2)
                {
                    lastTorqueDiag = Time.time;
                    double surfT = 0, wheelT = 0, rcsT = 0, gimbalT = 0;
                    foreach (Part p in vessel.parts)
                    {
                        foreach (ITorqueProvider tp in p.FindModulesImplementing<ITorqueProvider>())
                        {
                            Vector3 tpPos, tpNeg;
                            tp.GetPotentialTorque(out tpPos, out tpNeg);
                            double m = tpPos.magnitude + tpNeg.magnitude;
                            if (tp is ModuleControlSurface) surfT += m;
                            else if (tp is ModuleReactionWheel) wheelT += m;
                            else if (tp is ModuleGimbal) gimbalT += m;
                            else rcsT += m;
                        }
                    }
                    Log.Info("TORQUE diag: cmd=(" + state.pitch.ToString("F2") + "," + state.yaw.ToString("F2") + "," + state.roll.ToString("F2")
                        + ") bellyErr=" + bellyErr.ToString("F1") + " surf=" + surfT.ToString("F1") + " wheel=" + wheelT.ToString("F1")
                        + " rcs+other=" + rcsT.ToString("F1") + " gimbal=" + gimbalT.ToString("F1") + " kNm phase=" + controller.phase
                        + " vair=" + vessel.velocityD.magnitude.ToString("F0"));
                }
            }
            else
            {
                // SAS was not enabled at guidance-enable for starship (the
                // belly PD would fight it); enable it lazily now that a
                // SAS-flown phase (Flip/LandingBurn) has been reached.
                // Flight 46: Autopilot.Enabled was apparently true while the
                // SAS action group stayed OFF (user had toggled it off
                // manually), so the Enable() below never fired and the
                // landing burn fell in the aero trim with ZERO attitude
                // control (cs=0 all the way down, crash at -24 m/s).
                // Check the action group too and force it on
                if ((recoveryProfile == "starship") && (!lowAltManualStick)) // f104: while the <20km stick override holds, the autopilot stays OFF - re-enabling here would fight the player (and churn every tick)
                {
                    if ((!vessel.Autopilot.Enabled) || (!vessel.ActionGroups[KSPActionGroup.SAS]))
                    {
                        if (!starshipSasForceLogged)
                        {
                            starshipSasForceLogged = true;
                            Log.Info("Starship " + controller.phase + ": force-enabling SAS (autopilotEnabled=" + vessel.Autopilot.Enabled + " sasGroup=" + vessel.ActionGroups[KSPActionGroup.SAS] + ")");
                        }
                        vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.StabilityAssist);
                        vessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
                    }
                }
                if (!lowAltManualStick) // f104: player on the stick below 20km - SAS stays disabled, do not feed it a target it cannot act on anyway
                {
                    vessel.Autopilot.SAS.lockedMode = false;
                    vessel.Autopilot.SAS.SetTargetOrientation(steer, false);
                }
            }

            // f104 low-altitude manual override (user: "2w米以下的末端调控
            // 允许玩家手动介入调整飞船姿态和油门,当玩家不操作的时候回到自动
            // 操作"). Below 20 km AGL in starship mode:
            //  - stick deflection >5% hands ALL THREE axes to the player's raw
            //    input (the belly PD overwrote them earlier in Fly; the SAS
            //    phases get the same treatment so the authority is
            //    unambiguous) and the autopilot is disengaged while held, so
            //    SAS is not secretly fighting the stick back toward steer.
            //  - MOVING the throttle lever latches the throttle to the player
            //    until the lever returns to zero (a lever is not momentary
            //    like a stick; "not operating" for a lever = zeroed).
            // Guidance keeps simulating underneath either way, so release is
            // an instant hand-back to full auto.
            // f105 cutoff (user: "水平速度低于50后就不要我来接管了因为这个
            // 时候飞船应该进入到翻转点火阶段,这个阶段需要你强行介入,无论我
            // 是否在手动调控"): once the horizontal speed is under 50 m/s the
            // flip/landing sequence OWNS the ship - stick and lever are
            // ignored, latches cleared, autopilot back on, no matter what the
            // player is holding
            if ((recoveryProfile == "starship") && (vessel == FlightGlobals.ActiveVessel))
            {
                double agl = vessel.radarAltitude; // terrain- and ocean-correct
                Vector3d upM = Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position);
                double vhM = Vector3d.Exclude(upM, vessel.srf_velocity).magnitude;
                if ((agl < 20000) && (vhM >= 50))
                {
                    lowAltManualCutLogged = false; // re-arm the cut edge for the next descent
                    bool stick = (Math.Abs(pilotPitch) > 0.05f) || (Math.Abs(pilotYaw) > 0.05f) || (Math.Abs(pilotRoll) > 0.05f);
                    lowAltManualStick = stick;
                    if ((prevPilotThr >= 0) && (Math.Abs(pilotThr - prevPilotThr) > 0.01f))
                        lowAltManualThr = pilotThr > 0.001f; // lever moved: latch on if nonzero, off when zeroed
                    prevPilotThr = pilotThr;
                    if (stick)
                    {
                        state.pitch = pilotPitch;
                        state.yaw = pilotYaw;
                        state.roll = pilotRoll;
                        if (vessel.Autopilot.Enabled)
                            vessel.Autopilot.Disable(); // released: the Flip/LandingBurn block re-enables lazily, PD phases never use it
                        if (Time.time - lastManualLogT > 1)
                        {
                            bool edge = !lowAltManualStickLoggedPrev;
                            lastManualLogT = Time.time;
                            Log.Info("[ManualOverride] stick takeover agl=" + agl.ToString("F0") + " vh=" + vhM.ToString("F0") + " phase=" + controller.phase
                                + " in=(" + pilotPitch.ToString("F2") + "," + pilotYaw.ToString("F2") + "," + pilotRoll.ToString("F2") + ")");
                            if (edge)
                                GuiUtils.ScreenMessage("Manual attitude override ACTIVE (below 20km, vh>=50; center the stick to return to auto)");
                        }
                        lowAltManualStickLoggedPrev = true;
                    }
                    else
                        lowAltManualStickLoggedPrev = false;
                    if (lowAltManualThr)
                        state.mainThrottle = pilotThr; // beats the steer-angle gate above - the player owns the lever in the manual zone
                }
                else
                {
                    prevPilotThr = pilotThr; // keep the baseline warm so entering the zone with a held lever is not a "move"
                    if (((lowAltManualStick) || (lowAltManualThr)) && (vhM < 50) && (!lowAltManualCutLogged))
                    {
                        lowAltManualCutLogged = true;
                        Log.Info("[ManualOverride] CUT: vh=" + vhM.ToString("F0") + " <50 m/s - flip/landing sequence owns the ship now (player input ignored)");
                        GuiUtils.ScreenMessage("vh<50 m/s: flip/landing is forced AUTO - manual input ignored");
                    }
                    lowAltManualThr = false;
                    lowAltManualStick = false;
                    lowAltManualStickLoggedPrev = false;
                }
            }
        }

        public string Info()
        {
            // update if present, otherwise use last message
            // e.g. distance from target at landing
            if (controller != null)
                info = controller.info;
            else if (returnFuelHint != "")
                return returnFuelHint;
            return info;
        }

        // Return-fuel hint ("optimal return point" advisory): while guidance is
        // DISABLED (e.g. the ascent autopilot is flying), a background
        // controller simulates a full return from the current state once per
        // second and Info() shows required vs available propellant. The margin
        // shrinking toward zero as the ascent carries the booster downrange IS
        // the optimal-return-point signal: enable guidance while it is positive
        [KSPField(isPersistant = true, guiActive = false)]
        public bool returnFuelHintEnabled = true;
        private BLController hintController = null;
        private double lastHintTime = -10;
        // Wall-clock adaptive throttle state for the hint/scope sims (see
        // the gate in OnUpdate; flight 58 lag lesson)
        private static readonly System.Diagnostics.Stopwatch hintClock = System.Diagnostics.Stopwatch.StartNew();
        private double hintWallT = -1;
        private double hintWallDur = 0.05;
        private string returnFuelHint = "";

        // f83 cleanup: the deorbit scope fields/method were deleted - the
        // Trajectories red cross is the authoritative impact marker. The
        // panel hint for starship mode is a static line (below)
        private bool starshipRailsEngaged = false;

        // Hotkey state: last parsed source string + KeyCode (re-parse only on
        // change so a typo costs nothing per frame), and the phase tracker
        // for the auto-brakes edge detection above
        private string hkGuidSrc = null;
        private KeyCode hkGuidKC = KeyCode.None;
        private BLControllerPhase prevPhase = BLControllerPhase.Unset;

        private static KeyCode ParseKey(string s, ref string src)
        {
            src = s;
            if (string.IsNullOrEmpty(s))
                return KeyCode.None;
            KeyCode kc;
            try
            {
                kc = (KeyCode)System.Enum.Parse(typeof(KeyCode), s, true);
            }
            catch
            {
                return KeyCode.None;
            }
            return kc;
        }

        // One-key workflow (user request): guidance key = set target to the
        // starred site + enable guidance (press again to disable). No
        // emergency key: it collided with the user's camera tool and fired
        // EmergencyLand mid-descent (f134) - the corner red button stays the
        // only emergency trigger. Active vessel only, and never while the
        // user is typing in a text field (GUIUtility.keyboardControl)
        private void PollHotkeys()
        {
            if ((vessel == null) || (vessel != FlightGlobals.ActiveVessel))
                return;
            if (GUIUtility.keyboardControl != 0)
                return;
            if (hkGuidSrc != hotkeyGuidance)
                hkGuidKC = ParseKey(hotkeyGuidance, ref hkGuidSrc);
            if ((hkGuidKC != KeyCode.None) && Input.GetKeyDown(hkGuidKC))
            {
                if (Enabled())
                    DisableGuidance();
                else
                {
                    // ★ starred site wins; otherwise (user: 我收藏的着陆点有
                    // 很多,选离再入轨迹落点最近的) aim at the site nearest to
                    // Trajectories' predicted impact - different launch
                    // azimuths naturally impact near different sites, so the
                    // corridor picks itself. Falls back to nearest-to-vessel
                    // when Trajectories has no computed trajectory (pad,
                    // pre-launch, mod absent)
                    LandingSite site = null;
                    if (!string.IsNullOrEmpty(hotkeySiteName))
                        site = LandingSites.Find(vessel.mainBody.name, hotkeySiteName);
                    if (site == null)
                    {
                        double aimLat, aimLon;
                        TrajAPI.SetAlwaysUpdate(true); // once-guarded inside
                        Vector3d? imp = TrajAPI.GetImpactPosition();
                        if (imp.HasValue)
                        {
                            double aimAlt;
                            vessel.mainBody.GetLatLonAlt(imp.Value + vessel.mainBody.position, out aimLat, out aimLon, out aimAlt);
                        }
                        else
                        {
                            aimLat = vessel.latitude;
                            aimLon = vessel.longitude;
                        }
                        site = LandingSites.Nearest(vessel.mainBody.name, aimLat, aimLon);
                    }
                    if (site != null)
                    {
                        SetTarget(site.lat, site.lon, site.alt);
                        Targets.RedrawTarget(vessel.mainBody, site.lat, site.lon, site.alt);
                        GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_HotkeyTarget", site.name));
                    }
                    EnableGuidance();
                }
            }
        }

        public override void OnUpdate()
        {
            base.OnUpdate();
            // Auto-brakes (user request 2026-09-12: 刹车=减速板+栅格舵总开关,
            // boost点火完成就直接开启): the moment the phase LEAVES BoostBack
            // the burn is done and the whole deceleration / attitude work
            // should fly with boards+fins out. SetGroup is absolute (never
            // Toggle) so an earlier manual press cannot invert the state. No
            // auto-retract: fins stay out through the landing by design.
            // Falcon-style profiles only - starship manages its own flaps
            if ((controller != null) && (controller.enabled))
            {
                if ((prevPhase == BLControllerPhase.BoostBack) && (controller.phase != BLControllerPhase.BoostBack)
                    && (recoveryProfile != "starship"))
                {
                    vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                    Log.Info("BoostBack complete - brakes ON (airbrakes + grid fins)");
                }
                prevPhase = controller.phase;
            }
            else
                prevPhase = BLControllerPhase.Unset;
            PollHotkeys();
            // Starship warp police (design D10): rails warp stops physics, so
            // Fly never fires during warp - this must live in OnUpdate. Rails
            // warp is cut below the entry interface + 10 km margin and in the
            // BellyFlop; PHYSICS warp stays allowed there (Fly runs every
            // fixed step under physics warp, and test flights are slow enough
            // already). Flip/LandingBurn force 1x in every mode - suicide-burn
            // timing does not tolerate time compression. EntryCoast above the
            // margin warps freely (attitude is frozen by warp and the PD
            // re-engages at 1x)
            if ((controller != null) && (recoveryProfile == "starship") && (vessel == FlightGlobals.ActiveVessel))
            {
                // f104 (user: "当我选择星舰模式之后Tri的落点预测轨迹自动变成
                // 星舰的配置,不然每次都得点很麻烦"): the moment starship mode
                // is active on this vessel, Trajectories flies the
                // StarshipBelly profile + BodyFixedMode - no per-flight GUI
                // clicks. Rate-limited and absent-safe inside
                TrajAPI.SetStarshipProfile();

                bool criticalPhase = (controller.phase == BLControllerPhase.Flip)
                    || (controller.phase == BLControllerPhase.LandingBurn);
                bool slowPhase = (controller.phase == BLControllerPhase.BellyFlop)
                    || (vessel.altitude < vessel.mainBody.atmosphereDepth + 10000);
                bool railsWarp = (TimeWarp.fetch != null) && (TimeWarp.fetch.Mode == TimeWarp.Modes.HIGH);
                bool cut = (TimeWarp.CurrentRateIndex > 0) && (criticalPhase || (slowPhase && railsWarp));
                if (cut)
                {
                    TimeWarp.SetRate(0, true);
                    if (!starshipWarpCutNotified)
                    {
                        starshipWarpCutNotified = true;
                        Log.Info("Starship warp cut: phase=" + controller.phase + " alt=" + vessel.altitude.ToString("F0"));
                        GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_StarshipWarpCut"));
                    }
                }
                else if (TimeWarp.CurrentRateIndex == 0)
                    starshipWarpCutNotified = false;

                // Starship high-coast RAILS autowarp (flight 49: the exo-
                // atmospheric fall to the interface is the longest do-nothing
                // segment and physics warp caps at 4x). Fly does not run under
                // rails warp so this lives in OnUpdate next to the police.
                // Attitude freezing exo-atmospheric is harmless (no torques);
                // the belly PD re-engages when the police cuts warp at the
                // +10km line. Rate scales with the remaining margin so one
                // frame can never jump the whole gap; raise-only like the
                // physics autowarp so a higher manual setting is left alone
                if ((coastAutoWarp) && (TimeWarp.fetch != null) && (controller.phase == BLControllerPhase.EntryCoast))
                {
                    double margin = vessel.altitude - vessel.mainBody.atmosphereDepth;
                    int rails = (margin > 40000) ? 4 : ((margin > 20000) ? 2 : 0); // 100x / 10x
                    if ((rails > 0) && ((TimeWarp.CurrentRateIndex < rails) || (TimeWarp.fetch.Mode != TimeWarp.Modes.HIGH)))
                    {
                        TimeWarp.fetch.Mode = TimeWarp.Modes.HIGH;
                        TimeWarp.SetRate(rails, true);
                        starshipRailsEngaged = true;
                    }
                    else if ((rails == 0) && (starshipRailsEngaged) && (TimeWarp.fetch.Mode == TimeWarp.Modes.HIGH))
                    {
                        TimeWarp.SetRate(0, true); // hand over to Fly's physics autowarp below the rails floor
                        starshipRailsEngaged = false;
                    }
                }
                else if ((starshipRailsEngaged) && (TimeWarp.fetch != null) && (TimeWarp.fetch.Mode == TimeWarp.Modes.HIGH))
                {
                    TimeWarp.SetRate(0, true); // phase left EntryCoast while railed
                    starshipRailsEngaged = false;
                }

                // 自动加速 AutoWarp (user request): while armed, own the
                // warp rate outright - every frame pick the highest SAFE
                // rate and actively DROP when conditions degrade (unlike
                // the raise-only coast autowarp):
                //   authority-demanding phases (Flip/LandingBurn/burns) -> 1x
                //   thin-air BellyFlop (q < bellyQFullBrake) -> physics 4x
                //   exo-atmospheric calm phases -> rails ladder by altitude
                //     margin (10x above atmo+20km, 100x above +40km, 1000x
                //     above +120km), capped so a pending maneuver node stays
                //     at least 45 REAL seconds away
                // The warp police above stays the final backstop
                if ((autoWarp) && (TimeWarp.fetch != null))
                {
                    double qNow = 0.5 * vessel.atmDensity * vessel.srf_velocity.sqrMagnitude;
                    bool calmPhase = (controller.phase == BLControllerPhase.AwaitDeorbit)
                        || (controller.phase == BLControllerPhase.EntryCoast)
                        || (controller.phase == BLControllerPhase.Coasting);
                    bool lowQBelly = (controller.phase == BLControllerPhase.BellyFlop) && (qNow < controller.bellyQFullBrake);
                    double margin = vessel.altitude - vessel.mainBody.atmosphereDepth;
                    int wantIdx = 0;
                    bool wantRails = false;
                    if ((calmPhase) && (margin > 20000))
                    {
                        wantRails = true;
                        wantIdx = (margin > 120000) ? 5 : ((margin > 40000) ? 4 : 2); // 1000x / 100x / 10x
                        // Node cap: never warp to less than 45 real seconds
                        // before the next burn (the deorbit is hand-flown)
                        var solver = vessel.patchedConicSolver;
                        if ((solver != null) && (solver.maneuverNodes != null) && (solver.maneuverNodes.Count > 0))
                        {
                            double lead = solver.maneuverNodes[0].UT - Planetarium.GetUniversalTime();
                            float[] rates = TimeWarp.fetch.warpRates;
                            while ((wantIdx > 0) && ((lead <= 0) || (wantIdx >= rates.Length) || (lead / rates[wantIdx] < 45)))
                                wantIdx--;
                        }
                    }
                    else if ((lowQBelly) || ((calmPhase) && (margin > 0)))
                        wantIdx = 3; // physics 4x
                    if (wantIdx == 0)
                    {
                        if (TimeWarp.CurrentRateIndex > 0)
                        {
                            TimeWarp.SetRate(0, true);
                            if (autoWarpActive)
                                GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_AutoWarpCut"));
                        }
                        autoWarpActive = false;
                    }
                    else
                    {
                        TimeWarp.Modes wantMode = wantRails ? TimeWarp.Modes.HIGH : TimeWarp.Modes.LOW;
                        if (TimeWarp.fetch.Mode != wantMode)
                            TimeWarp.fetch.Mode = wantMode;
                        if (TimeWarp.CurrentRateIndex != wantIdx)
                            TimeWarp.SetRate(wantIdx, true);
                        autoWarpActive = true;
                    }
                }
                else if (autoWarpActive)
                {
                    if ((TimeWarp.fetch != null) && (TimeWarp.CurrentRateIndex > 0))
                        TimeWarp.SetRate(0, true); // disarmed while warping
                    autoWarpActive = false;
                }
            }

            // Manual-flight input recorder: runs with guidance DISABLED (that
            // was the original point - the user flies the reentry by hand),
            // 20 Hz, active vessel only, one writer per vessel. Columns
            // documented in the file header; rate signs match the control
            // channels (positive pitch input should produce positive pitch
            // rate).
            // f104 (user: "我需要你记录我的操作,你要看我到底做什么来校准飞船
            // 然后再学习"): the recorder now ALSO runs in starship mode with
            // guidance ENABLED - the interesting signal is exactly the
            // <20km manual-override windows (man column: 0=auto, 1=stick,
            // 2=throttle latch, 3=both). During an override cs_* == in_* by
            // construction; outside it cs_* is what guidance commanded, so
            // the learning data is "what the player did vs what guidance
            // was doing at the same moment"
            if ((flightRecorderEnabled || (recoveryProfile == "starship")) && (vessel != null) && (vessel == FlightGlobals.ActiveVessel)
                && (GetBoosterGuidanceCore(vessel) == this) && (!vessel.checkLanded()) && (vessel.missionTime > 1))
            {
                if (inputLog == null)
                {
                    string name = vessel.name.Replace(" ", "_").Replace("(", "").Replace(")", "");
                    string dir = KSPUtil.ApplicationRootPath + "Logs/BoosterGuidance/";
                    if (!System.IO.Directory.Exists(dir))
                        System.IO.Directory.CreateDirectory(dir);
                    // f105: APPEND mode - the f105 flight's whole input log
                    // was destroyed when the ship's landed-state flickered
                    // while settling: the gate flicked true/false six times
                    // in 1.3 s and each (re)start TRUNCATED the file, leaving
                    // a lone header. Append + a header line per (re)start
                    // keeps every flight's data; parsers skip '#' lines
                    inputLog = new System.IO.StreamWriter(dir + name + ".Input.dat", true);
                    inputLog.AutoFlush = true; // survive a crash/alt-F4
                    inputLog.WriteLine("# time alt vair vy q aoa bellyerr prate yrate rrate in_p in_y in_r tr_p tr_y tr_r cs_p cs_y cs_r thr sas guid phase mass man");
                    Log.Info("Flight input recorder started: " + name + ".Input.dat");
                }
                if (Time.time - lastInputLog >= 0.05f)
                {
                    lastInputLog = Time.time;
                    Transform rt = vessel.ReferenceTransform;
                    Vector3d srfvel = vessel.srf_velocity;
                    double vair = srfvel.magnitude;
                    double aoa = 0, bellyerr = 0, prate = 0, yrate = 0, rrate = 0;
                    if ((rt != null) && (vair > 0.1))
                    {
                        Vector3d vdir = srfvel / vair;
                        aoa = HGUtils.angle_between(rt.up, vdir);
                        bellyerr = HGUtils.angle_between(StarshipAttitudeController.BellyAxisWorld(vessel, bellyRollOffset), -vdir);
                        Vector3d lw = Quaternion.Inverse(rt.rotation) * vessel.angularVelocity;
                        prate = -lw.x * Mathf.Rad2Deg;
                        yrate = -lw.z * Mathf.Rad2Deg;
                        rrate = -lw.y * Mathf.Rad2Deg;
                    }
                    FlightCtrlState raw = FlightInputHandler.state;
                    FlightCtrlState cs = vessel.ctrlState;
                    double q = 0.5 * vessel.atmDensity * vair * vair;
                    bool sas = vessel.ActionGroups[KSPActionGroup.SAS];
                    string ph = (controller != null) ? controller.phase.ToString() : "manual";
                    int man = (lowAltManualStick ? 1 : 0) + (lowAltManualThr ? 2 : 0);
                    inputLog.WriteLine(string.Format(
                        "{0:F2} {1:F1} {2:F1} {3:F1} {4:F0} {5:F1} {6:F1} {7:F2} {8:F2} {9:F2} {10:F3} {11:F3} {12:F3} {13:F3} {14:F3} {15:F3} {16:F3} {17:F3} {18:F3} {19:F3} {20} {21} {22} {23:F1} {24}",
                        vessel.missionTime, vessel.altitude, vair, vessel.verticalSpeed, q,
                        aoa, bellyerr, prate, yrate, rrate,
                        raw.pitch, raw.yaw, raw.roll, raw.pitchTrim, raw.yawTrim, raw.rollTrim,
                        cs.pitch, cs.yaw, cs.roll, cs.mainThrottle,
                        sas ? 1 : 0, (controller != null) ? 1 : 0, ph, vessel.totalMass, man));
                }
            }
            else if (inputLog != null)
            {
                inputLog.Close();
                inputLog = null;
            }
            // Scope also runs while guidance is ENABLED but parked in
            // AwaitDeorbit: that is exactly when the user is dragging the
            // maneuver node, and hiding the red cross there forced
            // aim-by-maneuver-line (flight 44 feedback). Flight 48: same
            // blind-aiming happened in EntryCoast - an orbit that already
            // dips the atmosphere auto-picks EntryCoast at enable, so the
            // node-drag had no cross there either. Include EntryCoast while
            // still exo-atmospheric (once falling inside the atmosphere the
            // Fly-driven prediction cross owns the display). [SUPERSEDED
            // flight 58: enabled EntryCoast-exo cross ownership moved to
            // Fly's prediction - see starshipScope below]
            // Flight 48 retry: MJ-style always-on - the scope must NOT depend
            // on the fuel-hint toggle, and a fresh vessel with no target set
            // still gets the impact cross (just no deviation readout).
            // Otherwise a brand-new vessel shows absolutely nothing and the
            // user cannot tell why
            // Pull the live calibration back to the core every pass so the
            // deorbit scope (and a fresh enable after a disable) predicts
            // with the SAME realized lift/drag the live flight measured
            if (controller != null)
            {
                aeroCalLift = controller.aeroCalLift;
                aeroCalDrag = controller.aeroCalDrag;
                aeroCalBinsLift = ((controller.aeroCalLiftQ != null) && (controller.aeroCalLiftQ.Length == 3)) ? (double[])controller.aeroCalLiftQ.Clone() : null;
                aeroCalBinsDrag = ((controller.aeroCalDragQ != null) && (controller.aeroCalDragQ.Length == 3)) ? (double[])controller.aeroCalDragQ.Clone() : null;
                aeroCalBinsSamples = ((controller.aeroCalSamplesQ != null) && (controller.aeroCalSamplesQ.Length == 3)) ? (int[])controller.aeroCalSamplesQ.Clone() : null;
                aeroCalBinsLiftAb = ((controller.aeroCalLiftAbQ != null) && (controller.aeroCalLiftAbQ.Length == 3)) ? (double[])controller.aeroCalLiftAbQ.Clone() : null;
                aeroCalBinsSamplesAb = ((controller.aeroCalSamplesAbQ != null) && (controller.aeroCalSamplesAbQ.Length == 3)) ? (int[])controller.aeroCalSamplesAbQ.Clone() : null;
                // f63: keep the revert-proof file fresh mid-flight too (a
                // crash or a revert WITHOUT a clean disable would otherwise
                // lose the whole flight's learning), rate-limited 30 s
                if ((controller.calSamples > 20) && (controller.calSamples != lastAeroCalSavedN)
                    && (Time.time - lastAeroCalFileSave > 30))
                {
                    lastAeroCalFileSave = Time.time;
                    lastAeroCalSavedN = controller.calSamples;
                    SaveAeroCalToFile("periodic");
                }
            }
            bool starship = (recoveryProfile == "starship");
            // f83 cleanup: the deorbit scope (own-sim prediction cross +
            // trajectory line) is DELETED - the Trajectories red cross is
            // the single authoritative impact marker. Starship mode keeps
            // nothing of its own to draw here; BG's PredictionCross now
            // follows Trajectories' impact and is drawn by Fly (TrajCal)
            if (starship)
                return;
            if ((!returnFuelHintEnabled) || (controller != null))
                return; // falcon9: hint honors its toggle and stays off while guided
            if ((vessel == null) || (vessel != FlightGlobals.ActiveVessel))
                return;
            // Only ONE module per vessel runs the estimate: every
            // BoosterGuidanceCore on the vessel passes the checks above, so
            // a multi-core vessel ran the full return sim N times per second
            // and logged N identical ReturnFuelHint lines per tick
            if (GetBoosterGuidanceCore(vessel) != this)
                return;
            if (vessel.checkLanded() || (vessel.missionTime < 1))
                return;
            // (falcon9 return-fuel hint below) Wall-clock adaptive throttle
            // (flight 58): never let these sims own more than ~12% of wall
            // time
            double hw0 = hintClock.Elapsed.TotalSeconds;
            double hintInterval = Math.Max(1.0, hintWallDur / 0.12);
            if ((hintWallT >= 0) && (hw0 - hintWallT < hintInterval))
                return;
            hintWallT = hw0;
            lastHintTime = Time.time;
            if ((tgtLatitude == 0) && (tgtLongitude == 0) && (tgtAlt == 0))
                return; // no target -> no return to estimate
            try
            {
                if (hintController == null)
                {
                    hintController = new BLController(vessel, useFAR);
                    ConfigureController(hintController);
                }
                hintController.phase = BLControllerPhase.Unset; // re-pick the phase for the current state
                hintController.EstimateReturnFuel();
                hintWallDur = Math.Max(0.001, hintClock.Elapsed.TotalSeconds - hw0);
                if (hintController.simLastFlightT >= 599)
                {
                    // The estimate sim timed out without reaching the ground
                    // (e.g. an orbiting payload whose trajectory never lands):
                    // the ballistic miss is a timeout artifact and the hint
                    // would be nonsense - stay silent instead
                    returnFuelHint = "";
                    return;
                }
                double needKg = hintController.simFuelBoostbackKg + hintController.simFuelReentryKg + hintController.simFuelLandingKg;
                double haveKg = KSPUtils.ComputeUsablePropellantKg(vessel);
                // Display as delta-V (user request: tonnes are not intuitive).
                // have-dV: rocket equation on usable propellant; need-dV:
                // boostback dV plus the dV equivalent of the simulated
                // reentry+landing propellant (KSP totalMass is in tonnes)
                double isp = KSPUtils.GetAverageIsp(vessel);
                double massT = vessel.totalMass;
                double bbT = hintController.simFuelBoostbackKg / 1000;
                double reLdT = (hintController.simFuelReentryKg + hintController.simFuelLandingKg) / 1000;
                // The reentry/landing propellant burns AFTER the boostback,
                // so its dV equivalent must be evaluated at the post-
                // boostback mass, not the current mass. At the current mass
                // a big boostback hides most of the landing burn's dV cost
                // (flight 30: 18.5t of landing propellant is 360 m/s at the
                // 174t enable mass but 557 m/s at the 103t it actually
                // burns at - the hint read ~430 m/s optimistic there)
                double massAtLanding = Math.Max(0.001 + reLdT, massT - bbT);
                double dvNeed = hintController.simLastDvBoostback + isp * 9.80665 * Math.Log(massAtLanding / (massAtLanding - reLdT));
                double dvHave = isp * 9.80665 * Math.Log(massT / Math.Max(0.001, massT - haveKg / 1000));
                returnFuelHint = Localizer.Format("#BoosterGuidance_ReturnFuelHint", dvNeed.ToString("F0"), dvHave.ToString("F0"), (dvHave - dvNeed).ToString("+0;-0"));
                Log.Info("ReturnFuelHint: needKg=" + needKg.ToString("F0") + " (bb=" + hintController.simFuelBoostbackKg.ToString("F0") + " re=" + hintController.simFuelReentryKg.ToString("F0") + " ld=" + hintController.simFuelLandingKg.ToString("F0") + ") ballErr=" + hintController.simLastBallisticError.ToString("F0") + " flightT=" + hintController.simLastFlightT.ToString("F0") + " dv=" + hintController.simLastDvBoostback.ToString("F0") + " dvNeed=" + dvNeed.ToString("F0") + " dvHave=" + dvHave.ToString("F0") + " haveKg=" + haveKg.ToString("F0") + " alt=" + vessel.altitude.ToString("F0"));
            }
            catch (Exception e)
            {
                returnFuelHint = "";
                Log.Info("Return fuel hint failed: " + e.Message);
            }
        }

    }
}