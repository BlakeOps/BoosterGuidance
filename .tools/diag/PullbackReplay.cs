// Regression harness for the f141 runaway-aim crash (49.5 km overshoot,
// fuel exhausted, splashed at 216 m/s). Reflection into the REAL compiled
// BoosterGuidance.dll, same pattern as TerminalReplay. NOT part of the mod.
//
// The bug (f141): the f141-batch BoostbackAimError priced the reentry
// pullback as (speed - 660) x targetT, so the aim RECEDED as the burn added
// speed (aim-recede gain ~241 m per m/s ~= mark-approach gain ~250 m per
// m/s -> near-unity net gain, three relight pulses, 407 t burned, entry
// 283 m/s hot). The f142 fix measures the pullback once per second with two
// open-loop passive sims (Simulate.PassiveToGround) and consumes a SMOOTHED,
// CAPPED vector that does not depend on live speed or targetT.
//
// Checks (f141 magnitudes, synthetic orthogonal frame - the law under test
// works on horizontal components, so the frame does not matter):
//   [T1 RUNAWAY] same measured pullback, two entry speeds (1440 vs 1749
//        m/s, f141 boostback start/end) -> IDENTICAL aim. The old law's aim
//        depended on speed (that WAS the bug); the new law must not.
//   [T2 NOMEAS ] no pullback measurement yet -> bare error (under-aim is the
//        safe side: the brake can only pull the impact back - f135/f136).
//   [T3 NODATA ] no live Traj data -> bare error (legacy own-sim behavior).
//   [T4 OFFSET ] with a measurement, returned aim = error - pullback EXACTLY
//        (vector offset, direction included - aero asymmetry is priced).
//   [T5 COPY   ] copy ctor carries the knobs (factor/max) but NOT the live
//        measurement (sim copies must never inherit real-flight state).
//
// Run against the A1F18BC4 (f141) DLL for RED (T1/T2/T4 must FAIL there),
// against the deployed build for GREEN. Usage: PullbackReplay.exe [bgDir] [managed]
using System;
using System.Reflection;

class PullbackReplay
{
    static int failures = 0;
    static bool legacy = false; // old 6-arg signature (f141 build) detected

    static void Check(bool cond, string label)
    {
        Console.WriteLine((cond ? "  PASS " : "  FAIL ") + label);
        if (!cond) failures++;
    }

    static Type ctlType;
    static FieldInfo F(string name)
    {
        return ctlType.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
    }
    static void SetF(object o, string name, object val)
    {
        FieldInfo f = F(name);
        if (f == null) return; // field absent in this build (legacy)
        f.SetValue(o, val);
    }
    static object GetF(object o, string name)
    {
        FieldInfo f = F(name);
        return (f == null) ? null : f.GetValue(o);
    }

    static MethodInfo miAim;   // new 5-arg
    static MethodInfo miAim6;  // old 6-arg (error, vel_air, up, targetT, t, simulate)

    static Vector3d CallAim(object ctl, Vector3d error, Vector3d velAir, Vector3d up, double t, bool simulate)
    {
        if (miAim != null)
            return (Vector3d)miAim.Invoke(ctl, new object[] { error, velAir, up, t, simulate });
        // f141 build: targetT fed the lever. Use 120 s - below cap saturation
        // at both test speeds so the speed-dependence is visible (f141's real
        // 241 s pinned the aim at the 150 km cap, the OTHER half of the bug)
        return (Vector3d)miAim6.Invoke(ctl, new object[] { error, velAir, up, 120.0, t, simulate });
    }

    static Vector3d Sub(Vector3d a, Vector3d b) { return a - b; }
    static bool VecEq(Vector3d a, Vector3d b, double tol)
    {
        return (a - b).magnitude <= tol;
    }

    static int Main(string[] args)
    {
        string bgDir = args.Length > 0 ? args[0] : @"E:\ksp_mod\BoosterGuidance\Source\bin\Release";
        string managed = args.Length > 1 ? args[1] : @"D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program\KSP_x64_Data\Managed";

        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new AssemblyName(e.Name).Name + ".dll";
            foreach (string dir in new[] { bgDir, managed })
            {
                string p = System.IO.Path.Combine(dir, name);
                if (System.IO.File.Exists(p)) return Assembly.LoadFrom(p);
            }
            return null;
        };
        try
        {
            return Run(bgDir);
        }
        catch (Exception ex)
        {
            Console.Out.Flush();
            Console.WriteLine("EXCEPTION: " + ex);
            return 2;
        }
    }

    static object FreshController(bool trajLive)
    {
        object ctl = Activator.CreateInstance(ctlType);
        SetF(ctl, "trajImpactValid", trajLive);
        SetF(ctl, "trajImpactT", 0.0); // age 1 s at t=1, well under trajImpactMaxAge (30 s)
        return ctl;
    }

    static int Run(string bgDir)
    {
        Assembly asm = Assembly.LoadFrom(System.IO.Path.Combine(bgDir, "BoosterGuidance.dll"));
        ctlType = asm.GetType("BoosterGuidance.BLController");
        miAim = ctlType.GetMethod("BoostbackAimError", BindingFlags.Instance | BindingFlags.NonPublic,
            null, new Type[] { typeof(Vector3d), typeof(Vector3d), typeof(Vector3d), typeof(double), typeof(bool) }, null);
        if (miAim == null)
        {
            miAim6 = ctlType.GetMethod("BoostbackAimError", BindingFlags.Instance | BindingFlags.NonPublic,
                null, new Type[] { typeof(Vector3d), typeof(Vector3d), typeof(Vector3d), typeof(double), typeof(double), typeof(bool) }, null);
            legacy = miAim6 != null;
        }
        Console.WriteLine("Build under test: " + bgDir);
        Console.WriteLine("BoostbackAimError signature: " + (miAim != null ? "5-arg (f142 measured pullback)" : legacy ? "6-arg (f141 speed x targetT lever)" : "ABSENT (pre-f140 or post-f143 build)"));
        if ((miAim == null) && (miAim6 == null))
        {
            // f143 方案一 tombstone: the aim-long architecture was DELETED
            // (no future burn -> the passive mark is aimed at the target
            // directly), so the offset law this harness guarded no longer
            // exists. Absence IS the pass condition; if anyone reintroduces
            // an aim offset, the checks below run again
            Console.WriteLine("  PASS TOMBSTONE: aim-offset machinery absent (f143 deleted the aim-long architecture)");
            Console.WriteLine("ALL CHECKS PASSED");
            return 0;
        }

        Vector3d up = new Vector3d(0, 1, 0);
        Vector3d error = new Vector3d(200000, 0, 0);        // impact 200 km long of target (f141 scale)
        Vector3d pull = new Vector3d(80000, 0, 0);          // measured pullback ~80 km (truth for the brick)
        Vector3d vSlow = new Vector3d(1440, -50, 0);        // f141 boostback start speed
        Vector3d vFast = new Vector3d(1749, -50, 0);        // f141 boostback END speed (runaway +283 m/s)

        // T1: runaway regression - speed must not feed the aim
        {
            object a = FreshController(true);
            SetF(a, "trajPullback", pull); SetF(a, "trajPullbackValid", true);
            object b = FreshController(true);
            SetF(b, "trajPullback", pull); SetF(b, "trajPullbackValid", true);
            Vector3d aimSlow = CallAim(a, error, vSlow, up, 1.0, false);
            Vector3d aimFast = CallAim(b, error, vFast, up, 1.0, false);
            Console.WriteLine("[T1] aim@1440=" + aimSlow.x.ToString("F0") + "  aim@1749=" + aimFast.x.ToString("F0"));
            Check(VecEq(aimSlow, aimFast, 1e-6), "T1 RUNAWAY: aim identical at 1440 vs 1749 m/s (speed no longer feeds the pullback)");
        }

        // T2: no measurement -> bare error (under-aim safe side)
        {
            object c = FreshController(true);
            SetF(c, "trajPullbackValid", false);
            Vector3d aim = CallAim(c, error, vSlow, up, 1.0, false);
            Check(VecEq(aim, error, 1e-6), "T2 NOMEAS: no measurement -> bare error (brake can only pull back)");
        }

        // T3: no live Traj data -> bare error (regression guard; also true pre-f142)
        {
            object d = FreshController(false);
            SetF(d, "trajPullback", pull); SetF(d, "trajPullbackValid", true);
            Vector3d aim = CallAim(d, error, vSlow, up, 1.0, false);
            Check(VecEq(aim, error, 1e-6), "T3 NODATA: stale/absent Traj -> bare error");
        }

        // T4: offset applied exactly (vector, not just magnitude)
        {
            object e = FreshController(true);
            Vector3d pull2 = new Vector3d(30000, 0, 5000); // cross-range component must survive
            SetF(e, "trajPullback", pull2); SetF(e, "trajPullbackValid", true);
            Vector3d aim = CallAim(e, error, vSlow, up, 1.0, false);
            Vector3d expect = Sub(error, pull2);
            Check(VecEq(aim, expect, 1e-6), "T4 OFFSET: aim == error - pullback exactly (direction priced)");
        }

        // T5: copy ctor carries knobs, not the live measurement
        {
            if (legacy)
            {
                Console.WriteLine("  SKIP T5 on f141 build (no trajPullback state to verify)");
            }
            else
            {
                object src = FreshController(true);
                SetF(src, "reentryPullbackFactor", 0.7);
                SetF(src, "trajPullback", pull); SetF(src, "trajPullbackValid", true);
                ConstructorInfo ci = ctlType.GetConstructor(new Type[] { ctlType });
                object copy = ci.Invoke(new object[] { src });
                Check(Math.Abs((double)GetF(copy, "reentryPullbackFactor") - 0.7) < 1e-12,
                    "T5 COPY: knob reentryPullbackFactor carried");
                object pv = GetF(copy, "trajPullbackValid");
                Check((pv != null) && ((bool)pv == false),
                    "T5 COPY: live measurement NOT carried into the copy (sim copies stay clean)");
            }
        }

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        return failures == 0 ? 0 : 1;
    }
}
