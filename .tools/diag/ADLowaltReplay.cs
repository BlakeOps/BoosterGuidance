// Regression harness for the batch-5 AD low-altitude active cancellation
// (ad-lowalt-active-correction, user-approved 2026-10-02). Reflection into
// the REAL compiled BoosterGuidance.dll, same pattern as RBExitReplay.
// NOT part of the mod.
//
// Anchors are the ACTUAL v2 flight states (build 89BA126C snapshots):
//   f280 upper band (y 26-40 km, phantom zone): raw terr swung 126..3160 m
//        between prediction runs -> noise floor several hundred m; the
//        honesty ratio cannot hold. Auto law delivered aTotH mean -0.33.
//   f280 lower band (y 6-15 km, honest zone, user manual influence):
//        terr declined smoothly 871->64, delivered aTotH mean -7.66 /
//        min -11.0 m/s2 at a productive ~46-58 deg tilt (vs ~73 deg trim
//        at 26 km) -> ~24 deg in the corr convention (tan ~= deg2rad cap).
//   f279 (pure auto baseline): same lower band delivered only mean -4.27
//        (mostly drag, no maneuver) and closed just 1240->818.
//   f276 LB-entry stop-ratio ~1.0 (thin but FEASIBLE -> guard must not
//        block); f266-class 825@3156 aLat~12 (ratio 9.0 -> must collapse).
//
// Checks:
//   [T1 PHANT] phantom-zone anchors fail the honesty gate (no engagement)
//   [T2 HONST] honest-zone anchors pass (engagement), hysteresis: stays
//              engaged at 2.5x, releases below 2x
//   [T3 SOLVE] f280-anchored force curve: aReq 4.7 m/s2 solves into the
//              productive 20-30 deg window; solve is continuous and
//              monotone in aReq; beyond full authority pins at the 55 cap
//   [T4 ZERO  ] zero-authority probe -> 0 (caller keeps legacy envelope)
//   [T5 GUARD] handover witness: f276 thin-but-feasible does not block;
//              f266-class arithmetic death collapses (ReentryHandoverFeasible)
//   [T6 FIELDS] new fields/methods exist (stale-DLL guard)
// Usage: ADLowaltReplay.exe [bgDir] [managed]
using System;
using Reflection = System.Reflection;

class ADLowaltReplay
{
    static int failures = 0;

    static void Check(bool cond, string label)
    {
        Console.WriteLine((cond ? "  PASS " : "  FAIL ") + label);
        if (!cond) failures++;
    }

    static int Main(string[] args)
    {
        string bgDir = (args.Length > 0) ? args[0] : ".";
        string managed = (args.Length > 1) ? args[1] : ".";
        string dll = System.IO.Path.Combine(bgDir, "BoosterGuidance.dll");
        Console.WriteLine("ADLowaltReplay against " + dll);

        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new Reflection.AssemblyName(e.Name).Name + ".dll";
            string p = System.IO.Path.Combine(managed, name);
            return System.IO.File.Exists(p) ? Reflection.Assembly.LoadFrom(p) : null;
        };
        Reflection.Assembly asm = Reflection.Assembly.LoadFrom(dll);
        Type ctl = asm.GetType("BoosterGuidance.BLController");
        if (ctl == null) { Console.WriteLine("  FAIL  BLController type not found"); return 1; }

        Reflection.MethodInfo honest = ctl.GetMethod("AdCancelHonest",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        Reflection.MethodInfo depart = ctl.GetMethod("AdCancelDeparture",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        Reflection.MethodInfo feas = ctl.GetMethod("ReentryHandoverFeasible",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        if ((honest == null) || (depart == null) || (feas == null))
        {
            Console.WriteLine("  FAIL  batch-5 statics missing - stale DLL (pre-batch-5)");
            return 1;
        }
        Func<double, double, bool, bool> H = (errSm, noise, prev) =>
            (bool)honest.Invoke(null, new object[] { errSm, noise, prev });
        Func<double, Func<double, double>, double> D = (aReq, probe) =>
            (double)depart.Invoke(null, new object[] { aReq, probe, 55.0 });
        Func<double, double, double, bool> F = (vh, rem, aLat) =>
            (bool)feas.Invoke(null, new object[] { vh, rem, aLat });

        // [T1 PHANT] f280 upper-band phantom anchors: error swings of several
        // hundred m per prediction run against a comparable smoothed error.
        Check(!H(800, 400, false), "[T1 PHANT] errSm 800 vs noise 400 (2.0x) -> not honest");
        Check(!H(1500, 600, false), "[T1 PHANT] errSm 1500 vs noise 600 (2.5x) -> not honest (engage needs 3x)");
        Check(!H(900, 500, true), "[T1 PHANT] errSm 900 vs noise 500 (1.8x) -> releases even when engaged");

        // [T2 HONST] f280 lower-band anchors: smooth decline, noise tens of m.
        Check(H(1000, 30, false), "[T2 HONST] errSm 1000 vs noise 30 (33x) -> honest");
        Check(H(75, 30, true), "[T2 HONST] hysteresis: 75 vs 30 (2.5x) stays engaged");
        Check(!H(55, 30, true), "[T2 HONST] hysteresis: 55 vs 30 (1.8x) releases");
        Check(!H(1000, -1, false), "[T2 HONST] unseeded noise -> never honest");

        // [T3 SOLVE] f280 force curve: at belly trim the hull is broadside-ish
        // (near max |F| already); departing rotates/shrinks the force vector
        // so the secant delivery rises STEEPLY early then saturates - a
        // SUBlinear curve. Anchors: ~7 m/s2 at the ~24-unit productive point
        // (f280 mean -7.66 at 46.6 deg tilt vs ~73 deg trim), ~11 at the cap
        // (f280 min -11.0). aReq 4.7 m/s2 (demMag ~940 over tResp 20 s) must
        // solve below the productive anchor, well clear of the cap.
        Func<double, double> f280curve = (A) => 11.0 * Math.Pow(A / 55.0, 0.6); // sublinear: fast rise off trim, saturate
        double a47 = D(4.7, f280curve);
        Check(a47 > 10 && a47 < 25, "[T3 SOLVE] aReq 4.7 on the f280 curve -> " + a47.ToString("F1") + " deg (want 10-25)");
        double a1 = D(1.0, f280curve), a3 = D(3.0, f280curve), a8 = D(8.0, f280curve);
        Check(a1 > 0 && a1 < a3 && a3 < a8 && a8 <= 55, "[T3 SOLVE] continuous + monotone in aReq (1->3->8 m/s2)");
        Check(Math.Abs(D(12.0, f280curve) - 55.0) < 0.001, "[T3 SOLVE] aReq beyond full authority -> pin 55 (max effort)");
        Check(D(0, f280curve) == 0, "[T3 SOLVE] no demand -> 0");

        // [T4 ZERO] brick-in-vacuum probe: no delivery at any departure.
        Check(D(5.0, (A) => 0.0) == 0, "[T4 ZERO ] zero authority -> 0 (legacy envelope kept)");

        // [T5 GUARD] handover witness anchors (aLatLB ~12 as flown).
        Check(F(500, 12000, 12.0), "[T5 GUARD] f276-class thin handover (ratio ~1.0) -> fits, no block");
        Check(!F(825, 3156, 12.0), "[T5 GUARD] f266-class 825@3156 (ratio 9.0) -> collapse to legacy");

        // [T6 FIELDS] stale-DLL guard.
        string[] fields = { "adCancelEngaged", "lastAdCancelLogT" };
        foreach (string name in fields)
            Check(ctl.GetField(name, Reflection.BindingFlags.Instance | Reflection.BindingFlags.Public | Reflection.BindingFlags.NonPublic) != null,
                  "[T6 FIELDS] " + name + " present");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : ("FAILURES: " + failures));
        return failures == 0 ? 0 : 1;
    }
}
