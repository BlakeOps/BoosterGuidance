// Regression harness for the batch-2 思路1 ceiling (user-approved 2026-10-01):
// the ReentryBurn v* release must be gated on the sim-captured LandingBurn
// handover being stoppable. Anchors are the ACTUAL flight states:
//   f266 (1808m LONG): handover 825 m/s over 3156 m room, aLatLB~12 -> ratio 9.0
//   f268 (4800m LONG): handover 792 m/s over  494 m room, aLatLB~12 -> ratio 52.9
//   f269 ( 171m OK  ): handover 344 m/s over 3489 m room, aLatLB~12 -> ratio 1.42
// Reflection into the REAL compiled BoosterGuidance.dll, same pattern as
// PullbackReplay. NOT part of the mod.
//
// Checks:
//   [T1 f268 ] 792@494  must be INFEASIBLE (this exit parked at v*=849, markHit=0)
//   [T2 f266 ] 825@3156 must be INFEASIBLE
//   [T3 f269 ] 344@3489 must be FEASIBLE (the proven-working handover)
//   [T4 NOWIT] no capture (lbVh<=0) must be INFEASIBLE - no witness, no release
//   [T5 REMFL] rem is floored at 200 m: 100 m/s over 0 m -> infeasible (2.08),
//              90 m/s over 0 m -> feasible (1.69)
//   [T6 FIELDS] the new harvest/gate fields exist (stale-DLL guard)
// Usage: RBExitReplay.exe [bgDir] [managed]
using System;
using System.Reflection;

class RBExitReplay
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
        Console.WriteLine("RBExitReplay against " + dll);

        // Dependencies resolve from the game's Managed dir
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new AssemblyName(e.Name).Name + ".dll";
            string p = System.IO.Path.Combine(managed, name);
            return System.IO.File.Exists(p) ? Assembly.LoadFrom(p) : null;
        };
        Assembly asm = Assembly.LoadFrom(dll);
        Type ctl = asm.GetType("BoosterGuidance.BLController");
        if (ctl == null) { Console.WriteLine("  FAIL  BLController type not found"); return 1; }

        MethodInfo feas = ctl.GetMethod("ReentryHandoverFeasible",
            BindingFlags.Static | BindingFlags.Public);
        if (feas == null)
        {
            Console.WriteLine("  FAIL  ReentryHandoverFeasible missing - stale DLL (pre-batch-2)");
            return 1;
        }
        Func<double, double, double, bool> F = (vh, rem, aLat) =>
            (bool)feas.Invoke(null, new object[] { vh, rem, aLat });

        double aLatLB = 12.0; // 69 m/s2 amax x sin(10deg landingBurnMaxAoA) - the 超重星舰 knob as flown

        Check(!F(792, 494, aLatLB),  "[T1 f268 ] 792 m/s @ 494 m  -> infeasible (ratio 52.9)");
        Check(!F(825, 3156, aLatLB), "[T2 f266 ] 825 m/s @ 3156 m -> infeasible (ratio 9.0)");
        Check(F(344, 3489, aLatLB),  "[T3 f269 ] 344 m/s @ 3489 m -> feasible (ratio 1.42, the 171m flight)");
        Check(!F(-1, -1, aLatLB) && !F(0, 500, aLatLB),
                                     "[T4 NOWIT] no capture -> infeasible (v* cannot release)");
        Check(!F(100, 0, aLatLB) && F(90, 0, aLatLB),
                                     "[T5 REMFL] rem floored at 200 m (2.08 blocked / 1.69 allowed)");

        // batch-2 harvest fields (pre-batch-12; simLBDelivA/rbLbDelivA added by batch-12)
        string[] fields = { "simLBEntryVh", "simLBEntryRem", "simLBEntryT", "rbLbVh", "rbLbRem", "rbLbT", "rbFeasStreak" };
        foreach (string name in fields)
            Check(ctl.GetField(name, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic) != null,
                  "[T6 FIELDS] " + name + " present");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : ("FAILURES: " + failures));
        return failures == 0 ? 0 : 1;
    }
}
