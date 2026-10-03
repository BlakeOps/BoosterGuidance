// Regression harness for 批次六/七 (fix-terminal-tipover-and-cross-track,
// user-approved 2026-10-02): ③ X′ seeding fix + ① V′-a terminal vh-deadline
// floor + ② Y′ velocity-axis-miss steer demand. Reflection into the REAL
// compiled BoosterGuidance.dll, same pattern as RBExitReplay/ADLowaltReplay.
// NOT part of the mod.
//
// Anchors are the ACTUAL v2 flight states (build 89BA126C snapshots):
//   f278 (超重星舰): LB entry y=8140 vh=230 demand 9.5 delivered ~20 (2-4x
//        over-brake all burn, aMeasH=NaN - the X′ seed never happened);
//        dist FROZE 345-356 the last 25 s = 356 m SHORT; TD tilt 8.0.
//   f279: axisMiss=188-192 m PINNED the whole burn (nobody bent the
//        velocity axis); touchdown 618 = sqrt(190^2+588^2); TD tilt 5.5.
//   f276/f277 (light/200T): terminal chop 0.9->0.13-0.15 at y~31-44,
//        TD tilt 8.0/6.8 still +2.0/+1.0 deg/s (TVC starved).
//
// Checks:
//   [T1 XSEED] VhKillMeasStep: cold seed (lastT=-1) must NOT update (the
//              f274 bug was the CALLER's stash inside this guard - the pure
//              step still guards); a normal second tick warms with the
//              instantaneous slope; dt>5 gap is guarded; EMA blends after
//   [T2 VFLOOR] TermVhFloor anchors: heavy f278 y=30 cap binds at hover;
//              f278 y=187 raw below cap passes through; light f277 y=31
//              lifts to the hover cap (~2.8x the flown 0.14); tiny-tGo clip
//   [T3 NOHOIST] grid sweep: floor NEVER exceeds g/amax (f259 hoist guard)
//              and never goes below minThrottle
//   [T4 AXIS  ] AxisMissComponent: f279-geometry error decomposes to the
//              pinned 190 m perpendicular; parallel error -> ~0; degenerate
//              vh -> error unchanged
//   [T5 FIELDS] new statics + lastTermVhFloorLogT present (stale-DLL guard)
// Usage: TerminalCrossReplay.exe [bgDir] [managed]
using System;
using Reflection = System.Reflection;

class TerminalCrossReplay
{
    static int failures = 0;

    static void Check(bool cond, string label)
    {
        Console.WriteLine((cond ? "  PASS " : "  FAIL ") + label);
        if (!cond) failures++;
    }

    delegate void MeasStep(double prevVh, double lastT, double vhNow, double t,
        double aMeasPrev, bool warmPrev, out double aMeas, out bool warm);

    static int Main(string[] args)
    {
        string bgDir = (args.Length > 0) ? args[0] : ".";
        string managed = (args.Length > 1) ? args[1] : ".";
        string dll = System.IO.Path.Combine(bgDir, "BoosterGuidance.dll");
        Console.WriteLine("TerminalCrossReplay against " + dll);

        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new Reflection.AssemblyName(e.Name).Name + ".dll";
            string p = System.IO.Path.Combine(managed, name);
            return System.IO.File.Exists(p) ? Reflection.Assembly.LoadFrom(p) : null;
        };
        Reflection.Assembly asm = Reflection.Assembly.LoadFrom(dll);
        Type ctl = asm.GetType("BoosterGuidance.BLController");
        // Vector3d lives in Assembly-CSharp, not in the mod DLL (icall family)
        Reflection.Assembly asmCSharp = Reflection.Assembly.LoadFrom(System.IO.Path.Combine(managed, "Assembly-CSharp.dll"));
        Type vec = asmCSharp.GetType("Vector3d");
        if ((ctl == null) || (vec == null)) { Console.WriteLine("  FAIL  types not found"); return 1; }

        Reflection.MethodInfo step = ctl.GetMethod("VhKillMeasStep",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        Reflection.MethodInfo floor = ctl.GetMethod("TermVhFloor",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        Reflection.MethodInfo axis = ctl.GetMethod("AxisMissComponent",
            Reflection.BindingFlags.Static | Reflection.BindingFlags.Public);
        if ((step == null) || (floor == null) || (axis == null))
        {
            Console.WriteLine("  FAIL  batch-6/7 statics missing - stale DLL (pre-batch-6/7)");
            return 1;
        }

        // [T1 XSEED] the f274 cold-start: lastT=-1, t=130 (mission clock) -
        // the guarded step must NOT fabricate a slope from a dead clock.
        object[] io = new object[] { -1.0, -1.0, 230.0, 130.0, 0.0, false, null, null };
        step.Invoke(null, io);
        Check((bool)io[7] == false, "[T1 XSEED] cold clock (lastT=-1, dtK=131) -> no update, stays cold");
        // first real sample stashed, second tick 0.125 s later: instant slope
        // (230->228)/0.125 = 16 m/s2, warms.
        io = new object[] { 230.0, 130.0, 228.0, 130.125, 0.0, false, null, null };
        step.Invoke(null, io);
        Check((bool)io[7] == true && Math.Abs((double)io[6] - 16.0) < 0.01,
              "[T1 XSEED] second tick seeds warm, aMeas=" + ((double)io[6]).ToString("F1") + " (want 16.0)");
        // EMA blend: same slope again stays ~16; a bigger drop pulls it up.
        io = new object[] { 228.0, 130.125, 225.0, 130.25, 16.0, true, null, null };
        step.Invoke(null, io);
        double expect = 16.0 + (24.0 - 16.0) * 0.125;
        Check(Math.Abs((double)io[6] - expect) < 0.01,
              "[T1 XSEED] EMA blend toward 24 -> " + ((double)io[6]).ToString("F2") + " (want " + expect.ToString("F2") + ")");
        // dt>5 gap: guarded, no update, warm preserved.
        io = new object[] { 200.0, 130.0, 150.0, 136.0, 12.0, true, null, null };
        step.Invoke(null, io);
        Check((bool)io[7] == true && (double)io[6] == 12.0,
              "[T1 XSEED] dtK=6 gap -> EMA untouched (caller reseeds the clock)");
        // prevVh=-1 (non-LB reset): no slope even with a good clock.
        io = new object[] { -1.0, 130.0, 228.0, 130.125, 0.0, false, null, null };
        step.Invoke(null, io);
        Check((bool)io[7] == false, "[T1 XSEED] prevVh=-1 (phase reset) -> no slope");

        // [T2 VFLOOR] TermVhFloor(vh, vy, yG, amax, g, minThrottle), g=9.81.
        Func<double, double, double, double, double> F = (vh, vy, y, amax) =>
            (double)floor.Invoke(null, new object[] { vh, vy, y, amax, 9.81, 0.02 });
        // heavy f278 y=30: raw = sqrt(36+961)/(55*2.73) = 0.213 -> cap 0.178 binds.
        double f278y30 = F(6, -11, 30, 55);
        Check(Math.Abs(f278y30 - 9.81 / 55) < 0.005,
              "[T2 VFLOOR] f278 y=30 vh=6 -> " + f278y30.ToString("F3") + " = hover cap (want 0.178)");
        // f278 y=187: raw = sqrt(361+2500)/(55*6.23) = 0.156 < cap -> passes.
        double f278y187 = F(19, -30, 187, 55);
        Check(f278y187 > 0.14 && f278y187 < 0.17,
              "[T2 VFLOOR] f278 y=187 vh=19 -> " + f278y187.ToString("F3") + " raw below cap (want ~0.156)");
        // light f277 y=31: raw = sqrt(16+961)/(25*2.82) = 0.443 -> cap 0.392,
        // ~2.8x the flown 0.14 chop.
        double f277y31 = F(4, -11, 31, 25);
        Check(Math.Abs(f277y31 - 9.81 / 25) < 0.005 && f277y31 > 2.5 * 0.14,
              "[T2 VFLOOR] f277 y=31 vh=4 -> " + f277y31.ToString("F3") + " = cap, ~2.8x the flown chop");
        // tiny tGo (y=2, vy=-2): raw 0.407 -> still capped, never pinned high.
        double f278y2 = F(4, -2, 2, 55);
        Check(Math.Abs(f278y2 - 9.81 / 55) < 0.005,
              "[T2 VFLOOR] f278 y=2 tiny-tGo -> " + f278y2.ToString("F3") + " capped (no end-spike)");

        // [T3 NOHOIST] grid sweep: 0 < floor <= g/amax always (f259 guard),
        // >= minThrottle always.
        bool hoistOk = true;
        for (int am = 10; am <= 70; am += 15)
            for (int yy = 1; yy <= 100; yy += 11)
                for (int vv = -60; vv <= 0; vv += 13)
                    for (int hv = 1; hv <= 30; hv += 7)
                    {
                        double f = F(hv, vv, yy, am);
                        if ((f > 9.81 / am + 1e-9) || (f < 0.02 - 1e-9)) hoistOk = false;
                    }
        Check(hoistOk, "[T3 NOHOIST] 4-deep grid: minThrottle <= floor <= g/amax everywhere");

        // [T4 AXIS] AxisMissComponent(error, vhVec) via reflection on Vector3d.
        Reflection.ConstructorInfo vctor = vec.GetConstructor(new Type[] { typeof(double), typeof(double), typeof(double) });
        Reflection.PropertyInfo vMag = vec.GetProperty("magnitude");
        Reflection.FieldInfo fx = vec.GetField("x"), fz = vec.GetField("z");
        Func<double, double, double, object> V3 = (a, b, c) => vctor.Invoke(new object[] { a, b, c });
        // f279 geometry: vh along +x (100,0,0); error = (-588,0,190): the
        // along-track -588 (the floor's job) must vanish, the pinned 190
        // perpendicular (axisMiss) survives whole.
        object err = V3(-588, 0, 190), vh = V3(100, 0, 0);
        object res = axis.Invoke(null, new object[] { err, vh });
        double rx = (double)fx.GetValue(res), rz = (double)fz.GetValue(res);
        Check(Math.Abs(rx) < 1e-6 && Math.Abs(rz - 190) < 1e-6,
              "[T4 AXIS  ] f279 error(-588,190) @ vh+x -> (" + rx.ToString("F1") + "," + rz.ToString("F1") + ") = pure 190 perpendicular");
        // fully parallel error -> ~0 (nothing for the steer, all the floor's).
        object res2 = axis.Invoke(null, new object[] { V3(-650, 0, 0), vh });
        Check((double)vMag.GetValue(res2) < 1e-6, "[T4 AXIS  ] parallel error -> ~0");
        // degenerate vh -> error unchanged (caller gates vh>5 anyway).
        object res3 = axis.Invoke(null, new object[] { err, V3(0, 0, 0) });
        Check((double)vMag.GetValue(res3) == (double)vMag.GetValue(err), "[T4 AXIS  ] degenerate vh -> error unchanged");

        // [T5 FIELDS] stale-DLL guard.
        Check(ctl.GetField("lastTermVhFloorLogT", Reflection.BindingFlags.Instance | Reflection.BindingFlags.Public | Reflection.BindingFlags.NonPublic) != null,
              "[T5 FIELDS] lastTermVhFloorLogT present");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : ("FAILURES: " + failures));
        return failures == 0 ? 0 : 1;
    }
}
