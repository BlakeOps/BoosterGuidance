// Diagnostic replay harness (flight 25 hover trap) - NOT part of the mod build.
// Replays the logged states of flight 25's terminal descent through the REAL
// compiled BLController.SuicideBurnThrottle (reflection into BoosterGuidance.dll)
// and checks:
//   [TRAP]  at the logged wall rows (y_eff~0) the OLD handoff (taperSwitchY=0)
//           commands accel that BRACKETS g within the vy noise band - a hover
//           equilibrium exists (flight 25: 33s hover at mean cmd = 9.79 = g,
//           then fuel exhaustion and crash). The NEW handoff must admit none.
//   [LAND]  closed-loop 1-DOF vertical sim: the NEW handoff lands gently from
//           the arrested state, at both high and low TWR
//   [FIX]   the NEW handoff commands a fall (min throttle) on the logged
//           parked rows - the taper is reachable
//   [CATCH] on the logged arrest rows just above the new switch the new law
//           brakes into the taper (no bang-bang)
//   [REGR]  above y_log=100m the two laws are identical
//   [SWEEP] no hover fixed point anywhere near y_eff=0 for low/high TWR
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;

class ThrottleReplay
{
    static int failures = 0;

    static void Check(bool cond, string label)
    {
        Console.WriteLine((cond ? "  PASS " : "  FAIL ") + label);
        if (!cond) failures++;
    }

    static int Main(string[] args)
    {
        string logPath = args.Length > 0 ? args[0] : @"E:\ksp_mod\BoosterGuidance\.claude\flight25.dat";
        string bgDir = args.Length > 1 ? args[1] : @"E:\ksp_mod\BoosterGuidance\Source\bin\Release";
        string managed = args.Length > 2 ? args[2] : @"D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program\KSP_x64_Data\Managed";

        var rows = new List<double[]>();
        foreach (string line in File.ReadLines(logPath))
        {
            if (line.StartsWith("#")) continue;
            string[] f = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (f.Length != 17) continue;
            var v = new double[17];
            bool ok = true;
            for (int i = 0; i < 17; i++)
                if (i != 1 && !double.TryParse(f[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v[i])) { ok = false; break; }
            if (!ok) continue;
            if (f[1] == "LandingBurn") rows.Add(v);
        }
        Console.WriteLine("Loaded " + rows.Count + " LandingBurn rows from " + Path.GetFileName(logPath));
        if (rows.Count == 0) { Console.WriteLine("FAIL: no rows"); return 1; }

        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new AssemblyName(e.Name).Name + ".dll";
            foreach (string dir in new[] { bgDir, managed })
            {
                string p = Path.Combine(dir, name);
                if (File.Exists(p)) return Assembly.LoadFrom(p);
            }
            return null;
        };
        try
        {
            return Run(rows);
        }
        catch (Exception ex)
        {
            Console.Out.Flush();
            Console.WriteLine("EXCEPTION: " + ex);
            return 2;
        }
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    static int Run(List<double[]> rows)
    {
        MethodInfo mi = typeof(BoosterGuidance.BLController).GetMethod("SuicideBurnThrottle",
            BindingFlags.NonPublic | BindingFlags.Static);
        if (mi == null) { Console.WriteLine("FAIL: SuicideBurnThrottle not found"); return 1; }

        const double g = 9.8, sf = 0.9, tdSpeed = 2.0, tdMargin = 25.0, dt = 0.02, minThr = 0.01;
        // y_eff = y_log - tdMargin + lowestY. lowestY calibrated so the logged
        // hover rows (y_log=34.90) sit at the observed fixed point y_eff=+0.01
        const double lowestY = -9.89;

        Func<double, double, double, double, double> call = (yLog, vy, amax, switchY) =>
        {
            double yEff = yLog - tdMargin + lowestY;
            double av = Math.Max(0.1, amax - g);
            object[] a = new object[] { yEff, vy, av, g, 0.0, amax, sf, tdSpeed, tdMargin, dt, minThr, switchY, 0.0, 0.0, 0.0 }; // 15th arg = vh (f110 total-speed budget); 0 = vy-only profile, what these near-ground hover rows need
            object r = mi.Invoke(null, a);
            return (double)r;
        };

        int parked = 0, fixFall = 0, regr = 0, catchRows = 0, catchBrake = 0;
        double maxRegrDiff = 0;
        foreach (double[] v in rows)
        {
            // 17 columns with index 1 (phase) skipped: v[3]=y, v[6]=vy, v[13]=amax
            double yLog = v[3], vy = v[6], amax = v[13];
            if (yLog > 34.7 && yLog < 35.1 && vy > -0.4 && vy < 0.15)
            {
                parked++;
                double thrNew = call(yLog, vy, amax, 2.0);
                if (thrNew <= minThr + 1e-9) fixFall++;
            }
            if (yLog > 36.0 && yLog < 36.9 && vy > -20 && vy < -8)
            {
                catchRows++;
                if (call(yLog, vy, amax, 2.0) > 0.2) catchBrake++;
            }
            if (yLog > 100)
            {
                regr++;
                double d = Math.Abs(call(yLog, vy, amax, 0.0) - call(yLog, vy, amax, 2.0));
                if (d > maxRegrDiff) maxRegrDiff = d;
            }
        }

        Console.WriteLine();
        // [TRAP] hover-equilibrium existence at the wall (y_eff ~ 0).
        // Flight data: at the parked rows the OLD law's command BRACKETS g
        // within the vy noise band: vy=-0.1 rows command 14.59 m/s^2 (>g),
        // vy~0 rows 6.39, vy=+0.1 rows 4.15 (<g) - so the closed loop has an
        // equilibrium holding vy in the band at y_eff~0. That equilibrium is
        // the flight-25 hover trap: it held for 33s at mean command = 9.79 = g,
        // airborne the whole time (belly 25m up - it fell 22.8m and crashed
        // when the tanks ran dry). NOTE: a plain 1-DOF sim of the law alone
        // leaks through the wall in <1s; the 33s latch needed real-system
        // detail (engine response, suspension, filtering) that is deliberately
        // NOT modelled here. What this harness locks down is the defect itself:
        // the OLD law admits an equilibrium at y_eff~0, the NEW law admits none.
        double sumOldFall = 0, sumOldRise = 0;
        double sumLogFall = 0, sumLogRise = 0;
        int nFall = 0, nRise = 0, nWall = 0;
        foreach (double[] v in rows)
        {
            double yLog = v[3], vy = v[6], amax = v[13];
            if (yLog < 34.88 || yLog > 34.92) continue; // the wall: y_eff ~ +0.01
            nWall++;
            double daOld = call(yLog, vy, amax, 0.0) * (0.01 + amax); // replayed commanded accel
            double daLog = Math.Sqrt(v[8] * v[8] + v[9] * v[9] + v[10] * v[10]); // logged |a|
            if (vy > -0.15 && vy < -0.05)
            {
                sumOldFall += daOld; sumLogFall += daLog; nFall++;
            }
            else if (vy > -0.05 && vy < 0.15)
            {
                sumOldRise += daOld; sumLogRise += daLog; nRise++;
            }
        }
        double meanOldFall = sumOldFall / Math.Max(1, nFall);
        double meanOldRise = sumOldRise / Math.Max(1, nRise);
        double meanLogFall = sumLogFall / Math.Max(1, nFall);
        double meanLogRise = sumLogRise / Math.Max(1, nRise);
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "[TRAP]  wall rows (y_eff~0): vy~-0.1 -> logged cmd {0:F2} / replay {1:F2} ({2} rows); vy~0,+0.1 -> logged {3:F2} / replay {4:F2} ({5} rows); g=9.8",
            meanLogFall, meanOldFall, nFall, meanLogRise, meanOldRise, nRise));
        Check(nFall > 50 && nRise > 50, "replay covered the logged wall states");
        Check(meanLogFall > g + 1.0 && meanLogRise < g - 1.0,
            "bug reproduces in the flight data: commands bracket g within the vy noise band -> hover equilibrium at y_eff~0 (flight 25)");
        Check(meanOldFall > meanOldRise + 2.0,
            "OLD law replay reproduces the bracket ordering (hot when falling, cool when rising)");
        // Closed-loop check that the NEW law gets DOWN from the same arrested
        // state (this one the 1-DOF sim CAN show): integrate (y_eff, vy)
        // through the real law, thrust instant, 20ms tick, from the state
        // flight 25 arrested at. Touchdown = belly at the ground (y_eff=-tdMargin).
        Func<double, double, double, double, double> simToEnd = (y0, vy0, amax, switchY) =>
        {
            double y = y0, vyl = vy0, dtSim = 0.02;
            double av = Math.Max(0.1, amax - g);
            for (int i = 0; i < (int)(30.0 / dtSim); i++)
            {
                object[] a = new object[] { y, vyl, av, g, 0.0, amax, sf, tdSpeed, tdMargin, dtSim, minThr, switchY, 0.0, 0.0, 0.0 }; // 15th = vh (f110); 0 = vertical-only sim
                double thr = (double)mi.Invoke(null, a);
                double acc = thr * (0.01 + amax) - g; // amin = 0
                vyl += acc * dtSim;
                y += vyl * dtSim;
                if (y <= -tdMargin) return vyl; // touchdown: return impact speed
            }
            return double.NaN; // never landed within 30s
        };
        double vyNew = simToEnd(0.05, -0.5, 66.5, 2.0);
        double vyNewLo = simToEnd(0.05, -0.5, 35.0, 2.0); // 4-engine TWR class (flight 26)
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "[LAND]  NEW law closed-loop from arrested state (y_eff=0.05, vy=-0.5):" +
            " touchdown vy={0} (amax=66.5), vy={1} (amax=35)",
            double.IsNaN(vyNew) ? "none" : vyNew.ToString("F2"),
            double.IsNaN(vyNewLo) ? "none" : vyNewLo.ToString("F2")));
        Check(!double.IsNaN(vyNew) && Math.Abs(vyNew) < 6, "FIX: NEW law lands gently from the arrested state (high TWR)");
        Check(!double.IsNaN(vyNewLo) && Math.Abs(vyNewLo) < 6, "FIX: NEW law lands gently from the arrested state (low TWR)");
        Console.WriteLine("[FIX]   logged parked rows: " + parked + "; NEW law at min throttle: " + fixFall + "/" + parked);
        Check(fixFall > 0.9 * parked, "FIX: NEW law commands the fall (taper reachable) on parked rows");
        Console.WriteLine("[CATCH] arrest rows just above switch: " + catchRows + "; NEW law braking: " + catchBrake);
        Check(catchRows == 0 || catchBrake > 0.9 * catchRows, "FIX: NEW law brakes into the taper at the switch (catch, not dive)");
        Console.WriteLine("[REGR]  rows above 100m: " + regr + "; max OLD-vs-NEW diff: " + maxRegrDiff.ToString("E2"));
        Check(regr > 0 && maxRegrDiff < 1e-12, "REGR: above the switch the law is unchanged");

        // No fixed point near y_eff=0 for either TWR class (f25: amax~70,
        // f26 four engines: amax~35): at the old trap state the new law must
        // command a fall, for both engine fits
        double thrHiTwr = call(34.9, -0.1, 70.0, 2.0);
        double thrLoTwr = call(34.9, -0.1, 35.0, 2.0);
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "[SWEEP] NEW law at the old trap state: thr={0:F3} (amax=70), {1:F3} (amax=35)", thrHiTwr, thrLoTwr));
        Check(thrHiTwr <= minThr + 1e-9 && thrLoTwr <= minThr + 1e-9, "FIX: no hover fixed point at y_eff~0 for high or low TWR");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        return failures == 0 ? 0 : 1;
    }
}
