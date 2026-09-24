// Diagnostic replay harness (flight 22 tip-over) - NOT part of the mod build.
// Replays the terminal rows of a flight log through the REAL compiled
// BLController.VelocityToGoCorrection (reflection into BoosterGuidance.dll)
// and checks:
//   [REPRO]  the old law (v2gBrakeOnlyRadius=0) commands toward-target
//            acceleration while nearly on top of the target (through-zero
//            pumping, flight 22)
//   [FIX]    the new law (v2gBrakeOnlyRadius=15) is strictly brake-only
//            inside the radius (correction anti-parallel to vh)
//   [REGR]   outside the radius the correction is unchanged
//   [TILT]   uncapped steer (retro-lean + old correction) exceeds 25 deg
//            somewhere in the last 100m, and the lowTiltCap math bounds
//            every capped command to <= 15 deg
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;

class TerminalReplay
{
    static int failures = 0;

    static void Check(bool cond, string label)
    {
        Console.WriteLine((cond ? "  PASS " : "  FAIL ") + label);
        if (!cond) failures++;
    }

    static Vector3d CallV2G(object ctl, MethodInfo mi, Vector3d posErr, Vector3d vel, Vector3d up, double y, double vy, double maxAoA)
    {
        // f212: VelocityToGoCorrection gained 4 optional params (markShortHold,
        // t, body, tgtR) - reflection Invoke needs the full count; the replay
        // exercises the plain law so the mark-hold guard stays off
        return (Vector3d)mi.Invoke(ctl, new object[] { posErr, vel, up, y, vy, maxAoA, new Vector3d(0, 0, 0), false, 0.0, null, null });
    }

    static int Main(string[] args)
    {
        string logPath = args.Length > 0 ? args[0] : @"E:\ksp_mod\BoosterGuidance\.claude\flight17.dat";
        string bgDir = args.Length > 1 ? args[1] : @"E:\ksp_mod\BoosterGuidance\Source\bin\Release";
        string managed = args.Length > 2 ? args[2] : @"D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program\KSP_x64_Data\Managed";

        // Parse terminal LandingBurn rows (y < 120m) - no KSP types here, so
        // this stays in Main above the resolver registration boundary
        var rows = new List<double[]>();
        foreach (string line in File.ReadLines(logPath))
        {
            if (line.StartsWith("#")) continue;
            string[] f = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
            if (f.Length != 17) continue;
            if (f[1] != "LandingBurn") continue;
            var v = new double[17];
            bool ok = true;
            for (int i = 0; i < 17; i++)
                if (i != 1 && !double.TryParse(f[i], NumberStyles.Float, CultureInfo.InvariantCulture, out v[i])) { ok = false; break; }
            if (!ok) continue;
            if (v[3] < 120) rows.Add(v); // y < 120m
        }
        Console.WriteLine("Replayed " + rows.Count + " terminal rows from " + Path.GetFileName(logPath));
        if (rows.Count == 0) { Console.WriteLine("FAIL: no rows"); return 1; }

        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new AssemblyName(e.Name).Name + ".dll";
            // f212: + the SpaceTuxLibrary Plugins dir - VelocityToGoCorrection's
            // mark-hold guard calls Log.Info (KSP_Log.dll ships with
            // GameData\SpaceTuxLibrary\Plugins, not Managed)
            foreach (string dir in new[] { bgDir, managed, Path.GetFullPath(Path.Combine(managed, @"..\..\GameData\SpaceTuxLibrary\Plugins")) })
            {
                string p = Path.Combine(dir, name);
                if (File.Exists(p)) return Assembly.LoadFrom(p);
            }
            return null;
        };
        try
        {
            return Run(logPath, rows);
        }
        catch (Exception ex)
        {
            Console.Out.Flush();
            Console.WriteLine("EXCEPTION: " + ex);
            return 2;
        }

    }

    // Managed replacements: KSP's Vector3d.Angle/Slerp are InternalCalls into
    // the Unity native module, which only exists inside the game process
    static double AngleDeg(Vector3d a, Vector3d b)
    {
        double d = Vector3d.Dot(Vector3d.Normalize(a), Vector3d.Normalize(b));
        if (d > 1) d = 1; if (d < -1) d = -1;
        return Math.Acos(d) * 180.0 / Math.PI;
    }

    static Vector3d SlerpManaged(Vector3d a, Vector3d b, double t)
    {
        double theta = AngleDeg(a, b) * Math.PI / 180.0;
        if (theta < 1e-9) return a;
        Vector3d an = Vector3d.Normalize(a);
        Vector3d rel = Vector3d.Normalize(Vector3d.Normalize(b) - an * Math.Cos(theta));
        return an * Math.Cos(t * theta) + rel * Math.Sin(t * theta);
    }

    // BoosterGuidance-typed code lives here so Main can register the
    // AssemblyResolve handler before this method is JIT-compiled
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    static int Run(string logPath, List<double[]> rows)
    {
        // Real controller instances from the built DLL
        var ctlOld = new BoosterGuidance.BLController(); // pre-fix behaviour
        ctlOld.v2gBrakeOnlyRadius = 0;
        ctlOld.steerDamping = 0;
        var ctlNew = new BoosterGuidance.BLController(); // deployed behaviour
        ctlNew.v2gBrakeOnlyRadius = 15;
        ctlNew.steerDamping = 0;
        MethodInfo miV2G = typeof(BoosterGuidance.BLController).GetMethod("VelocityToGoCorrection",
            BindingFlags.NonPublic | BindingFlags.Instance);
        if (miV2G == null) { Console.WriteLine("FAIL: VelocityToGoCorrection not found"); return 1; }

        Vector3d up = new Vector3d(0, 1, 0);
        const double lowTiltCap = 15.0, lowTiltCapHeight = 100.0, v2gMaxAoA = 12.0;

        int reproRows = 0, insideRows = 0, outsideRows = 0;
        double maxTiltUncapped = 0, maxTiltCapped = 0, maxOutsideDiff = 0, worstBrakeCross = 0;
        string reproSample = "";

        foreach (double[] v in rows)
        {
            double x = v[2], y = v[3], z = v[4], vx = v[5], vy = v[6], vz = v[7];
            var posErr = new Vector3d(x, 0, z);
            var vel = new Vector3d(vx, vy, vz);
            var vh = Vector3d.Exclude(up, vel);

            Vector3d corrOld = CallV2G(ctlOld, miV2G, posErr, vel, up, y, vy, v2gMaxAoA);
            Vector3d corrNew = CallV2G(ctlNew, miV2G, posErr, vel, up, y, vy, v2gMaxAoA);

            if (posErr.magnitude < 15)
            {
                insideRows++;
                // [REPRO] old law pushes toward the target (through-zero pumping)
                if (posErr.magnitude > 0.5 && corrOld.magnitude > 0.01)
                {
                    double toward = Vector3d.Dot(Vector3d.Normalize(corrOld), -Vector3d.Normalize(posErr));
                    if (toward > 0.1)
                    {
                        reproRows++;
                        if (reproSample == "")
                            reproSample = string.Format(CultureInfo.InvariantCulture,
                                "t={0:F1} y={1:F1} off={2:F2} vh={3:F2} toward={4:F2}", v[0], y, posErr.magnitude, vh.magnitude, toward);
                    }
                }
                // [FIX] new law strictly anti-parallel to vh (pure braking)
                if (vh.magnitude > 0.5 && corrNew.magnitude > 1e-6)
                {
                    double cross = Vector3d.Cross(corrNew, vh).magnitude / (corrNew.magnitude * vh.magnitude);
                    double dot = Vector3d.Dot(corrNew, vh);
                    if (cross > worstBrakeCross) worstBrakeCross = cross;
                    if (dot >= 0) { Check(false, "brake-only corr has positive dot with vh at y=" + y); }
                }
            }
            else
            {
                outsideRows++;
                // [REGR] outside the radius: identical correction
                double diff = (corrNew - corrOld).magnitude;
                if (diff > maxOutsideDiff) maxOutsideDiff = diff;
            }

            // [TILT] full uncapped command (retro-lean + old corr), then the cap math
            Vector3d steerBase = -Vector3d.Normalize(vel - 20 * up);
            Vector3d steerUnc = steerBase + corrOld;
            double tiltUnc = AngleDeg(steerUnc, up);
            if (tiltUnc > maxTiltUncapped) maxTiltUncapped = tiltUnc;
            if (y < lowTiltCapHeight)
            {
                double tilt = tiltUnc;
                Vector3d steerCap = steerUnc;
                if (tilt > lowTiltCap)
                    steerCap = SlerpManaged(steerUnc, up, 1 - lowTiltCap / tilt);
                double tc = AngleDeg(steerCap, up);
                if (tc > maxTiltCapped) maxTiltCapped = tc;
            }
        }

        Console.WriteLine();
        Console.WriteLine("[REPRO] rows inside 15m where OLD law accelerates toward target: " + reproRows + "/" + insideRows);
        if (reproSample != "") Console.WriteLine("        sample: " + reproSample);
        Check(reproRows > 0, "bug reproduces: old law pumps toward-target inside 15m");
        Check(insideRows > 0, "replay actually covered the inside-15m zone");
        Check(worstBrakeCross < 1e-6, "FIX: new correction strictly brake-only inside 15m (worst cross=" + worstBrakeCross.ToString("E2") + ")");
        Check(outsideRows > 0 && maxOutsideDiff < 1e-9, "REGR: outside 15m correction unchanged (rows=" + outsideRows + ", maxDiff=" + maxOutsideDiff.ToString("E2") + ")");
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "[TILT]  max uncapped steer tilt = {0:F1} deg; max after 15deg cap = {1:F2} deg", maxTiltUncapped, maxTiltCapped));
        Check(maxTiltUncapped > 25, "bug reproduces: uncapped tilt exceeded 25 deg in last 120m");
        Check(maxTiltCapped <= 15.01, "FIX: capped steer tilt <= 15 deg below 100m");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECK(S) FAILED");
        return failures == 0 ? 0 : 1;
    }
}
