// LogFormatCheck - flight-logging-v2 gate (task 5.4).
// Unit-style verification of the v2 Actual.dat format against the built DLL
// and the source, without launching KSP:
//   T1  V2Columns manifest readable via reflection, arity == 40
//   T2  row format string in source has exactly {0}..{39} (arity matches)
//   T3  manifest's first 19 names == the legacy column order (verbatim prefix)
//   T4  synthetic v2 file round-trip: #-lines skipped, rows split to exactly
//       40 fields, numeric columns parse (NaN allowed), tags parse
//   T5  NaN renders as "NaN" under every format used in the row (F0/F1/F2/F3)
// 批次九-A (Simulate.dat v2, 17 columns, multi-run self-describing):
//   T6  SimV2Columns manifest on BoosterGuidance.Simulate, arity/order verbatim
//   T7  synthetic sim v2 multi-run round-trip (repeated '# format v2' = run delimiter)
//   T8  sim row format string in Simulate.cs has exactly {0}..{16}
// Usage: LogFormatCheck.exe <dir-with-BoosterGuidance.dll> <path-to-BLController.cs>
using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;

class LogFormatCheck
{
    static int failures = 0;

    static void Check(bool ok, string name, string detail)
    {
        Console.WriteLine((ok ? "  PASS " : "  FAIL ") + name + "  " + detail);
        if (!ok) failures++;
    }

    static int Main(string[] args)
    {
        string bgDir = args.Length > 0 ? args[0] : @"..\..\Source\bin\Release";
        string srcPath = args.Length > 1 ? args[1] : @"..\..\Source\Core\BLController.cs";
        string kspManaged = args.Length > 2 ? args[2] : null;
        if (kspManaged != null)
            AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
            {
                string p = Path.Combine(kspManaged, new AssemblyName(e.Name).Name + ".dll");
                return File.Exists(p) ? Assembly.LoadFrom(p) : null;
            };
        Console.WriteLine("LogFormatCheck against " + bgDir);

        // T1: manifest via reflection (private const)
        var asm = Assembly.LoadFrom(Path.Combine(bgDir, "BoosterGuidance.dll"));
        var t = asm.GetType("BoosterGuidance.BLController");
        var f = t.GetField("V2Columns", BindingFlags.NonPublic | BindingFlags.Static);
        Check(f != null, "[T1 CONST] ", "V2Columns field present");
        if (f == null) return 1;
        string manifest = (string)f.GetValue(null);
        string[] cols = manifest.Split(' ');
        Check(cols.Length == 40, "[T1 ARITY] ", "manifest columns = " + cols.Length + " (want 40)");

        // T2: the row format string in source covers {0}..{39}
        string src = File.ReadAllText(srcPath);
        var m = Regex.Match(src, "\"(\\{0:F1\\}[^\"]*\\{39[^\"]*)\"");
        Check(m.Success, "[T2 FMT]  ", "row format string located in source");
        int maxPh = -1; int phCount = 0;
        if (m.Success)
        {
            foreach (Match ph in Regex.Matches(m.Groups[1].Value, "\\{(\\d+)"))
            {
                maxPh = Math.Max(maxPh, int.Parse(ph.Groups[1].Value));
                phCount++;
            }
        }
        Check(maxPh == 39 && phCount == 40, "[T2 ARITY] ", "placeholders: max {" + maxPh + "}, count " + phCount + " (want 39/40)");

        // T3: legacy 19-column verbatim prefix
        string[] legacy = { "t", "phase", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az",
            "att_err", "amin", "amax", "steer_gain", "target_error", "totalMass", "traj_x", "traj_z" };
        bool prefix = cols.Take(19).SequenceEqual(legacy);
        Check(prefix, "[T3 LEGACY]", prefix ? "19 legacy columns verbatim" : "MISMATCH: " + string.Join(",", cols.Take(19)));

        // T4: synthetic v2 file round-trip
        string[] sample = {
            "# format v2",
            "# build deadbeef",
            "# columns " + manifest,
            "# settings maxAoA(l/a/r)=... (opaque)",
            "# phase Unset->TurnAround t=0.0 y=70000 vy=10.0 vh=250.0 dist=42000 terr=42000 thr=0.00 fuelKg=85000",
            "0.0 TurnAround 100.5 70000.0 -200.0 1.0 10.0 250.0 0.0 0.0 0.0 0.0 0.000 0.0 25.00 42100.0 45.20 NaN NaN 42100.0 42050.0 175.2 45.0 12.5 0.50 -9.81 0.1 -0.2 0.3 30.4 NaN NaN NaN 0.00 85000 2 NaN 04:20:20 TA-steer -",
            "0.1 TurnAround 100.5 69900.0 -201.0 1.0 9.0 249.0 0.0 0.0 0.0 0.0 0.000 0.0 25.00 42800.0 45.20 41000.0 40500.0 42700.0 42000.0 176.1 44.0 13.0 0.51 -9.80 0.1 -0.2 0.3 30.5 NaN NaN NaN 0.00 84000 2 1.5 04:20:21 TA-steer TERRJMP|THRJMP",
            "# engines on t=25.0 n=3",
            "# touchdown t=310.5 offset=9.3 vy=-2.1 vh=0.4 tilt=3.2 om=1.1 fuelKg=12000",
        };
        int rowNum = 0;
        bool arityOk = true, parseOk = true, tagsOk = true;
        foreach (string line in sample)
        {
            if (line.StartsWith("#")) continue; // header/event/epilogue lines skipped by design
            rowNum++;
            string[] parts = line.Split(' ');
            if (parts.Length != cols.Length) { arityOk = false; Console.WriteLine("    row " + rowNum + " arity " + parts.Length + " != " + cols.Length); }
            for (int i = 0; i < Math.Min(parts.Length, cols.Length); i++)
            {
                string cname = cols[i];
                if (cname == "phase" || cname == "wallT" || cname == "owner" || cname == "tags") continue;
                double dv;
                if (!double.TryParse(parts[i], out dv))
                {
                    // int column engOn parses as double too; only NaN/number expected
                    parseOk = false; Console.WriteLine("    row " + rowNum + " col " + cname + " unparsable: '" + parts[i] + "'");
                }
            }
            string tags = parts[parts.Length - 1];
            if (tags != "-" && !Regex.IsMatch(tags, "^[A-Z]+(\\|[A-Z]+)*$")) { tagsOk = false; Console.WriteLine("    row " + rowNum + " bad tags: " + tags); }
        }
        Check(rowNum == 2 && arityOk, "[T4 ARITY] ", "synthetic rows split to manifest arity");
        Check(parseOk, "[T4 PARSE] ", "all numeric columns parse (NaN allowed)");
        Check(tagsOk, "[T4 TAGS]  ", "tags column: '-' or |-joined CODES");

        // T5: NaN formatting under the row's format verbs
        bool nanOk = string.Format("{0:F1}", double.NaN) == "NaN"
                  && string.Format("{0:F0}", double.NaN) == "NaN"
                  && string.Format("{0:F2}", double.NaN) == "NaN"
                  && string.Format("{0}", double.NaN) == "NaN";
        Check(nanOk, "[T5 NaN]   ", "NaN renders as 'NaN' under F0/F1/F2/plain");

        // ---- 批次九-A: Simulate.dat v2 (multi-run, self-describing) ----
        // T6: SimV2Columns manifest on BoosterGuidance.Simulate, arity 17
        var st = asm.GetType("BoosterGuidance.Simulate");
        Check(st != null, "[T6 TYPE]  ", "BoosterGuidance.Simulate present");
        var sf = (st != null) ? st.GetField("SimV2Columns", BindingFlags.NonPublic | BindingFlags.Static) : null;
        Check(sf != null, "[T6 CONST] ", "SimV2Columns field present");
        if (sf == null) { Console.WriteLine(failures + " CHECKS FAILED"); return 1; }
        string simManifest = (string)sf.GetValue(null);
        string[] simCols = simManifest.Split(' ');
        Check(simCols.Length == 17, "[T6 ARITY] ", "sim manifest columns = " + simCols.Length + " (want 17)");
        string[] simWant = { "t","x","y","z","vx","vy","vz","aeroH","aeroV","thrH","thrV","throttle","tiltDeg","phase","owner","target_error","total_mass" };
        Check(simCols.SequenceEqual(simWant), "[T6 ORDER] ", "sim manifest order verbatim");

        // T7: synthetic sim v2 multi-run round-trip (header repeats = run delimiter)
        string[] simSample = {
            "# format v2",
            "# build deadbeef",
            "# columns " + simManifest,
            "# tgtAlt=0",
            "286.00 -1234.50000 14500.00000 678.90000 -45.20000 -210.30000 30.10000 -15.20 -3.40 5.10 9.60 0.750 44.2 LandingBurn vh-kill-floor 4210.50 195000.00",
            "288.00 -1300.50000 14100.00000 700.90000 -44.20000 -205.30000 29.10000 NaN -9.50 NaN 8.60 0.000 12.2 LandingBurn term-falcon 4300.50 195000.00",
            "# format v2",
            "# columns " + simManifest,
            "0.00 100.50000 70000.00000 -200.00000 1.00000 10.00000 250.00000 0.00 -9.81 0.00 0.00 0.000 175.0 Coasting Coasting 42100.00 195000.00",
        };
        int simRowNum = 0, simRuns = 0;
        bool simArityOk = true, simParseOk = true;
        foreach (string line in simSample)
        {
            if (line.StartsWith("#")) { if (line.StartsWith("# format")) simRuns++; continue; }
            simRowNum++;
            string[] parts = line.Split(' ');
            if (parts.Length != simCols.Length) { simArityOk = false; Console.WriteLine("    sim row " + simRowNum + " arity " + parts.Length + " != " + simCols.Length); }
            for (int i = 0; i < Math.Min(parts.Length, simCols.Length); i++)
            {
                string cname = simCols[i];
                if (cname == "phase" || cname == "owner") continue;
                double dv;
                if (!double.TryParse(parts[i], out dv))
                { simParseOk = false; Console.WriteLine("    sim row " + simRowNum + " col " + cname + " unparsable: '" + parts[i] + "'"); }
            }
        }
        Check(simRowNum == 3 && simArityOk, "[T7 ARITY] ", "synthetic sim rows split to manifest arity");
        Check(simParseOk, "[T7 PARSE] ", "all numeric sim columns parse (NaN allowed)");
        Check(simRuns == 2, "[T7 RUNS]  ", "repeated '# format v2' delimits runs (" + simRuns + ")");

        // T8: the sim row format string in Simulate.cs covers {0}..{16}
        string simSrcPath = Path.Combine(Path.GetDirectoryName(srcPath), "Simulate.cs");
        string simSrc = File.ReadAllText(simSrcPath);
        var sm = Regex.Match(simSrc, "\"(\\{0:F2\\} \\{1:F5\\}[^\"]*\\{16:F2\\})\"");
        Check(sm.Success, "[T8 FMT]  ", "sim row format string located in Simulate.cs");
        int simMaxPh = -1; int simPhCount = 0;
        if (sm.Success)
        {
            foreach (Match ph in Regex.Matches(sm.Groups[1].Value, "\\{(\\d+)"))
            {
                simMaxPh = Math.Max(simMaxPh, int.Parse(ph.Groups[1].Value));
                simPhCount++;
            }
        }
        Check(simMaxPh == 16 && simPhCount == 17, "[T8 ARITY] ", "sim placeholders: max {" + simMaxPh + "}, count " + simPhCount + " (want 16/17)");

        Console.WriteLine(failures == 0 ? "ALL CHECKS PASSED" : failures + " CHECKS FAILED");
        return failures == 0 ? 0 : 1;
    }
}
