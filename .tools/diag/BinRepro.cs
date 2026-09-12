using System;
using System.IO;
using System.Reflection;

// Decisive repro for the f64 IndexOutOfRangeException in EffectiveCalLift:
// load the BUILT BoosterGuidance.dll exactly like MissingMethodAudit does,
// new up a controller, and hammer the bin accessors.
class BinRepro
{
    static int Main(string[] args)
    {
        string root = args.Length > 0 ? args[0] : Path.GetFullPath(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, @"..\.."));
        string ksp = Environment.GetEnvironmentVariable("KSPDIR");
        if (string.IsNullOrEmpty(ksp)) ksp = @"D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program";
        string managed = Path.Combine(ksp, @"KSP_x64_Data\Managed");
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            string name = new AssemblyName(e.Name).Name + ".dll";
            foreach (string dir in new[] { managed, Path.Combine(root, @"Source\bin\Release") })
            {
                string p = Path.Combine(dir, name);
                if (File.Exists(p)) return Assembly.LoadFrom(p);
            }
            return null;
        };
        var asm = Assembly.LoadFrom(Path.Combine(root, @"Source\bin\Release\BoosterGuidance.dll"));
        Type t = asm.GetType("BoosterGuidance.BLController");
        object c = Activator.CreateInstance(t);
        foreach (string f in new[] { "aeroCalLiftQ", "aeroCalDragQ", "aeroCalSamplesQ", "calBinMinSamples", "aeroCalLift" })
        {
            var fi = t.GetField(f);
            object v = fi.GetValue(c);
            Array a = v as Array;
            Console.WriteLine(f + " = " + ((a != null) ? ("len " + a.Length) : (v == null ? "null" : v.ToString())));
        }
        var edges = t.GetField("AeroCalQEdges").GetValue(null) as Array;
        Console.WriteLine("AeroCalQEdges = len " + ((edges == null) ? -1 : edges.Length));
        var bin = t.GetMethod("AeroCalBin");
        var effL = t.GetMethod("EffectiveCalLift");
        var effD = t.GetMethod("EffectiveCalDrag");
        foreach (double q in new[] { 0.0, 100.0, 3999.0, 4000.0, 4001.0, 9999.0, 10000.0, 10001.0, 20000.0, double.NaN, double.PositiveInfinity, -5.0 })
        {
            try
            {
                object b = bin.Invoke(c, new object[] { q });
                object l = effL.Invoke(c, new object[] { q });
                object d = effD.Invoke(c, new object[] { q });
                Console.WriteLine("q=" + q + " bin=" + b + " kL=" + l + " kD=" + d);
            }
            catch (Exception e)
            {
                Console.WriteLine("q=" + q + " THREW: " + e.GetBaseException());
            }
        }
        // now simulate a copy-ctor round trip like the prediction does
        object c2 = Activator.CreateInstance(t, c);
        Console.WriteLine("copy: liftQ len " + ((Array)t.GetField("aeroCalLiftQ").GetValue(c2)).Length);
        Console.WriteLine("copy EffectiveCalLift(5000) = " + effL.Invoke(c2, new object[] { 5000.0 }));
        Console.WriteLine("DONE");
        return 0;
    }
}
