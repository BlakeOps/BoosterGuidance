using System;
using System.IO;
using System.Reflection;

// Dump the IL of EffectiveCalLift / AeroCalBin from the built DLL
class IlDump
{
    static readonly string[] oneByte = new string[256];
    static int Main(string[] args)
    {
        string root = args.Length > 0 ? args[0] : ".";
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
        foreach (string mn in new[] { "AeroCalBin", "EffectiveCalLift", "EffectiveCalDrag" })
        {
            var m = t.GetMethod(mn);
            var body = m.GetMethodBody();
            byte[] il = body.GetILAsByteArray();
            Console.WriteLine("=== " + mn + " (" + il.Length + " bytes) ===");
            int i = 0;
            while (i < il.Length)
            {
                int off = i;
                byte op = il[i++];
                string name = op.ToString("X2");
                if (op == 0xFE) { byte op2 = il[i++]; name = "FE" + op2.ToString("X2"); }
                object operand = "";
                int sz = 0;
                switch (name)
                {
                    case "20": sz = 8; operand = BitConverter.ToInt64(il, i); break; // ldc.i8
                    case "1F": sz = 4; operand = BitConverter.ToInt32(il, i); break; // ldc.i4
                    case "23": sz = 8; operand = BitConverter.ToDouble(il, i); break; // ldc.r8
                    case "28": case "6F": case "73": case "7B": case "7E": case "80": case "7D": case "74": case "25": case "8C": case "72": case "D0": // call/callvirt/newobj/ldfld/ldsfld/stsfld/stfld/... 4-byte token
                    case "70": case "A5":
                        sz = 4;
                        int tok = BitConverter.ToInt32(il, i);
                        try { operand = m.Module.ResolveMember(tok).ToString(); } catch { operand = "tok " + tok.ToString("X8"); }
                        break;
                    case "38": sz = 4; operand = "br " + (i + 4 + BitConverter.ToInt32(il, i)); break;
                    case "2B": case "2C": case "2D": case "2E": case "2F": case "30": case "31": case "32": case "33": case "34": case "35": case "36": case "37":
                        sz = 1; operand = "br " + (i + 1 + (sbyte)il[i]); break;
                    default:
                        if ((op >= 0x0E && op <= 0x12) || (op >= 0x06 && op <= 0x09) || op == 0x1E || op == 0x1E) { sz = 0; break; } // ldc.i4.0-4/ldarg
                        if (op == 0x1F && false) break;
                        // table-driven for a few common ones
                        if (op == 0x11 || op == 0x12 || op == 0x13) { sz = 1; operand = il[i]; break; } // ldloc.s etc
                        if (op >= 0x16 && op <= 0x1E) { sz = 0; break; } // ldc.i4.N / ldc.i4.m1
                        break;
                }
                i += sz;
                Console.WriteLine(off.ToString("X4") + ": " + name + " " + operand);
            }
        }
        Console.WriteLine("DONE");
        return 0;
    }
}
