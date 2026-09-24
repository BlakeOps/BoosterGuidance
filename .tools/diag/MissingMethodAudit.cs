// MissingMethod audit: loads BoosterGuidance.dll with the game's own assembly
// set (KSP_x64_Data/Managed + GameData mod DLLs), walks every method body of
// every BoosterGuidance/Trajectories-namespace type, and reports every member
// token that FAILS to resolve - i.e. the exact methods that would throw
// MissingMethodException in game. Deterministic, no flight needed.
using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;

class MissingMethodAudit
{
    static Dictionary<ushort, OpCode> opcodes = new Dictionary<ushort, OpCode>();
    static List<string> resolveDirs = new List<string>();

    static void InitOpcodes()
    {
        foreach (var f in typeof(OpCodes).GetFields(BindingFlags.Public | BindingFlags.Static))
        {
            var op = (OpCode)f.GetValue(null);
            opcodes[(ushort)op.Value] = op;
        }
    }

    static int OperandSize(OperandType t, byte[] il, ref int pos)
    {
        switch (t)
        {
            case OperandType.InlineNone: return 0;
            case OperandType.ShortInlineBrTarget:
            case OperandType.ShortInlineI:
            case OperandType.ShortInlineVar: return 1;
            case OperandType.InlineVar: return 2;
            case OperandType.InlineI:
            case OperandType.InlineBrTarget:
            case OperandType.InlineField:
            case OperandType.InlineMethod:
            case OperandType.InlineSig:
            case OperandType.InlineString:
            case OperandType.InlineTok:
            case OperandType.InlineType:
            case OperandType.ShortInlineR: return 4;
            case OperandType.InlineI8:
            case OperandType.InlineR: return 8;
            case OperandType.InlineSwitch:
                int n = BitConverter.ToInt32(il, pos);
                return 4 + 4 * n;
            default: return 0;
        }
    }

    static int Main(string[] args)
    {
        // The runtime cannot print some loader exceptions (their ToString
        // itself throws -> "Exception.ToString() failed" and no diagnostics).
        // Report the crash through type name + HResult only, never ToString
        try
        {
            return MainInner(args);
        }
        catch (Exception ex)
        {
            try { Console.WriteLine("AUDIT CRASH: " + ex.GetType().FullName + " hr=0x" + ex.HResult.ToString("X8")); }
            catch { }
            try { Console.WriteLine("msg: " + ex.Message); } catch { try { Console.WriteLine("msg: <unavailable>"); } catch { } }
            try { Console.WriteLine(ex.StackTrace); } catch { }
            return 2;
        }
    }

    static int MainInner(string[] args)
    {
        string bgDir = args.Length > 0 ? args[0] : @"E:\ksp_mod\BoosterGuidance\Source\bin\Release";
        string ksp = args.Length > 1 ? args[1] : @"D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program";
        string managed = Path.Combine(ksp, @"KSP_x64_Data\Managed");
        InitOpcodes();
        resolveDirs.Add(bgDir);
        resolveDirs.Add(managed);
        foreach (string dll in Directory.GetFiles(Path.Combine(ksp, "GameData"), "*.dll", SearchOption.AllDirectories))
            resolveDirs.Add(Path.GetDirectoryName(dll));
        AppDomain.CurrentDomain.AssemblyResolve += (s, e) =>
        {
            try
            {
                string name = new AssemblyName(e.Name).Name + ".dll";
                foreach (string dir in resolveDirs)
                {
                    string p = Path.Combine(dir, name);
                    if (File.Exists(p)) return Assembly.LoadFrom(p);
                }
            }
            catch (Exception ex)
            {
                try { Console.WriteLine("resolve failed for " + e.Name + ": " + ex.GetType().FullName); } catch { }
            }
            return null;
        };
        return Run(bgDir);
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    static int Run(string bgDir)
    {
        var asm = Assembly.LoadFrom(Path.Combine(bgDir, "BoosterGuidance.dll"));
        var failures = new Dictionary<string, int>();
        var icalls = new Dictionary<string, int>();
        var bclRefs = new HashSet<string>();
        int methods = 0, tokens = 0;
        Type[] types;
        try
        {
            types = asm.GetTypes();
        }
        catch (ReflectionTypeLoadException tle)
        {
            types = tle.Types;
            foreach (var le in tle.LoaderExceptions)
            {
                if (le == null) continue;
                try { Console.WriteLine("LOADER: " + le.GetType().FullName + ": " + le.Message); } catch { }
            }
        }
        foreach (var type in types)
        {
            if (type == null) continue;
            foreach (var m in type.GetMethods(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly))
            {
                AuditMember(type, m, failures, icalls, bclRefs, ref methods, ref tokens);
            }
            foreach (var c in type.GetConstructors(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static))
            {
                AuditMember(type, c, failures, icalls, bclRefs, ref methods, ref tokens);
            }
        }
        Console.WriteLine("Audited " + methods + " methods, " + tokens + " member tokens");
        var sorted = new List<string>(bclRefs);
        sorted.Sort();
        Console.WriteLine("--- " + sorted.Count + " distinct BCL (mscorlib/System*) member refs:");
        foreach (string s in sorted) Console.WriteLine("  BCL " + s);
        if (icalls.Count > 0)
        {
            Console.WriteLine("--- InternalCall callees (native icalls; unregistered ones throw MissingMethodException at first execution - flight 23 was Vector3d.Slerp):");
            foreach (var kv in icalls)
                Console.WriteLine("  x" + kv.Value + "  " + kv.Key);
        }
        int rc = 0;
        // icalls declared in Assembly-CSharp (KSP's own) are NOT reliably
        // registered - Vector3d.Slerp killed flight 23 while Vector3d.Angle
        // (managed) always worked. UnityEngine/mscorlib icalls are registered
        // by the engine/Mono and are safe.
        foreach (var kv in icalls)
            if (kv.Key.StartsWith("Assembly-CSharp"))
            {
                Console.WriteLine("FAIL: unregistered-icall risk: " + kv.Key);
                rc = 1;
            }
        if (failures.Count == 0)
        {
            Console.WriteLine("NO missing members - all tokens resolve against the game assemblies");
        }
        else
        {
            Console.WriteLine("MISSING MEMBERS (would throw MissingMethodException in game):");
            foreach (var kv in failures)
                Console.WriteLine("  x" + kv.Value + "  " + kv.Key);
            rc = 1;
        }
        return rc;
    }

    static void AuditMember(Type type, MethodBase m, Dictionary<string, int> failures, Dictionary<string, int> icalls, HashSet<string> bclRefs, ref int methods, ref int tokens)
    {
        MethodBody body;
        try { body = m.GetMethodBody(); } catch { return; }
        if (body == null) return;
        methods++;
        byte[] il = body.GetILAsByteArray();
        Module mod = m.Module;
        Type[] typeArgs = type.IsGenericType ? type.GetGenericArguments() : null;
        Type[] methodArgs = m.IsGenericMethod ? m.GetGenericArguments() : null;
        int pos = 0;
        while (pos < il.Length)
        {
            OpCode op;
            ushort b = il[pos++];
            if (b == 0xFE) b = (ushort)(0xFE00 | il[pos++]);
            if (!opcodes.TryGetValue(b, out op)) break; // unknown opcode - bail on this method
            OperandType ot = op.OperandType;
            if (ot == OperandType.InlineMethod || ot == OperandType.InlineField ||
                ot == OperandType.InlineType || ot == OperandType.InlineTok)
            {
                int token = BitConverter.ToInt32(il, pos);
                tokens++;
                try
                {
                    var resolved = mod.ResolveMember(token, typeArgs, methodArgs);
                    var mb = resolved as MethodBase;
                    if (mb != null && mb.DeclaringType != null)
                    {
                        string scope = mb.DeclaringType.Assembly.GetName().Name;
                        if (scope == "mscorlib" || scope.StartsWith("System"))
                            bclRefs.Add(mb.DeclaringType.FullName + "." + mb.Name);
                        if (mb.MethodImplementationFlags == MethodImplAttributes.InternalCall)
                        {
                            string key = scope + ": " + mb.DeclaringType.FullName + "." + mb.Name + " (called from " + type.Name + "." + m.Name + ")";
                            icalls[key] = icalls.ContainsKey(key) ? icalls[key] + 1 : 1;
                        }
                    }
                }
                catch (Exception ex)
                {
                    string key = string.Format("{0} -> token 0x{1:X8} in {2}: {3}",
                        op.Name, token, type.Name + "." + m.Name, ex.Message.Trim());
                    failures[key] = failures.ContainsKey(key) ? failures[key] + 1 : 1;
                }
            }
            pos += OperandSize(ot, il, ref pos);
        }
    }
}
