using System;
using System.Linq;
using System.Reflection;
using UnityEngine;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    /// <summary>
    /// Reflection wrapper for the Trajectories mod's public API (optional
    /// dependency - BoosterGuidance must load and run without it, same
    /// pattern as the FAR access in Trajectories/FARModel.cs). Used by the
    /// starship Traj-calibration mode (user request after f76): the
    /// calibration reads TRAJECTORIES' predicted impact point instead of
    /// our own prediction sim - "校准以Trajectories的为准".
    ///
    /// Frame note: API.GetImpactPosition() returns the impact as a
    /// body-relative world position rotated back to the CURRENT universal
    /// time (Trajectory.CalculateRotatedPosition), i.e. directly
    /// comparable to our tgt_r = body.GetWorldSurfacePosition(...) -
    /// body.position at read time. Between Trajectories' own refreshes
    /// the caller rotates the stored value forward with the body spin.
    /// </summary>
    public static class TrajAPI
    {
        private static bool searched = false;
        private static MethodInfo getImpactPosition = null;
        private static PropertyInfo alwaysUpdate = null;
        private static bool available = false;
        private static bool failLogged = false;
        // f104 starship profile auto-select (all optional - the stock
        // Trajectories build has no custom profiles, every member may
        // come back null and the feature just stays dark)
        private static FieldInfo dpCustomEnabled = null;
        private static FieldInfo dpCustomName = null;
        private static MethodInfo dpSave = null;
        private static PropertyInfo settingsBodyFixed = null;
        private static MethodInfo cdpKnotCount = null;

        public static bool Available
        {
            get
            {
                if (!searched)
                    Search();
                return available;
            }
        }

        private static void Search()
        {
            searched = true;
            try
            {
                AssemblyLoader.LoadedAssembly traj =
                    AssemblyLoader.loadedAssemblies.SingleOrDefault(a => a.dllName == "Trajectories");
                if (traj == null)
                    return;
                Type api = traj.assembly.GetTypes().SingleOrDefault(t => t.FullName == "Trajectories.API");
                if (api == null)
                    return;
                getImpactPosition = api.GetMethod("GetImpactPosition",
                    BindingFlags.Public | BindingFlags.Static);
                alwaysUpdate = api.GetProperty("AlwaysUpdate",
                    BindingFlags.Public | BindingFlags.Static);
                available = (getImpactPosition != null);
                // f104: the custom build (E:\ksp_mod\KSPTrajectories) adds a
                // named multi-knot descent profile + per-vessel opt-in; grab
                // the knobs so starship mode can select StarshipBelly itself
                Type dp = traj.assembly.GetTypes().SingleOrDefault(t => t.FullName == "Trajectories.DescentProfile");
                if (dp != null)
                {
                    dpCustomEnabled = dp.GetField("CustomEnabled", BindingFlags.NonPublic | BindingFlags.Static);
                    dpCustomName = dp.GetField("CustomName", BindingFlags.NonPublic | BindingFlags.Static);
                    dpSave = dp.GetMethod("Save", BindingFlags.NonPublic | BindingFlags.Static, null, Type.EmptyTypes, null);
                }
                Type cdp = traj.assembly.GetTypes().SingleOrDefault(t => t.FullName == "Trajectories.CustomDescentProfile");
                if (cdp != null)
                    cdpKnotCount = cdp.GetMethod("KnotCount", BindingFlags.NonPublic | BindingFlags.Static);
                Type settings = traj.assembly.GetTypes().SingleOrDefault(t => t.FullName == "Trajectories.Settings");
                if (settings != null)
                    settingsBodyFixed = settings.GetProperty("BodyFixedMode", BindingFlags.NonPublic | BindingFlags.Static);
            }
            catch (Exception)
            {
                available = false; // duplicate assemblies etc - treat as absent
            }
        }

        /// <summary>
        /// Trajectories' predicted impact, body-relative world position in
        /// the current rotating frame, or null when Trajectories is absent,
        /// has no computed trajectory, or the call failed. Never throws.
        /// </summary>
        public static Vector3d? GetImpactPosition()
        {
            if (!Available)
                return null;
            try
            {
                object result = getImpactPosition.Invoke(null, null);
                // boxed Nullable<Vector3>: null when it has no value
                if (result is Vector3)
                    return (Vector3d)(Vector3)result;
            }
            catch (Exception e)
            {
                if (!failLogged)
                {
                    failLogged = true;
                    Log.Info("[TrajCal] Trajectories API call failed (disabled until next scene): " + e.Message);
                }
                available = false; // do not spam the invoke every tick
            }
            return null;
        }

        private static bool alwaysUpdateForced = false;

        /// <summary>
        /// Force Trajectories to keep computing its trajectory even with
        /// its window closed (it skips computation otherwise when no
        /// target is set). Needed so the calibration has fresh data.
        /// Setting true is once-guarded (callers invoke every frame).
        /// </summary>
        public static void SetAlwaysUpdate(bool value)
        {
            if ((value) && (alwaysUpdateForced))
                return;
            if (!Available || (alwaysUpdate == null))
                return;
            try
            {
                alwaysUpdate.SetValue(null, value, null);
                if (value)
                    alwaysUpdateForced = true;
            }
            catch (Exception)
            {
                // cosmetic - Trajectories just keeps its own cadence
            }
        }

        private static float lastProfileCheck = -100; // Time.time, rate limit
        private static bool profileMissingLogged = false;

        /// <summary>
        /// f104 (user: "当我选择星舰模式之后Tri的落点预测轨迹自动变成星舰的
        /// 配置,不然每次都得点很麻烦"): select the custom StarshipBelly
        /// descent profile on the ACTIVE vessel and force BodyFixedMode -
        /// the manual Trajectories GUI clicks the user had to redo before
        /// every flight. DescentProfile state is per-vessel (Clear() on
        /// vessel switch), so this re-checks once per second and re-applies
        /// after every vessel change; callers gate on starship mode so a
        /// falcon booster is never touched. Stock Trajectories (no custom
        /// profile support) makes this a silent no-op.
        /// </summary>
        public static void SetStarshipProfile(bool force = false)
        {
            if (!Available || (dpCustomEnabled == null) || (dpCustomName == null))
                return;
            if ((!force) && (Time.time - lastProfileCheck < 1)) // reflection - once a second
                return;
            lastProfileCheck = Time.time;
            try
            {
                // The named profile must exist in DescentProfiles.cfg -
                // GetAngleOfAttack silently falls back to the legacy 4-node
                // profile for an unknown name, which would look "set" but
                // fly the wrong aerodynamics
                if (cdpKnotCount != null)
                {
                    int knots = (int)cdpKnotCount.Invoke(null, new object[] { "StarshipBelly" });
                    if (knots <= 0)
                    {
                        if (!profileMissingLogged)
                        {
                            profileMissingLogged = true;
                            Log.Info("[TrajCal] StarshipBelly profile not found in DescentProfiles.cfg - leaving the Trajectories profile alone");
                        }
                        return;
                    }
                }
                bool enabled = (bool)dpCustomEnabled.GetValue(null);
                string name = (string)dpCustomName.GetValue(null);
                bool bodyFixed = (settingsBodyFixed != null) && (bool)settingsBodyFixed.GetValue(null, null);
                if ((enabled) && (name == "StarshipBelly") && (bodyFixed))
                    return; // already set - nothing to do
                dpCustomEnabled.SetValue(null, true);
                dpCustomName.SetValue(null, "StarshipBelly");
                if (settingsBodyFixed != null)
                    settingsBodyFixed.SetValue(null, true, null);
                if (dpSave != null)
                    dpSave.Invoke(null, null); // persist to the vessel module (survives save/load)
                Log.Info("[TrajCal] auto-selected Trajectories profile StarshipBelly + BodyFixedMode (starship mode)");
            }
            catch (Exception e)
            {
                Log.Info("[TrajCal] starship profile auto-select failed (disabled until next scene): " + e.Message);
                dpCustomEnabled = null; // do not retry every second
            }
        }
    }
}
