using System;
using System.Linq;
using System.Collections.Generic;
using KSP.UI.Screens;
using KSP.Localization;
using UnityEngine;
using UnityEngine.Profiling;
using ClickThroughFix;
using ToolbarControl_NS;
using static BoosterGuidance.InitLog;




// New rules
// - Window must be associated with BoosterGuidanceCore and BLController thro' core
// - All settings directly update core when changed
// - Controller settings need to update too - how?


namespace BoosterGuidance
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]

    public class MainWindow : MonoBehaviour
    {
        // constants
        Color tgt_color = new Color(1, 1, 0, 0.5f);
        Color pred_color = new Color(1, 0.2f, 0.2f, 0.5f);

        // private
        int tab = 0;
        bool hidden = true;
        BoosterGuidanceCore core = null;
        float maxReentryGain = 0.2f;
        float maxAeroDescentGain = 0.1f;
        float maxLandingBurnGain = 0.3f;
        float maxSteerAngle = 30; // 30 degrees
        Rect windowRect = new Rect(150, 150, 250, 680);

        // Main GUI Elements
        bool showTargets = true;
        bool debug = false; // show steer
        bool tgtSet = false;

        bool enableRCS = true;
        int actionGroup = 0;
        float degreeRange = 10f;
        // Hotkey editor (KeyCode name; parsed by the core, empty = unbound)
        string hotkeyGuidance = "Backspace";

        EditableAngle tgtLatitude = 0;
        EditableAngle tgtLongitude = 0;
        EditableInt tgtAlt = 0;
        int presetIndex = 0;
        string presetName = "";
        string presetBody = "";
        bool showLandingSites = false; // preset section starts collapsed to keep the window compact
        EditableInt reentryBurnAlt = 55000;
        // f113 (user request): super-heavy booster brake-depth knob on the
        // F9 panel - HIGHER = brake deeper in the re-entry burn = gentler
        // descent + better accuracy. Maps to core.reentryBurnTargetSpeed:
        // 0% -> 700 m/s (legacy default), 50% -> 500, 100% -> 300
        EditableInt heavyBrakeDepthPct = 0;
        string numLandingBurnEngines = "current";

        // Advanced GUI Elements
        EditableInt touchdownMargin = 25;
        EditableDouble touchdownSpeed = 2;
        EditableInt noSteerHeight = 100;
        EditableInt uprightHeight = 250;
        EditableDouble uprightMaxHorizSpeed = 5;
        EditableDouble steerDamping = 1.0;
        bool deployLandingGear = true;
        EditableInt deployLandingGearHeight = 500;
        EditableInt igniteDelay = 3; // Needed for RO

        // Targeting
        ITargetable lastVesselTarget = null;
        double lastNavLat = 0;
        double lastNavLon = 0;
        bool pickingPositionTarget = false;

        double pickLat, pickLon, pickAlt;

        // GUI Elements
        Color red = new Color(1, 0, 0, 0.5f);
        static bool hasFAR = false;

        internal const string MODID = "BoosterGuidance";
        internal const string MODNAME = "Booster Guidance";

        private static ToolbarControl toolbarControl;

        public void Awake()
        {
            InitToolbarButton();
        }

        private void InitToolbarButton()
        {
#if false
            Localizer.Init();
#endif
            if (toolbarControl == null)
            {
                toolbarControl = gameObject.AddComponent<ToolbarControl>();
                toolbarControl﻿.AddToAllToolbars(
                    Show, Hide,

                    ApplicationLauncher.AppScenes.MAPVIEW | ApplicationLauncher.AppScenes.FLIGHT,

                    MODID,
                    "boosterGuidanceButton",
                    "BoosterGuidance/PluginData/BoosterGuidanceIcon",
                   "BoosterGuidance/PluginData/BoosterGuidanceIcon",
                    MODNAME);

            }
        }


        public void Start()
        {
            hasFAR = Trajectories.AerodynamicModelFactory.HasFAR();
            Log.Info("Start hasFAR=" + hasFAR);

        }

        public void Update()
        {
            // Hotkey polling lives in this always-loaded flight addon rather
            // than the PartModule (f151 user report: the key seemed dead until
            // the window had been opened once). KSP instantiates this addon at
            // flight-scene start, so the key now works whether or not the
            // window has ever been shown. Still ignored while typing in a
            // text field (checked again inside PollHotkeys)
            if (GUIUtility.keyboardControl != 0)
                return;
            Vessel av = FlightGlobals.ActiveVessel;
            if (av == null)
                return;
            BoosterGuidanceCore c = BoosterGuidanceCore.GetBoosterGuidanceCore(av);
            if (c != null)
                c.PollHotkeys();
        }

        public void OnGUI()
        {
            if (!hidden)
            {
                Version version = typeof(BoosterGuidanceCore).Assembly.GetName().Version;
                string title = String.Format("BoosterGuidance {0}.{1}.{2}", version.Major, version.Minor, version.Build);
                windowRect = ClickThruBlocker.GUIWindow(0, windowRect, WindowFunction, title);
            }
        }

        public void OnDestroy()
        {
            hidden = true;
            if (Targets.targetingCross != null)
                Targets.targetingCross.enabled = false;
            if (Targets.predictedCross != null)
                Targets.predictedCross.enabled = false;
        }

        private BoosterGuidanceCore CheckCore(Vessel vessel)
        {
            if (core == null)
            {
                core = BoosterGuidanceCore.GetBoosterGuidanceCore(vessel);
                if (core != null)
                {
                    UpdateFromCore();
                    Targets.SetVisibility(showTargets, showTargets && core.Enabled() && (FlightGlobals.ActiveVessel == core.vessel));
                }
                else
                {
                    return core;
                }
            }
            else
            {
                if (core.vessel != vessel)
                {
                    Log.Info("core.vessel!=vessel vessel=" + vessel + " map=" + MapView.MapIsEnabled);
                    // Get new BoosterGuidanceCore as vessel changed
                    core = BoosterGuidanceCore.GetBoosterGuidanceCore(vessel);
                    UpdateFromCore();
                }
            }
            if (core == null)
                Log.Info("Vessel " + vessel.name + " has no BoosterGuidanceCore");
            return core;
        }

        void SetEnabledColors(bool phaseEnabled)
        {
            if (phaseEnabled)
            {
                GUI.skin.button.normal.textColor = new Color(1, 1, 1, 1);
                GUI.skin.label.normal.textColor = new Color(1, 1, 1, 1);
                GUI.skin.toggle.normal.textColor = new Color(1, 1, 1, 1);
                GUI.skin.box.normal.textColor = new Color(1, 1, 1, 1);
                GUI.skin.textArea.normal.textColor = new Color(1, 1, 1, 1);
                GUI.skin.textField.normal.textColor = new Color(1, 1, 1, 1);
            }
            else
            {
                GUI.skin.button.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
                GUI.skin.label.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
                GUI.skin.toggle.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
                GUI.skin.box.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
                GUI.skin.textArea.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
                GUI.skin.textField.normal.textColor = new Color(0.6f, 0.6f, 0.6f, 1);
            }
        }

        void WindowFunction(int windowID)
        {
            BoosterGuidanceCore core = CheckCore(FlightGlobals.ActiveVessel);
            if (core == null)
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("No BoosterGuidance Core");
                GUILayout.EndHorizontal();
                return;
            }

            OnUpdate();
            SetEnabledColors(true);
            // Close button
            if (GUI.Button(new Rect(windowRect.width - 18, 2, 16, 16), ""))
            {
                Hide();
                return;
            }

            // f129 UI cleanup (user: 应急着陆不要放在返回流程列表里,改成面板
            // 角落的红色圆形按钮): pinned bottom-right, visible on BOTH tabs,
            // no target needed - an emergency control must be reachable even
            // when the flow list is not. Clicking sets GUI.changed so the
            // UpdateCore() at the end of this pass runs - sync the window
            // fields to the emergency anchor first or the stale window
            // target clobbers it in the same frame
            DrawEmergencyButton(core);

            // Check for target being set
            if (core.vessel.targetObject != lastVesselTarget)
            {
                if (core.vessel.targetObject != null)
                {
                    Vessel target = core.vessel.targetObject.GetVessel();
                    tgtLatitude = target.latitude;
                    tgtLongitude = target.longitude;
                    tgtAlt = (int)target.altitude;
                    UpdateCore();
                    string msg = String.Format(Localizer.Format("#BoosterGuidance_TargetSetToX"), target.name);
                    GuiUtils.ScreenMessage(msg);
                    tgtSet = true;
                }
                lastVesselTarget = core.vessel.targetObject;
            }

            // Check for navigation target
            NavWaypoint nav = NavWaypoint.fetch;
            if (nav.IsActive)
            {
                // Does current nav position differ from last one used? A hack because
                // a can't see a way to check if the nav waypoint has changed
                // Doing it this way means lat and lon in window can be edited without them
                // getting locked to the nav waypoint
                if ((lastNavLat != nav.Latitude) || (lastNavLon != nav.Longitude))
                {
                    Coordinates pos = new Coordinates(nav.Latitude, nav.Longitude);
                    Log.Info("Target set to nav location " + pos.ToStringDMS());
                    tgtLatitude = nav.Latitude;
                    tgtLongitude = nav.Longitude;
                    lastNavLat = nav.Latitude;
                    lastNavLon = nav.Longitude;
                    tgtSet = true;

                    // This is VERY unreliable
                    //tgtAlt = (int)nav.Altitude;
                    tgtAlt = (int)FlightGlobals.ActiveVessel.mainBody.TerrainAltitude(tgtLatitude, tgtLongitude);
                    core.SetTarget(tgtLatitude, tgtLongitude, tgtAlt);
                    string msg = String.Format(Localizer.Format("#BoosterGuidance_TargetSetToX"), pos.ToStringDMS());
                    GuiUtils.ScreenMessage(msg);
                    UpdateCore();
                }
            }
            else
            {
                lastNavLat = 0;
                lastNavLon = 0;
            }

            // Check for unloaded vessels
            foreach (var controller in BoosterGuidanceCore.controllers)
            {
                if (!controller.vessel.loaded)
                {
                    GuiUtils.ScreenMessage("Guidance disabled for " + controller.vessel.name + " as out of physics range");
                    DisableGuidance();
                }
            }


            tab = GUILayout.Toolbar(tab, new string[] { Localizer.Format("#BoosterGuidance_Main"), Localizer.Format("Advanced") });
            bool changed = false;
            switch (tab)
            {
                case 0:
                    changed = MainTab(windowID);
                    break;
                case 1:
                    changed = AdvancedTab(windowID);
                    break;
            }

            if (changed)
                UpdateCore();
        }

        // Emergency land-anywhere: one click, no target needed - brake the
        // horizontal drift and land vertically wherever the ship is. The real
        // target is restored on disable. Rendered as a red round button in
        // the panel corner (f129 user request), NOT a row in the phase flow
        private static Texture2D emergencyTex = null;
        private static GUIStyle emergencyStyle = null;

        private void DrawEmergencyButton(BoosterGuidanceCore core)
        {
            if (emergencyTex == null)
            {
                int sz = 32;
                float rr = sz / 2f - 0.5f;
                emergencyTex = new Texture2D(sz, sz, TextureFormat.RGBA32, false);
                for (int yy = 0; yy < sz; yy++)
                    for (int xx = 0; xx < sz; xx++)
                    {
                        float dx = xx - sz / 2f + 0.5f, dy = yy - sz / 2f + 0.5f;
                        float d = Mathf.Sqrt(dx * dx + dy * dy);
                        emergencyTex.SetPixel(xx, yy, (d <= rr) ? new Color(0.75f, 0.05f, 0.05f, 1f) : new Color(0, 0, 0, 0));
                    }
                emergencyTex.Apply();
                emergencyStyle = new GUIStyle(GUI.skin.button);
                emergencyStyle.normal.background = emergencyTex;
                emergencyStyle.hover.background = emergencyTex;
                emergencyStyle.active.background = emergencyTex;
                emergencyStyle.normal.textColor = Color.white;
                emergencyStyle.fontStyle = FontStyle.Bold;
                emergencyStyle.border = new RectOffset(0, 0, 0, 0);
                emergencyStyle.padding = new RectOffset(0, 0, 0, 0);
            }
            Rect er = new Rect(windowRect.width - 34, windowRect.height - 34, 28, 28);
            if (GUI.Button(er, new GUIContent("!", Localizer.Format("#BoosterGuidance_EmergencyLand")), emergencyStyle))
            {
                core.EmergencyLand();
                tgtLatitude = core.tgtLatitude;
                tgtLongitude = core.tgtLongitude;
                tgtAlt = (int)core.tgtAlt;
            }
        }

        bool AdvancedTab(int windowID)
        {
            // Margin
            // Touchdown speed
            // No steer height

            GUILayout.BeginHorizontal();
            deployLandingGear = GUILayout.Toggle(deployLandingGear, Localizer.Format("#BoosterGuidance_DeployGear"));
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_DeployGearHeight"), deployLandingGearHeight, "m", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_NoSteerHeight"), noSteerHeight, "m", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_UprightHeight"), uprightHeight, "m", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_UprightMaxHorizSpeed"), uprightMaxHorizSpeed, "m/s", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_SteerDamping"), steerDamping, "s", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_TouchdownMargin"), touchdownMargin, "m", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_TouchdownSpeed"), touchdownSpeed, "m/s", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_IgniteDelay"), igniteDelay, "s", 65);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            debug = GUILayout.Toggle(debug, Localizer.Format("#BoosterGuidance_Debug"));
            Targets.showSteer = debug;
            GUILayout.EndHorizontal();

            // Show all active vessels
            GUILayout.Space(10);
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#BoosterGuidance_OtherVessels") + ":");
            GUILayout.EndHorizontal();
            GUIStyle info_style = new GUIStyle();
            info_style.normal.textColor = Color.white;
            GUIStyle red_style = new GUIStyle();
            red_style.normal.textColor = Color.red;
            foreach (var controller in BoosterGuidanceCore.controllers)
            {
                try
                {
                    if ((controller != null) && (controller.enabled) && (controller.vessel != FlightGlobals.ActiveVessel))
                    {
                        GUILayout.BeginHorizontal();
                        GUILayout.Label(controller.vessel.name + " (" + (int)controller.vessel.altitude + "m)");
                        GUILayout.FlexibleSpace();
                        //if (GUILayout.Button("X", GUILayout.Width(26))) // Cancel guidance
                        //  DisableGuidance();
                        GUILayout.EndHorizontal();

                        GUILayout.BeginHorizontal();
                        GUILayout.Label("  " + controller.info, info_style);
                        GUILayout.EndHorizontal();

                        GUILayout.BeginHorizontal();
                        GUILayout.Label("  " + controller.PhaseStr(), info_style);
                        GUILayout.EndHorizontal();
                    }
                }
                catch (Exception e) // TODO - Use correct exception to invalid reference
                {
                    Log.Info("Removing controller: " + e.Message);
                    BoosterGuidanceCore.controllers.Remove(controller);
                }
            }

            GUI.DragWindow();
            return GUI.changed;
        }

        bool MainTab(int windowID)
        {
            bool targetChanged = false;
            BoosterGuidanceCore core = CheckCore(FlightGlobals.ActiveVessel);
            BLControllerPhase phase = core.Phase();
            bool starship = core.recoveryProfile == "starship";

            // Recovery profile (per-vessel, persisted; default falcon9).
            // Starship auto-picks its phases at enable, so the manual phase
            // buttons below are hidden for it
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#BoosterGuidance_Profile"));
            string profileLabel = starship ? Localizer.Format("#BoosterGuidance_ProfileStarship") : Localizer.Format("#BoosterGuidance_ProfileFalcon9");
            if (GUILayout.Button(profileLabel))
            {
                core.recoveryProfile = starship ? "falcon9" : "starship";
                core.Changed();
                // f104: selecting starship mode immediately switches the
                // Trajectories prediction to the StarshipBelly profile too
                // (user: 不然每次都得点很麻烦) - works pre-enable because the
                // controller does not exist yet; rate-limited inside
                if (core.recoveryProfile == "starship")
                    TrajAPI.SetStarshipProfile(true); // force: the 1s rate limiter must not swallow the click
            }
            GUILayout.EndHorizontal();

            // Target:

            // Draw any Controls inside the window here
            GUILayout.Label(Localizer.Format("#BoosterGuidance_Target"));//Target coordinates:

            GUILayout.BeginHorizontal();
            double step = 1.0 / (60 * 60); // move by 1 arc second
            tgtLatitude.DrawEditGUI(EditableAngle.Direction.NS);
            if (GUILayout.Button("▲"))
            {
                tgtLatitude += step;
                targetChanged = true;
            }
            if (GUILayout.Button("▼"))
            {
                tgtLatitude -= step;
                targetChanged = true;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            tgtLongitude.DrawEditGUI(EditableAngle.Direction.EW);
            if (GUILayout.Button("◄"))
            {
                tgtLongitude -= step;
                targetChanged = true;
            }
            if (GUILayout.Button("►"))
            {
                tgtLongitude += step;
                targetChanged = true;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button(Localizer.Format("#BoosterGuidance_PickTarget")))
                PickTarget();
            if (GUILayout.Button("Set Here"))
                SetTargetHere();
            GUILayout.EndHorizontal();

            // Landing site presets (filtered to current body)
            string presetBodyName = FlightGlobals.ActiveVessel.mainBody.name;
            List<LandingSite> bodySites = LandingSites.ForBody(presetBodyName);
            if (presetBody != presetBodyName)
            {
                presetBody = presetBodyName;
                presetIndex = 0;
            }
            if (bodySites.Count > 0)
            {
                showLandingSites = GUILayout.Toggle(showLandingSites, Localizer.Format("#BoosterGuidance_LandingSites") + (showLandingSites ? " ▼" : " ▶"));
            }
            if ((bodySites.Count > 0) && showLandingSites)
            {
                presetIndex = Math.Min(presetIndex, bodySites.Count - 1);
                LandingSite site = bodySites[presetIndex];
                GUILayout.BeginHorizontal();
                if (GUILayout.Button("◄", GUILayout.Width(25)))
                    presetIndex = (presetIndex + bodySites.Count - 1) % bodySites.Count;
                GUILayout.Label(site.name + ((core.hotkeySiteName == site.name) ? " ★" : ""), GUILayout.MinWidth(60));
                if (GUILayout.Button("►", GUILayout.Width(25)))
                    presetIndex = (presetIndex + 1) % bodySites.Count;
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(Localizer.Format("#BoosterGuidance_ApplyPreset")))
                {
                    tgtLatitude = site.lat;
                    tgtLongitude = site.lon;
                    tgtAlt = (int)site.alt;
                    core.SetTarget(site.lat, site.lon, site.alt);
                    core.Changed();
                    tgtSet = true;
                    Targets.RedrawTarget(FlightGlobals.ActiveVessel.mainBody, site.lat, site.lon, site.alt);
                    GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_PresetApplied", site.name));
                }
                if (GUILayout.Button("★", GUILayout.Width(25)))
                {
                    // star = the site the guidance hotkey aims at; toggle off
                    // to let the hotkey auto-pick nearest-to-impact
                    if (core.hotkeySiteName == site.name)
                    {
                        core.hotkeySiteName = "";
                        GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_FavSiteCleared"));
                    }
                    else
                    {
                        core.hotkeySiteName = site.name;
                        GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_FavSiteSet", site.name));
                    }
                }
                if (!site.builtin)
                {
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_DeletePreset")))
                    {
                        string error;
                        if (!LandingSites.Delete(presetBodyName, site.name, out error))
                            GuiUtils.ScreenMessage(error);
                        presetIndex = 0;
                    }
                }
                GUILayout.EndHorizontal();
                if (tgtSet)
                {
                    GUILayout.BeginHorizontal();
                    presetName = GUILayout.TextField(presetName, GUILayout.MinWidth(80));
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_SavePreset")))
                    {
                        string error;
                        if (!LandingSites.Add(presetBodyName, presetName, tgtLatitude, tgtLongitude, tgtAlt, out error))
                            GuiUtils.ScreenMessage(error);
                        else
                            GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_PresetSaved", presetName));
                    }
                    GUILayout.EndHorizontal();
                }
            }

            if (tgtSet)
            {
                GUILayout.Label(Localizer.Format("#BoosterGuidance_OptionsSection"));
                GUILayout.BeginHorizontal();
                showTargets = GUILayout.Toggle(showTargets, Localizer.Format("#BoosterGuidance_ShowTargets"));

                bool prevLogging = core.logging;
                // TODO
                string filename = FlightGlobals.ActiveVessel.name;
                filename = filename.Replace(" ", "_");
                filename = filename.Replace("(", "");
                filename = filename.Replace(")", "");
                core.logFilename = filename;
                core.logging = GUILayout.Toggle(core.logging, Localizer.Format("#BoosterGuidance_Logging"));
                if (core.Enabled())
                {
                    if ((!prevLogging) && (core.logging)) // logging switched on
                        core.StartLogging();
                    if ((prevLogging) && (!core.logging)) // logging switched off
                        core.StopLogging();
                }
                GUILayout.EndHorizontal();

                // Falcon9-only knobs: RCS enable, action group and the
                // offset-thrust range limiter are booster-recovery features.
                // The starship profile manages RCS itself and never uses the
                // action-group trigger or the thrust-range cap - showing the
                // rows just crowded the fixed-height window (flight 49 UI
                // feedback)
                if (!starship)
                {
                GUILayout.BeginHorizontal();

                enableRCS = GUILayout.Toggle(enableRCS, "Enable RCS");
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();

                GUILayout.Label("Action Group:");
                GUILayout.TextField(actionGroup.ToString());
                if (GUILayout.Button("▲", GUILayout.Width(30)))
                    actionGroup++;
                if (GUILayout.Button("▼", GUILayout.Width(30)))
                    actionGroup--;
                actionGroup = Math.Max(0, Math.Min(10, actionGroup));
                GUILayout.EndHorizontal();
                // Hotkey (user request 2026-09-12): guidance key = one-key
                // starred-site target + enable/disable. No emergency key
                // (camera-tool conflict, f134) - the corner red button is the
                // only emergency trigger. KeyCode names (Backspace, Home, F10,
                // KeypadPlus, ...), empty = unbound. Written straight to the
                // persistent core field; the core re-parses on change
                GUILayout.BeginHorizontal();
                GUILayout.Label(Localizer.Format("#BoosterGuidance_HotkeyGuidance"));
                hotkeyGuidance = GUILayout.TextField(hotkeyGuidance, GUILayout.Width(80));
                core.hotkeyGuidance = hotkeyGuidance.Trim();
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                GUILayout.Label("Max Offset Thrust Range (" + degreeRange.ToString("F0") + ":");
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal();
                degreeRange = GUILayout.HorizontalSlider(degreeRange, 0, 180f);
                GUILayout.EndHorizontal();
                }

                // Info box
                GUILayout.BeginHorizontal();
                GUILayout.Label(core.Info());
                GUILayout.EndHorizontal();

                // Activate guidance
                SetEnabledColors(true); // back to normal
                GUILayout.BeginHorizontal();
                if (!core.Enabled())
                {
                    GUI.skin.button.normal.textColor = Color.green;
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_EnableGuidance"), GUILayout.Height(30)))
                    {
                        core.enableRCS = enableRCS;
                        core.actionGroup = actionGroup;
                        core.degreeRange = degreeRange;
                        core.EnableGuidance();
                    }
                }
                else
                {
                    GUI.skin.button.normal.textColor = Color.red;
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_DisableGuidance")))
                        core.DisableGuidance();
                }
                GUILayout.EndHorizontal();
                SetEnabledColors(true); // back to normal

                // (f129 UI cleanup: the emergency-land row is gone from the
                // flow - it is now the red round button pinned to the panel
                // corner, see DrawEmergencyButton; autoWarp moved into the
                // starship section below)
                // Starship-only section (correction gain and the
                // deorbit-scope readout land in later tasks)
                if (starship)
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(Localizer.Format("#BoosterGuidance_StarshipSection"));
                    GUILayout.EndHorizontal();
                    // f104 UI simplification (user: "现在UI面板东西太多了,
                    // 除了手动逆向顺向其他都不需要开放出来"): every tuning
                    // control (belly-axis arrows, gain slider, AoA boxes, the
                    // passive/TrajCal/violent-brake/aoa-mod/big-trim toggles)
                    // is HIDDEN. The persisted values keep flying - the
                    // defaults are all ON by design, and the user's save has
                    // them set from the f76-f103 tuning sessions. What stays
                    // on the panel: the Trajectories-missing warning (read-
                    // only), the dV fuel watch (read-only), and the manual
                    // RETRO/PROGRADE buttons
                    if (!TrajAPI.Available)
                        GUILayout.Label(Localizer.Format("#BoosterGuidance_NoTrajectories"));
                    // f96 fuel watch: keep the landing reserve visible so a
                    // long manual burn SEES itself eating the landing fuel
                    if (core.dvAvailable >= 0)
                    {
                        string dvTxt = string.Format(Localizer.Format("#BoosterGuidance_FuelWatch"), core.dvAvailable.ToString("F0"), core.landingReserveDv.ToString("F0"));
                        GUIStyle dvStyle = new GUIStyle(GUI.skin.label);
                        dvStyle.normal.textColor = (core.dvAvailable < core.landingReserveDv) ? Color.red : Color.white;
                        GUILayout.Label(dvTxt, dvStyle);
                    }
                    core.autoWarp = GUILayout.Toggle(core.autoWarp, Localizer.Format("#BoosterGuidance_AutoWarp"));
                    // Pilot burn attitude (f62 user redesign): pick the burn
                    // direction EXPLICITLY - the ship swings onto +/-vel_air
                    // the moment a button is on (already aligned when the
                    // pilot then adds throttle); OFF returns to the
                    // belly-forward guidance attitude on its own. The
                    // controller only honors this in the calm phases and
                    // never while guidance itself is burning
                    GUILayout.BeginHorizontal();
                    bool wantRetro = GUILayout.Toggle(core.pilotAttitudeMode < 0, Localizer.Format("#BoosterGuidance_RetroBurn"));
                    bool wantPro = GUILayout.Toggle(core.pilotAttitudeMode > 0, Localizer.Format("#BoosterGuidance_ProgradeBurn"));
                    if (wantRetro != (core.pilotAttitudeMode < 0))
                        core.pilotAttitudeMode = wantRetro ? -1 : 0;
                    if (wantPro != (core.pilotAttitudeMode > 0))
                        core.pilotAttitudeMode = wantPro ? 1 : 0;
                    GUILayout.EndHorizontal();
                }
                // Manual phase buttons are falcon9-only: the starship
                // profile auto-picks its entry phase at enable (design D2)
                if (!starship)
                {
                GUILayout.Label(Localizer.Format("#BoosterGuidance_PhasesSection"));
                // Turn Around (RCS flip to face horizontal retrograde before boostback)
                SetEnabledColors((phase == BLControllerPhase.TurnAround) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent(Localizer.Format("#BoosterGuidance_TurnAround"), "Use RCS to rotate the vessel to face opposite the horizontal velocity, then start Boostback")))
                    EnableGuidance(BLControllerPhase.TurnAround);
                GUILayout.EndHorizontal();

                // Boostback
                SetEnabledColors((phase == BLControllerPhase.BoostBack) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent(Localizer.Format("#BoosterGuidance_Boostback"), "Enable thrust towards target when out of atmosphere")))
                    EnableGuidance(BLControllerPhase.BoostBack);
                GUILayout.EndHorizontal();

                // Coasting
                SetEnabledColors((phase == BLControllerPhase.Coasting) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent(Localizer.Format("#BoosterGuidance_Coasting"), "Turn to retrograde attitude and wait for Aero Descent phase")))
                    EnableGuidance(BLControllerPhase.Coasting);
                GUILayout.EndHorizontal();

                // Re-Entry Burn
                SetEnabledColors((phase == BLControllerPhase.ReentryBurn) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent(Localizer.Format("#BoosterGuidance_ReentryBurn"), "Ignite engine on re-entry to reduce overheating")))
                    EnableGuidance(BLControllerPhase.ReentryBurn);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_EnableAltitude"), reentryBurnAlt, "m", 65);
                core.reentryBurnAlt = reentryBurnAlt;
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GuiUtils.SimpleTextBox(Localizer.Format("#BoosterGuidance_HeavyBrakeDepth"), heavyBrakeDepthPct, "%", 40);
                // f150 (user-approved 放宽到900): range widened to -50..100 so
                // the target speed spans 900..300 m/s; 0 keeps the 700 m/s
                // default. The resulting speed is shown so the entry is
                // unambiguous
                heavyBrakeDepthPct = Mathf.Clamp((int)heavyBrakeDepthPct, -50, 100);
                core.reentryBurnTargetSpeed = 700 - 4 * (int)heavyBrakeDepthPct;
                GUILayout.Label("-> " + (int)core.reentryBurnTargetSpeed + " m/s", GUILayout.Width(75));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Steer", GUILayout.Width(40));
                core.reentryBurnSteerKp = Mathf.Clamp(core.reentryBurnSteerKp, 0, maxReentryGain);
                core.reentryBurnSteerKp = GUILayout.HorizontalSlider(core.reentryBurnSteerKp, 0, maxReentryGain);
                GUILayout.Label(((int)(core.reentryBurnMaxAoA)).ToString() + "°(max)", GUILayout.Width(60));
                GUILayout.EndHorizontal();

                // Aero Descent
                SetEnabledColors((phase == BLControllerPhase.AeroDescent) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent(Localizer.Format("#BoosterGuidance_AeroDescent"), "No thrust aerodynamic descent, steering with gridfins within atmosphere")))
                    EnableGuidance(BLControllerPhase.AeroDescent);
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Steer", GUILayout.Width(40));
                core.aeroDescentSteerKp = Mathf.Clamp(core.aeroDescentSteerKp, 0, maxAeroDescentGain);
                core.aeroDescentSteerKp = GUILayout.HorizontalSlider(core.aeroDescentSteerKp, 0, maxAeroDescentGain); // max turn 2 degrees for 100m error
                GUILayout.Label(((int)core.aeroDescentMaxAoA).ToString() + "°(max)", GUILayout.Width(60));
                GUILayout.EndHorizontal();
                } // end falcon9-only phase buttons

                // Landing Burn (shared with the starship profile; the
                // manual phase button stays falcon9-only)
                if (!starship)
                {
                SetEnabledColors((phase == BLControllerPhase.LandingBurn) || (phase == BLControllerPhase.Unset));
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(Localizer.Format("#BoosterGuidance_LandingBurn")))
                    EnableGuidance(BLControllerPhase.LandingBurn);
                GUILayout.EndHorizontal();
                }

                GUILayout.BeginHorizontal();
                GUILayout.Label(Localizer.Format("#BoosterGuidance_EnableAltitude"));
                String text = "n/a";
                if (core.Enabled())
                {
                    if (core.LandingBurnHeight() > 0)
                        text = ((int)(core.LandingBurnHeight() + tgtAlt)).ToString() + "m";
                    else
                    {
                        if (core.LandingBurnHeight() < 0)
                            text = Localizer.Format("#BoosterGuidance_TooHeavy");
                    }
                }
                GUILayout.Label(text, GUILayout.Width(60));
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label(Localizer.Format("#BoosterGuidance_Engines"));
                if (numLandingBurnEngines == Localizer.Format("#BoosterGuidance_Current"))
                    GUILayout.Label(numLandingBurnEngines);
                else
                    GUILayout.Label(numLandingBurnEngines);
                if (numLandingBurnEngines == "current")  // Save active engines
                {
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_Set")))  // Set to currently active engines
                        numLandingBurnEngines = core.SetLandingBurnEngines();
                }
                else
                {
                    if (GUILayout.Button(Localizer.Format("#BoosterGuidance_Unset")))  // Set to currently active engines
                        numLandingBurnEngines = core.UnsetLandingBurnEngines();
                }
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Steer", GUILayout.Width(40));
                core.landingBurnSteerKp = Mathf.Clamp(core.landingBurnSteerKp, 0, maxLandingBurnGain);
                core.landingBurnSteerKp = GUILayout.HorizontalSlider(core.landingBurnSteerKp, 0, maxLandingBurnGain);
                string max = Localizer.Format("#BoosterGuidance_Max");
                GUILayout.Label(((int)(core.landingBurnMaxAoA)).ToString() + "°(" + max + ")", GUILayout.Width(60));
                GUILayout.EndHorizontal();

            }
            GUI.DragWindow();
            return (GUI.changed) || targetChanged;
        }

        public void Show()
        {
            hidden = false;
            tab = 0; // Switch to main tab incase Advanced tab got broken
        }

        public void Hide()
        {
            hidden = true;
            Targets.targetingCross.enabled = false;
            // Flight 49: do NOT hide the prediction cross with the window -
            // the user drags maneuver nodes with the window closed and needs
            // the MJ-style always-on impact cross + trajectory line. The
            // scope keeps refreshing (and re-enabling) it at 1 Hz anyway
        }

        // Core changed - update window
        public void UpdateFromCore()
        {
            reentryBurnAlt = (int)core.reentryBurnAlt;
            heavyBrakeDepthPct = (int)Mathf.Clamp((700 - (int)core.reentryBurnTargetSpeed) / 4, -50, 100);
            reentryBurnAlt = (int)core.reentryBurnAlt;
            heavyBrakeDepthPct = (int)Mathf.Clamp((700 - (int)core.reentryBurnTargetSpeed) / 4, -50, 100);
            tgtLatitude = core.tgtLatitude;
            tgtLongitude = core.tgtLongitude;
            tgtAlt = (int)core.tgtAlt;
            this.tgtSet = core.tgtSet;
            hotkeyGuidance = core.hotkeyGuidance;

            // This is bit-field
            numLandingBurnEngines = core.landingBurnEngines;
            igniteDelay = (int)core.igniteDelay;
            noSteerHeight = (int)core.noSteerHeight;
            uprightHeight = core.uprightHeight;
            uprightMaxHorizSpeed = core.uprightMaxHorizSpeed;
            steerDamping = core.steerDamping;
            deployLandingGear = core.deployLandingGear;
            deployLandingGearHeight = (int)core.deployLandingGearHeight;
            Targets.RedrawTarget(FlightGlobals.ActiveVessel.mainBody, tgtLatitude, tgtLongitude, tgtAlt);

            // Apply limits
            core.reentryBurnSteerKp = Mathf.Clamp(core.reentryBurnSteerKp, 0, maxReentryGain);
            core.aeroDescentSteerKp = Mathf.Clamp(core.aeroDescentSteerKp, 0, maxAeroDescentGain);
            core.landingBurnSteerKp = Mathf.Clamp(core.landingBurnSteerKp, 0, maxLandingBurnGain);

            // Set MaxAoA from core - overriden
            core.reentryBurnMaxAoA = maxSteerAngle * (core.reentryBurnSteerKp / maxReentryGain);
            core.aeroDescentMaxAoA = maxSteerAngle * (core.aeroDescentSteerKp / maxAeroDescentGain);
            core.landingBurnMaxAoA = maxSteerAngle * (core.landingBurnSteerKp / maxLandingBurnGain);
        }

        public void UpdateCore()
        {
            core.debug = debug;
            core.SetTarget(tgtLatitude, tgtLongitude, tgtAlt);
            // Set Angle - of - Attack from gains
            core.reentryBurnMaxAoA = maxSteerAngle * (core.reentryBurnSteerKp / maxReentryGain);
            core.aeroDescentMaxAoA = maxSteerAngle * (core.aeroDescentSteerKp / maxAeroDescentGain);
            core.landingBurnMaxAoA = maxSteerAngle * (core.landingBurnSteerKp / maxLandingBurnGain);
            // Other
            core.touchdownMargin = touchdownMargin;
            core.touchdownSpeed = (float)touchdownSpeed;
            core.noSteerHeight = noSteerHeight;
            core.uprightHeight = uprightHeight;
            core.uprightMaxHorizSpeed = (float)uprightMaxHorizSpeed;
            core.steerDamping = steerDamping;
            core.deployLandingGear = deployLandingGear;
            core.deployLandingGearHeight = deployLandingGearHeight;
            core.Changed();
            Targets.RedrawTarget(FlightGlobals.ActiveVessel.mainBody, tgtLatitude, tgtLongitude, tgtAlt);
        }

        void OnPickingPositionTarget()
        {
            if (GuiUtils.MouseIsOverWindow(windowRect))
                return;

            if (Input.GetKeyDown(KeyCode.Escape))
            {
                // Previous position
                Targets.RedrawTarget(FlightGlobals.ActiveVessel.mainBody, tgtLatitude, tgtLongitude, tgtAlt);
                pickingPositionTarget = false;
            }
            RaycastHit hit;
            Vessel vessel = FlightGlobals.ActiveVessel;
            bool isHit = false;

            if (!MapView.MapIsEnabled)
            {
                if (GuiUtils.GetMouseHit(vessel.mainBody, windowRect, MapView.MapIsEnabled, out hit))
                {
                    isHit = true;
                    // Moved or picked
                    vessel.mainBody.GetLatLonAlt(hit.point, out pickLat, out pickLon, out pickAlt);
                }
            }
            if (!isHit)
            {
                if (GuiUtils.GetBodyRayIntersect(vessel.mainBody, MapView.MapIsEnabled, out pickLat, out pickLon, out pickAlt))
                    isHit = true;
            }


            if (isHit)
            {
                Targets.RedrawTarget(vessel.mainBody, pickLat, pickLon, pickAlt);
                if ((Input.GetMouseButton(0)) && (!GuiUtils.MouseIsOverWindow(windowRect))) // Picked
                {
                    // Update GUI
                    tgtLatitude = pickLat;
                    tgtLongitude = pickLon;
                    tgtAlt = (int)pickAlt;
                    pickingPositionTarget = false;
                    core.SetTarget(pickLat, pickLon, pickAlt);
                    core.Changed();
                    tgtSet = true;

                }
            }
        }

        void OnUpdate()
        {
            // Set visibility of targets
            Targets.InitTargets(); // ensure updated with map switch
            Targets.SetVisibility(showTargets, core.Enabled() && showTargets);
            if (pickingPositionTarget)
                OnPickingPositionTarget();
        }


        void PickTarget()
        {
            showTargets = true;
            pickingPositionTarget = true;
            GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_ClickToPickTarget"));
        }


        void SetTargetHere()
        {
            Vessel vessel = FlightGlobals.ActiveVessel;
            if (vessel)
            {
                FlightGlobals.ActiveVessel.mainBody.GetLatLonAlt(vessel.GetWorldPos3D(), out double pickLat, out double pickLon, out double pickAlt);
                double lowestY = KSPUtils.FindLowestPointOnVessel(FlightGlobals.ActiveVessel);
                tgtLatitude = pickLat;
                tgtLongitude = pickLon;
                tgtAlt = (int)(pickAlt + lowestY);
                core.SetTarget(tgtLatitude, tgtLongitude, tgtAlt);
                core.Changed();
                GuiUtils.ScreenMessage(Localizer.Format("#BoosterGuidance_TargetSetToVessel"));
                tgtSet = true;
            }
        }

        void EnableGuidance(BLControllerPhase phase)
        {
            BoosterGuidanceCore core = BoosterGuidanceCore.GetBoosterGuidanceCore(FlightGlobals.ActiveVessel);
            KSPActionParam param = new KSPActionParam(KSPActionGroup.None, KSPActionType.Activate);
            core.useFAR = hasFAR;

            core.enableRCS = enableRCS;
            core.actionGroup = actionGroup;
            core.degreeRange = degreeRange;

            Log.Info("Vessel=" + FlightGlobals.ActiveVessel.name + " useFAR=" + core.useFAR);
            core.EnableGuidance(param);
            core.SetPhase(phase);
        }

        void DisableGuidance()
        {
            BoosterGuidanceCore core = BoosterGuidanceCore.GetBoosterGuidanceCore(FlightGlobals.ActiveVessel);
            core.DisableGuidance();
        }
    }
}
