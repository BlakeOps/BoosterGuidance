using System;
using System.Collections.Generic;
using UnityEngine;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    public class LandingSite
    {
        public string name;
        public string body;
        public double lat;
        public double lon;
        public double alt;
        public bool builtin;

        public ConfigNode Save()
        {
            ConfigNode node = new ConfigNode("LANDING_SITE");
            node.AddValue("name", name);
            node.AddValue("body", body);
            node.AddValue("lat", lat.ToString("R"));
            node.AddValue("lon", lon.ToString("R"));
            node.AddValue("alt", alt.ToString("R"));
            if (builtin)
                node.AddValue("builtin", "true");
            return node;
        }

        public static LandingSite Load(ConfigNode node)
        {
            LandingSite site = new LandingSite();
            site.name = node.GetValue("name");
            site.body = node.GetValue("body");
            if (String.IsNullOrEmpty(site.name) || String.IsNullOrEmpty(site.body))
                return null;
            if (!double.TryParse(node.GetValue("lat"), out site.lat))
                return null;
            if (!double.TryParse(node.GetValue("lon"), out site.lon))
                return null;
            if (!double.TryParse(node.GetValue("alt"), out site.alt))
                return null;
            bool b;
            site.builtin = bool.TryParse(node.GetValue("builtin"), out b) && b;
            return site;
        }
    }

    // Global, cross-save library of named landing sites.
    // Stored in GameData/BoosterGuidance/PluginData/landing_sites.cfg so presets
    // are shared by all vessels and all saves (design D1).
    public static class LandingSites
    {
        private static List<LandingSite> sites = null;

        public static string ConfigPath
        {
            get { return KSPUtil.ApplicationRootPath + "GameData/BoosterGuidance/PluginData/landing_sites.cfg"; }
        }

        private static void Info(string msg)
        {
            if (Log != null)
                Log.Info(msg);
            else
                Debug.Log("[BoosterGuidance] " + msg);
        }

        private static void Warn(string msg)
        {
            Debug.LogWarning("[BoosterGuidance] " + msg);
        }

        private static List<LandingSite> Defaults()
        {
            List<LandingSite> list = new List<LandingSite>();
            // KSC launch pad (centre of the pad)
            list.Add(new LandingSite { name = "KSC Launch Pad", body = "Kerbin", lat = -0.09721, lon = -74.55766, alt = 67, builtin = true });
            // KSC runway thresholds (09 = east end, 27 = west end)
            list.Add(new LandingSite { name = "KSC Runway 09", body = "Kerbin", lat = -0.04860, lon = -74.69540, alt = 68, builtin = true });
            list.Add(new LandingSite { name = "KSC Runway 27", body = "Kerbin", lat = -0.04860, lon = -74.73780, alt = 68, builtin = true });
            return list;
        }

        private static void EnsureLoaded()
        {
            if (sites != null)
                return;
            sites = new List<LandingSite>();
            bool ok = false;
            try
            {
                ConfigNode root = ConfigNode.Load(ConfigPath);
                ConfigNode parent = (root != null) ? root.GetNode("BOOSTERGUIDANCE_LANDING_SITES") : null;
                if (parent != null)
                {
                    foreach (ConfigNode node in parent.GetNodes("LANDING_SITE"))
                    {
                        LandingSite site = LandingSite.Load(node);
                        if (site != null)
                            sites.Add(site);
                        else
                            Warn("Skipping invalid LANDING_SITE entry in " + ConfigPath);
                    }
                    ok = sites.Count > 0;
                }
            }
            catch (Exception e)
            {
                Warn("Failed to load landing sites: " + e.Message);
            }
            if (!ok)
            {
                if (System.IO.File.Exists(ConfigPath))
                    Warn("Landing sites file missing or corrupt, regenerating defaults: " + ConfigPath);
                sites = Defaults();
                SaveAll();
            }
            Info("Loaded " + sites.Count + " landing site(s)");
        }

        private static void SaveAll()
        {
            try
            {
                ConfigNode parent = new ConfigNode("BOOSTERGUIDANCE_LANDING_SITES");
                foreach (LandingSite site in sites)
                    parent.AddNode(site.Save());
                ConfigNode root = new ConfigNode();
                root.AddNode(parent);
                root.Save(ConfigPath, "BoosterGuidance landing site presets");
            }
            catch (Exception e)
            {
                Warn("Failed to save landing sites: " + e.Message);
            }
        }

        // Presets applicable at the vessel's current body only (spec: body-filtered list)
        public static List<LandingSite> ForBody(string body)
        {
            EnsureLoaded();
            List<LandingSite> list = new List<LandingSite>();
            foreach (LandingSite site in sites)
            {
                if (site.body == body)
                    list.Add(site);
            }
            return list;
        }

        public static LandingSite Find(string body, string name)
        {
            EnsureLoaded();
            foreach (LandingSite site in sites)
            {
                if ((site.body == body) && (site.name == name))
                    return site;
            }
            return null;
        }

        // Nearest site on the body to a lat/lon (deg). Equirectangular
        // distance is enough - this ranks, it does not measure. Returns null
        // when the body has no sites
        public static LandingSite Nearest(string body, double lat, double lon)
        {
            EnsureLoaded();
            LandingSite best = null;
            double bestD = double.MaxValue;
            double cosLat = Math.Cos(lat * Math.PI / 180.0);
            foreach (LandingSite site in sites)
            {
                if (site.body != body)
                    continue;
                double dLat = site.lat - lat;
                double dLon = site.lon - lon;
                if (dLon > 180) dLon -= 360;
                if (dLon < -180) dLon += 360;
                double d = dLat * dLat + cosLat * cosLat * dLon * dLon;
                if (d < bestD)
                {
                    bestD = d;
                    best = site;
                }
            }
            return best;
        }

        // Returns false with an error message on duplicate name (spec: save current target)
        public static bool Add(string body, string name, double lat, double lon, double alt, out string error)
        {
            EnsureLoaded();
            error = null;
            if (String.IsNullOrEmpty(name))
            {
                error = "Preset name is empty";
                return false;
            }
            if (Find(body, name) != null)
            {
                error = "A preset named '" + name + "' already exists for " + body;
                return false;
            }
            sites.Add(new LandingSite { name = name, body = body, lat = lat, lon = lon, alt = alt, builtin = false });
            SaveAll();
            Info("Added landing site '" + name + "' on " + body);
            return true;
        }

        // Built-in presets cannot be deleted (spec: default presets)
        public static bool Delete(string body, string name, out string error)
        {
            EnsureLoaded();
            error = null;
            LandingSite site = Find(body, name);
            if (site == null)
            {
                error = "No preset named '" + name + "' for " + body;
                return false;
            }
            if (site.builtin)
            {
                error = "Built-in presets cannot be deleted";
                return false;
            }
            sites.Remove(site);
            SaveAll();
            Info("Deleted landing site '" + name + "' on " + body);
            return true;
        }
    }
}
