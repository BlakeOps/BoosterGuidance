// Utility functions that depend on KSP

using System;
using System.Collections.Generic;
using UnityEngine;
using KSP;
using ModuleWheels;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
  public class KSPUtils
  {
    // Find Y offset to lowest part from origin of the vessel
    public static double FindLowestPointOnVessel(Vessel vessel)
    {
      Vector3 CoM, up;

      CoM = vessel.localCoM;
      Vector3 bottom = Vector3.zero; // Offset from CoM
      up = FlightGlobals.getUpAxis(CoM); //Gets up axis
      Vector3 pos = vessel.GetWorldPos3D();
      Vector3 distant = pos - 1000 * up; // distant below craft
      double miny = 0;
      foreach (Part p in vessel.parts)
      {
        if (p.collider != null) //Makes sure the part actually has a collider to touch ground
        {
          Vector3 pbottom = p.collider.ClosestPointOnBounds(distant); //Gets the bottom point
          double y = Vector3.Dot(up, pbottom - pos); // relative to centre of vessel
          if (y < miny)
          {
            bottom = pbottom;
            miny = y;
          }
        }
      }
      return miny;
    }

    public static List<ModuleEngines> GetActiveEngines(Vessel vessel)
    {
      List<ModuleEngines> activeEngines = new List<ModuleEngines>();
      foreach (Part part in vessel.parts)
      {
        part.isEngine(out List<ModuleEngines> engines);
        foreach (ModuleEngines engine in engines)
        {
          if (engine.isOperational)
            activeEngines.Add(engine);
        }
      }
      return activeEngines;
    }

    public static void SetActiveEngines(Vessel vessel, List<ModuleEngines> active)
    {
      foreach(var engine in KSPUtils.GetAllEngines(vessel))
      {
        if (active.Contains(engine))
        {
          if (!engine.isOperational)
            engine.Activate();
        }
        else
        {
          if (engine.isOperational)
            engine.Shutdown();
        }
      }
    }

    // True pointing/thrust axis of the vessel: max-thrust-weighted average of
    // engine thrust-transform axes. Neither vessel.transform.up (root part) nor
    // ReferenceTransform.up (control point) is guaranteed to lie along the vessel
    // for craft with a sideways root part (e.g. a radially-mounted decoupler as
    // root), but the engines always push along their thrust transforms.
    public static Vector3d GetThrustAxis(Vessel vessel, List<ModuleEngines> useEngines = null)
    {
      if (useEngines == null)
        useEngines = GetAllEngines(vessel);
      Vector3d axis = Vector3d.zero;
      foreach (ModuleEngines engine in useEngines)
      {
        // weight by maxThrust so small aux engines (sepratrons etc.) don't skew the axis
        double w = Math.Max(1e-3, engine.maxThrust);
        foreach (Transform t in engine.thrustTransforms)
          axis -= w * (Vector3d)t.forward; // thrust pushes the vessel along -forward
      }
      Transform vt = vessel.GetTransform();
      if (axis.sqrMagnitude < 1e-6)
        return (vt != null) ? (Vector3d)vt.up : (Vector3d)vessel.transform.up;
      axis.Normalize();
      // Sanity check against the transform getOffsetFromHeading steers by (proven
      // to be the effective pointing axis in flight): if the engine axis points
      // the opposite way, trust GetTransform instead
      if ((vt != null) && (Vector3d.Dot(axis, vt.up) < 0))
        axis = (Vector3d)vt.up;
      return axis;
    }

    // Thrust-weighted average Isp (s) of the given engines (all operational engines
    // if not specified), used to convert simulated thrust into propellant usage
    public static double GetAverageIsp(Vessel vessel, List<ModuleEngines> useEngines = null)
    {
      if (useEngines == null)
        useEngines = GetOperationalEngines(vessel);
      double sumT = 0, sumToverI = 0; // Isp_avg = sum(T) / sum(T/Isp)
      foreach (ModuleEngines engine in useEngines)
      {
        double mt = Math.Max(1e-3, engine.maxThrust);
        double isp = (engine.realIsp > 0) ? engine.realIsp : 280; // same guess as GetEngineMinMaxThrust
        sumT += mt;
        sumToverI += mt / isp;
      }
      return (sumToverI > 0) ? sumT / sumToverI : 280;
    }

    // Propellant mass (kg) usable by the given engines (all operational engines if
    // not specified), limited by the scarcest propellant of the mixture. IntakeAir
    // and other free/flowless propellants are ignored.
    public static double ComputeUsablePropellantKg(Vessel vessel, List<ModuleEngines> useEngines = null)
    {
      if (useEngines == null)
        useEngines = GetOperationalEngines(vessel);
      if (useEngines.Count == 0)
        return 0;
      // Mixture ratios from the first engine - assume all engines burn the same mix
      ModuleEngines refEngine = useEngines[0];
      double ratioSum = 0;
      double minMixtureUnits = double.MaxValue;
      foreach (Propellant prop in refEngine.propellants)
      {
        if ((prop.ratio <= 0) || (prop.name == "IntakeAir"))
          continue;
        PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(prop.name);
        if ((def == null) || (def.density <= 0))
          continue;
        double units = 0;
        foreach (Part part in vessel.parts)
        {
          foreach (PartResource res in part.Resources)
          {
            if ((res.resourceName == prop.name) && (res.flowState))
              units += res.amount;
          }
        }
        ratioSum += prop.ratio;
        minMixtureUnits = Math.Min(minMixtureUnits, units / prop.ratio);
      }
      if ((minMixtureUnits == double.MaxValue) || (ratioSum <= 0))
        return 0;
      // Mass of the full mixture at the limiting propellant's availability
      double kg = 0;
      foreach (Propellant prop in refEngine.propellants)
      {
        if ((prop.ratio <= 0) || (prop.name == "IntakeAir"))
          continue;
        PartResourceDefinition def = PartResourceLibrary.Instance.GetDefinition(prop.name);
        if ((def == null) || (def.density <= 0))
          continue;
        kg += minMixtureUnits * prop.ratio * def.density * 1000; // density is t/unit
      }
      return kg;
    }

    public static List<ModuleEngines> GetAllEngines(Vessel vessel)
    {
      List<ModuleEngines> engines = new List<ModuleEngines>();
      foreach (Part part in vessel.parts)
      {
        part.isEngine(out List<ModuleEngines> partEngines);
        foreach (ModuleEngines engine in partEngines)
          engines.Add(engine);
      }
      return engines;
    }

    public static List<ModuleEngines> GetOperationalEngines(Vessel vessel)
    {
      List<ModuleEngines> opEngines = new List<ModuleEngines>();
      foreach (Part part in vessel.parts)
      {
        part.isEngine(out List<ModuleEngines> partEngines);
        foreach (ModuleEngines engine in partEngines)
          if (engine.isOperational)
            opEngines.Add(engine);
      }
      return opEngines;
    }

    private static void GetEngineMinMaxThrust(ModuleEngines engine, out double minThrust, out double maxThrust, bool log=false)
    {
      float isp = (engine.realIsp > 0) ? engine.realIsp : 280; // guess!
      float pressure = (float)FlightGlobals.getStaticPressure() * 0.01f; // so 1.0 at Kerbin sea level?
      float atmMaxThrust = engine.MaxThrustOutputAtm(true, true, pressure, FlightGlobals.getExternalTemperature());
      minThrust = engine.GetEngineThrust(isp, 0); // can't get atmMinThrust (this ignore throttle limiting but thats ok)
      maxThrust = atmMaxThrust; // this uses throttle limiting and should give vac thrust as pressure/temp specified too
      if (log)
        Log.Info("GetEngineMinMaxThrust:  engine=" + engine + " isp=" + isp + " MinThrust=" + engine.GetEngineThrust(isp, 0) + " MaxThrust=" + atmMaxThrust + " operational=" + engine.isOperational);
    }

    public static List<ModuleEngines> ComputeMinMaxThrust(Vessel vessel, out double totalMinThrust, out double totalMaxThrust, bool log = false, List<ModuleEngines> useEngines = null)
    {
      totalMinThrust = 0;
      totalMaxThrust = 0;

      // If no engines specified find all operational engines
      if (useEngines == null)
        useEngines = GetOperationalEngines(vessel);

      foreach(ModuleEngines engine in useEngines)
      {
        double minThrust, maxThrust;
        GetEngineMinMaxThrust(engine, out minThrust, out maxThrust);
        totalMinThrust += minThrust;
        totalMaxThrust += maxThrust;
      }
      return useEngines;
    }

    public static double GetCurrentThrust(List<ModuleEngines> allEngines)
    {
      double thrust = 0;
      foreach (ModuleEngines engine in allEngines)
        thrust += engine.GetCurrentThrust();
      return thrust;
    }

    public static double MinHeightAtMinThrust(double y, double vy, double amin, double g)
    {
      double minHeight = 0;
      if (amin < g)
        return -float.MaxValue;
      double tHover = -vy / amin; // time to come to hover
      minHeight = y + vy * tHover + 0.5 * amin * tHover * tHover - 0.5 * g * tHover * tHover;
      return minHeight;
    }

    static int Closest(KeyValuePair<double, ModuleEngines> a, KeyValuePair<double, ModuleEngines> b)
    {
      return a.Key.CompareTo(b.Key);
    }

    // Compute engine thrust if one set of symmetrical engines is shutdown
    // (primarily for a Falcon 9 landing to shutdown engines for slow touchdown)
    public static List<ModuleEngines> ShutdownOuterEngines(Vessel vessel, float desiredThrust, bool log = false)
    {
      List<ModuleEngines> shutdown = new List<ModuleEngines>();
      Log.Info("ShutdownOuterEngines desiredThrust=" + desiredThrust + " mass=" + vessel.totalMass);
      // Find engine parts and sort by closest to centre first
      List<KeyValuePair<double, ModuleEngines>> allEngines = new List<KeyValuePair<double, ModuleEngines>>();
      foreach (Part part in vessel.GetActiveParts())
      {
        Vector3 relpos = vessel.transform.InverseTransformPoint(part.transform.position);
        part.isEngine(out List<ModuleEngines> engines);
        double dist = Math.Sqrt(relpos.x * relpos.x + relpos.z * relpos.z);
        foreach (ModuleEngines engine in engines)
          allEngines.Add(new KeyValuePair<double, ModuleEngines>(dist, engine));
      }
      allEngines.Sort(Closest);

      // Loop through engines starting a closest to axis
      // Accumulate minThrust, once minThrust exceeds desiredThrust shutdown this and all
      // further out engines
      float minThrust = 0, maxThrust = 0;
      double shutdownDist = float.MaxValue;
      foreach (var engDist in allEngines)
      {
        ModuleEngines engine = engDist.Value;
        if (engine.isOperational)
        {
          minThrust += engine.GetEngineThrust(engine.realIsp, 0);
          maxThrust += engine.GetEngineThrust(engine.realIsp, 1);
          if (shutdownDist == float.MaxValue)
          {
            if ((minThrust < desiredThrust) && (desiredThrust < maxThrust)) // good amount of thrust
              shutdownDist = engDist.Key + 0.1f;
            if (minThrust > desiredThrust)
              shutdownDist = engDist.Key - 0.1f;
          }

          if (engDist.Key > shutdownDist)
          {
            if (log)
              Log.Info("ComputeShutdownMinMaxThrust(): minThrust=" + minThrust + " desiredThrust=" + desiredThrust + " SHUTDOWN");
            engine.Shutdown();
            shutdown.Add(engine);
          }
          else
            if (log)
              Log.Info("ComputeShutdownMinMaxThrust(): minThrust=" + minThrust + " desiredThrust=" + desiredThrust + " KEEP");
        }
      }
      Log.Info(shutdown.Count + " engines shutdown");
      return shutdown;
    }

    public static bool DeployLandingGear(Vessel vessel)
    {
      vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
      return true;
    }

    // Last-resort backup (user request after flight 51): force-deploy every
    // stowed stock parachute on the vessel. Callers gate on speed/altitude -
    // stock chutes shred above ~300 m/s regardless
    public static int DeployParachutes(Vessel vessel)
    {
      int n = 0;
      foreach (Part p in vessel.parts)
      {
        foreach (ModuleParachute chute in p.FindModulesImplementing<ModuleParachute>())
        {
          if (chute.deploymentState == ModuleParachute.deploymentStates.STOWED)
          {
            chute.Deploy();
            n++;
          }
        }
      }
      return n;
    }
  }
}
