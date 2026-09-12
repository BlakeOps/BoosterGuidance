using System;
using System.Collections.Generic;
using UnityEngine;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    public class Simulate
    {
        static float dt_space = 32;
        static float dt_reentry = 2; // used before reentry
        static float dt_aero = 4;

        double MinHeightAtMinThrust(double y, double vy, double amin, double g)
        {
            double minHeight = 0;
            if (amin < g)
                return -float.MaxValue;
            double tHover = -vy / amin; // time to come to hover
            minHeight = y + vy * tHover + 0.5 * amin * tHover * tHover - 0.5 * g * tHover * tHover;
            return minHeight;
        }

        static bool Hit(CelestialBody body, Vector3d r)
        {
            return (r - body.position).magnitude < body.Radius;
        }

        static private void EulerStep(
                double dt,
                Vessel vessel, Vector3d r, // in world space but relative to body.position
                Vector3d v, Vector3d att, double totalMass, double minThrust, double maxThrust,
                Trajectories.VesselAerodynamicModel aeroModel, CelestialBody body, double t,
                BLController controller, Vector3d tgt_r, double aeroFudgeFactor,
                out Vector3d steer,
                out Vector3d vel_air, out double throttle,
                out Vector3d out_r, out Vector3d out_v)
        {
            double y = r.magnitude - body.Radius;
            steer = -Vector3d.Normalize(v);
            throttle = 0;

            // gravity
            double R = r.magnitude;
            Vector3d g = r * (-body.gravParameter / (R * R * R));

            // Get steer and throttle
            bool bailOutLandingBurn = true;
            if (controller != null)
            {
                bool landingGear;
                controller.GetControlOutputs(vessel, totalMass, r, v, att, minThrust, maxThrust, t, body, true, out throttle, out steer, out landingGear, bailOutLandingBurn);
                // Stop throttle so we don't take off again in timestep, dt
                // TODO - Fix HACK!!
                if (y < controller.TgtAlt + 50)
                    throttle = 0;
            }

            Vector3d Ft = Vector3d.zero;
            if (throttle > 0)
                Ft = steer * (minThrust + throttle * (maxThrust - minThrust));

            // TODO: Do repeated calls to GetForces() mess up PID controllers which updates their internal estimates?
            vel_air = v - body.getRFrmVel(r + body.position);
            if (aeroModel == null)
            {
                Log.Info("EulerStep() - No aeroModel");
            }
            // Aerodynamic attitude: falcon9 flies retrograde (AoA = PI); the
            // starship belly flop follows the q-scheduled AoA (85 deg brake in
            // thin air, 18 deg glide at high q - flight 46 profile). The sim
            // ignores the <=15 deg steering tilt - only the broadside force
            // level matters for the trajectory
            double aoa = Math.PI;
            if ((controller != null) && (controller.recoveryProfile == "starship") && (controller.phase == BLControllerPhase.BellyFlop))
            {
                double qSim = BLController.DynamicPressure(r.magnitude - body.Radius, vel_air.magnitude, body);
                // Trim flip (f46 proved, f51 confirmed): above
                // bellyTrimFlipQ the hull's tail-first trim beats the flaps
                // and the glide is lost. f61 lesson: a fixed q latch is wrong
                // for EVERY craft eventually (cargo 8.8 kPa, EnginePlate3
                // held past 28 kPa after optimization) and a too-low latch
                // poisons the whole flight (f57/f61: sim dove ballistic while
                // the real ship glided on -> correction chased the short
                // fantasy and physically flew the ship hundreds of km long).
                // The latch is now LIVE-ONLY: att_err > 100 deg for 5 s in
                // BLController latches glideLost and the copy ctor carries it
                // into every subsequent sim. A craft that has not (yet)
                // flipped gets the optimistic glide prediction - the
                // self-correcting side of the asymmetry
                aoa = ((controller.glideLost) || (controller.bellyBraking))
                    ? Math.PI
                    : controller.BellyAoADeg(qSim, Vector3d.Exclude(Vector3d.Normalize(r), vel_air).magnitude) * Math.PI / 180.0;
            }
            Vector3d Fa = aeroModel.GetForces(body, r, vel_air, aoa);
            // Live-calibrated aero scaling (flight 59): the cache's 18-deg
            // glide lift is ~half what the real flap-flown hull realizes, so
            // the sim dove at 2.3x the real sink rate below 10km and read
            // every low-altitude prediction ~20km short of the truth (the f59
            // overshoot the brake could not see). BLController measures the
            // realized lift/drag ratios in the glide regime (aeroCalLift/
            // aeroCalDrag); apply them through the same band the measurement
            // came from, ramped out across 30-50 deg so the verified 85-deg
            // brake regime and the tail-first holds stay untouched.
            // f64: the ratios are q-banded (aeroCalLiftQ/DragQ) - one scalar
            // chased a regime-dependent truth all the way down and the
            // prediction walked ~10km per EMA update
            if ((controller != null) && (controller.recoveryProfile == "starship")
                && (controller.phase == BLControllerPhase.BellyFlop) && (controller.aeroLiveCal)
                && (vel_air.magnitude > 1))
            {
                double w = Math.Min(1, Math.Max(0, (0.873 - aoa) / 0.349)); // 1 at <=30deg, 0 at >=50deg
                if (w > 0)
                {
                    double qStep = BLController.DynamicPressure(r.magnitude - body.Radius, vel_air.magnitude, body);
                    double kl = 1 + (controller.EffectiveCalLift(qStep) - 1) * w;
                    double kd = 1 + (controller.EffectiveCalDrag(qStep) - 1) * w;
                    if ((kl != 1) || (kd != 1))
                    {
                        Vector3d dragAxis = -Vector3d.Normalize(vel_air);
                        Vector3d liftPerp = Vector3d.Exclude(Vector3d.Normalize(r), vel_air);
                        if (liftPerp.magnitude > 0.01)
                        {
                            Vector3d liftAxis = Vector3d.Normalize(liftPerp);
                            double fd = Vector3d.Dot(Fa, dragAxis);
                            double fl = Vector3d.Dot(Fa, liftAxis);
                            Fa = dragAxis * (fd * kd) + liftAxis * (fl * kl) + (Fa - dragAxis * fd - liftAxis * fl);
                        }
                    }
                }
            }
            Vector3d F = Fa * aeroFudgeFactor + Ft;
            Vector3d a = F / totalMass + g;

            out_r = r + v * dt + 0.5 * a * dt * dt;
            out_v = v + a * dt;
        }


        static private Vector3d GetForces(Vessel vessel, Vector3d r, Vector3d v, Vector3d att, double totalMass, double minThrust, double maxThrust,
          Trajectories.VesselAerodynamicModel aeroModel, CelestialBody body, double t, double dt,
          BLController controller, Vector3d tgt_r, double aeroFudgeFactor,
          out Vector3d steer, out Vector3d vel_air, out double throttle)
        {
            Vector3d F = Vector3d.zero;
            double y = r.magnitude - body.Radius;
            steer = -Vector3d.Normalize(v);
            throttle = 0;

            // gravity
            double R = r.magnitude;
            Vector3d g = r * (-body.gravParameter / (R * R * R));

            float lastAng = (float)((-1) * body.angularVelocity.magnitude / Math.PI * 180.0);
            Quaternion lastBodyRot = Quaternion.AngleAxis(lastAng, body.angularVelocity.normalized);
            vel_air = v - body.getRFrmVel(r + body.position);

            if (controller != null)
            {
                bool bailOutLandingBurn = true;
                bool simulate = true;
                bool landingGear;
                controller.GetControlOutputs(vessel, totalMass, r, v, att, minThrust, maxThrust, t, body, simulate, out throttle, out steer, out landingGear, bailOutLandingBurn);
                if (throttle > 0)
                {
                    F = steer * (minThrust + throttle * (maxThrust - minThrust));
                }
                att = steer; // assume attitude is always correct
            }
            F = F + aeroModel.GetForces(body, r, vel_air, Math.PI) * aeroFudgeFactor; // retrograde

            F = F + g * totalMass;

            return F;
        }


        static public Vector3d ToGround(double tgtAlt,
                                        Vessel vessel,
                                        Trajectories.VesselAerodynamicModel aeroModel,
                                        CelestialBody body, BLController controller,
                                        Vector3d tgt_r,
                                        out double T,
                                        Utils.LogType logtype = Utils.LogType.none,
                                        Transform logTransform = null,
                                        double timeOffset = 0,
                                        double maxT = 600,
                                        Vector3d? startR = null,
                                        Vector3d? startV = null,
                                        double leadTime = 0,
                                        List<Vector3d> path = null)
        // Changes step size, dt, based on the amount of deacceleration forces, aero or thrust and winds back to choose smaller timesteps
        // startR/startV: optional body-relative state to start from instead of
        // the vessel's current state (deorbit scope starts at the post-node
        // state). leadTime: seconds between NOW and the start state (node
        // execution time); the final body-rotation compensation uses
        // T + leadTime so the impact lands in the correct body-fixed frame
        {
            float ang;
            Quaternion bodyRotation;
            if (Utils.LoggingActive &&  logtype != Utils.LogType.none)
            {
                Utils.Log(logtype, "time x y z vx vy vz ax ay az att_err target_error total_mass");
                Utils.Log(logtype, "# tgtAlt=" + tgtAlt);
            }

            T = 0;
            Vector3d r = (startR.HasValue) ? startR.Value : vessel.GetWorldPos3D() - body.position;
            Vector3d v = (startV.HasValue) ? startV.Value : vessel.GetObtVelocity();
            Vector3d a = Vector3d.zero;
            Vector3d last_r = r;
            Vector3d last_v = v;
            BLControllerPhase last_phase = controller.phase;
            double minThrust, maxThrust;
            double totalMass = vessel.totalMass;
            // Initially thrust is for all operational engines
            KSPUtils.ComputeMinMaxThrust(vessel, out minThrust, out maxThrust);
            // Per-phase propellant accounting (kg) for the return-fuel estimate.
            // Fuel flow = thrust / (Isp * g0); exact in kg regardless of the sim's
            // constant-mass approximation
            double isp = KSPUtils.GetAverageIsp(vessel);
            controller.simFuelBoostbackKg = 0;
            controller.simFuelReentryKg = 0;
            controller.simFuelLandingKg = 0;
            controller.simLowSpeed = -1;
            double y = r.magnitude - body.Radius;
            // TODO: att should be supplied as vessel transform will be wrong in simulation
            // Use the ReferenceTransform (control point), matching what Fly passes for
            // the real vessel - engine thrust transforms rotate with the gimbal and
            // would inject gimbal motion into the simulated attitude
            Transform refTransform = vessel.ReferenceTransform;
            Vector3d att = (refTransform != null) ? (Vector3d)refTransform.up : vessel.transform.up;
            double targetError = 0;

            if (controller != null)
            {
                // Take target error from previously calculated trajectory
                // We would know this at the end but can't wait until then
                targetError = controller.targetError;
            }

            // Use small dt all the way when below 5000m
            double dt_max = (y > 5000) ? dt_space : 1;
            double dt = dt_max;
            double last_T = T;
            double lastPathT = -10; // path sampling cadence (sim seconds)

            while ((y > tgtAlt) && (T < maxT))
            {
                y = r.magnitude - body.Radius;
                double dy = (r + v * dt).magnitude - body.Radius - y;

                // Surface-relative speed when first descending past 2km above
                // the target - early enough that no landing burn can have
                // braked the sim yet. Feeds the return-fuel estimate's
                // analytic landing-reserve floor
                if ((controller.simLowSpeed < 0) && (y - tgtAlt < 2000) && (Vector3d.Dot(v, r) < 0))
                    controller.simLowSpeed = (v - body.getRFrmVel(r + body.position)).magnitude;

                if (y + dy < controller.reentryBurnAlt)
                    dt = dt_reentry;

                if ((controller.phase == BLControllerPhase.AeroDescent) || (controller.phase == BLControllerPhase.LandingBurn))
                    dt = Math.Min(dt_aero, dt_max);

                if (Utils.LoggingActive && logtype != Utils.LogType.none)
                {
                    // NOTE: Cancel out rotation of planet
                    ang = (float)((-T) * body.angularVelocity.magnitude / Math.PI * 180.0);
                    // Rotation 1 second earlier
                    float prevang = (float)((-(T - 1)) * body.angularVelocity.magnitude / Math.PI * 180.0);
                    // Consider body rotation at this time
                    bodyRotation = Quaternion.AngleAxis(ang, body.angularVelocity.normalized);
                    Quaternion prevbodyRotation = Quaternion.AngleAxis(prevang, body.angularVelocity.normalized);
                    Vector3d tr = bodyRotation * r;
                    Vector3d tr1 = prevbodyRotation * r;
                    Vector3d tr2 = bodyRotation * (r + v);
                    Vector3d ta = bodyRotation * a;
                    tr = logTransform.InverseTransformPoint(tr + body.position);
                    Vector3d tv = logTransform.InverseTransformVector(tr2 - tr1);
                    ta = logTransform.InverseTransformVector(ta);
                    Utils.Log(logtype, string.Format("{0} {1:F5} {2:F5} {3:F5} {4:F5} {5:F5} {6:F5} {7:F1} {8:F1} {9:F1} 0 {10:F2} {11:F2}", T + timeOffset, tr.x, tr.y, tr.z, tv.x, tv.y, tv.z, ta.x, ta.y, ta.z, targetError, totalMass));
                }

                if ((y < body.atmosphereDepth) || (y < controller.reentryBurnAlt + 1500 * dt))
                    dt = Math.Min(dt, 2);

                Vector3d vel_air;
                Vector3d steer;
                double throttle;
                double aeroFudgeFactor = 1.05; // Assume aero forces 5% higher which causes overshoot of target and more vertical final descent
                Vector3d out_r;
                Vector3d out_v;
                // Compute time step change in r and v
                EulerStep(dt, vessel, r, v, att, totalMass, minThrust, maxThrust, aeroModel, body, T, controller, tgt_r, aeroFudgeFactor, out steer, out vel_air, out throttle, out out_r, out out_v);

                if (throttle > 0)
                {
                    double kg = (minThrust + throttle * (maxThrust - minThrust)) / (isp * 9.80665) * dt;
                    if (controller.phase == BLControllerPhase.BoostBack)
                        controller.simFuelBoostbackKg += kg;
                    else if (controller.phase == BLControllerPhase.ReentryBurn)
                        controller.simFuelReentryKg += kg;
                    else if (controller.phase == BLControllerPhase.LandingBurn)
                        controller.simFuelLandingKg += kg;
                }

                y = r.magnitude - body.Radius;

                att = steer; // assume can turn immediately

                last_phase = controller.phase;
                last_r = r;
                last_v = v;
                last_T = T;
                r = out_r;
                v = out_v;

                T = T + dt;

                // Path capture (deorbit scope trajectory line): same body-
                // rotation compensation as the returned impact (-(T+lead)),
                // so the drawn line ends exactly ON the red cross
                if ((path != null) && (T - lastPathT >= 2))
                {
                    lastPathT = T;
                    Quaternion pr = Quaternion.AngleAxis((float)((-(T + leadTime)) * body.angularVelocity.magnitude / Math.PI * 180.0), body.angularVelocity.normalized);
                    path.Add(pr * r);
                }
            }
            if (T >= maxT) // >= not >: a dt that divides maxT lands exactly ON it (flight 55: silent 300-row timeouts)
                Log.Info("Simulation time exceeds maxT=" + maxT);

            // Correct to point of intersect on surface
            double vy = Vector3d.Dot(last_v, Vector3d.Normalize(r));
            double p = 0;
            if (vy < -0.1)
            {
                p = (tgtAlt - y) / -vy; // Backup proportion
                // Flight 56: on a maxT TIMEOUT in a shallow glide (vy ~ -50
                // from 30+ km) this linear extrapolation is fantasy - p hits
                // hundreds of seconds (~1000 km ahead), and as vy -> 0 it
                // explodes/negative (terr read 8.4 MILLION m). Bound it; with
                // PredictionMaxT=1500 the starship glide completes honestly
                // and this path should be rare
                if (p > 300)
                    p = 300;
                if (p < -dt_space)
                    p = -dt_space;
                r = r - last_v * p;
                T = T - p;
            }
            // NOTE: do NOT call Utils.EndLogging() here. This used to "finish" the sim
            // log at the end of a logged run, but EndLogging closes ALL writers including
            // the actual-flight log - killing logging 25ms after every StartLogging.
            // Logging lifetime is owned by Enable/DisableGuidance, and writers AutoFlush.

                // Compensate for body rotation giving world position in the surface point now
                // that would be hit in the future (including any lead time
                // between now and the sim start state, e.g. time to a node)
                ang = (float)((-(T + leadTime)) * body.angularVelocity.magnitude / Math.PI * 180.0);
            bodyRotation = Quaternion.AngleAxis(ang, body.angularVelocity.normalized);
            r = bodyRotation * r;
            return r;
        }

        // Simulate trajectory to ground and work out point to fire landing burn assuming air resistance will help slow the vessel down
        // This point will be MUCH later than thrust would be applied minus air resistance
        // Height is used to mean the height above the target altitude
        static public double CalculateLandingBurnHeight(double tgtAlt, Vector3d r, Vector3d v, Vessel vessel, double totalMass, double minThrust, double maxThrust, Trajectories.VesselAerodynamicModel aeroModel, CelestialBody body,
          BLController controller = null, double maxT = 600, string filename = "", double suicideFactor = 0.8f)
        {
            double T = 0;
            double y = r.magnitude - body.Radius;
            double amin = minThrust / totalMass;
            double amax = maxThrust / totalMass;
            double LandingBurnHeight = -1;
            Vector3d att = -Vector3d.Normalize(v);

            double touchdownSpeed = 2;

            System.IO.StreamWriter f = null;
            if (filename != "")
            {
                f = new System.IO.StreamWriter(filename);
                f.WriteLine("time y vy dvy");
            }

            double dt = 1; // finer than dt_aero: the lowest-safe-ignition search below
                           // needs a decently sampled near-ground profile (4s steps at
                           // 300 m/s are 1.2 km apart) and explicit Euler over-credits
                           // aero braking at coarse steps
            // No-burn descent profile (height above target, airspeed) recorded so the
            // ignition criterion can credit the aero braking still available below
            // each point
            List<double> profileY = new List<double>();
            List<double> profileV = new List<double>();
            // Net suicide-burn deceleration (constant mass over the sim). Floor of
            // 0.1: pretend we have more thrust to look like we are doing something
            // rather than giving up!!
            double av = amax - body.gravParameter / (r.magnitude * r.magnitude);
            if (av < 0)
                av = 0.1;
            while ((y > tgtAlt) && (T < maxT))
            {
                y = r.magnitude - body.Radius;

                // Get all forces, i.e. aero-dynamic and thrust
                Vector3d steer, vel_air;
                double throttle;
                double aeroFudgeFactor = 1;
                // Need to simulate reentry burn to get reduced mass and less velocity
                // could probably approximation this well without much effort though
                Vector3d F = GetForces(vessel, r, v, -Vector3d.Normalize(v), totalMass, minThrust, maxThrust, aeroModel, body, T, dt, null, Vector3d.zero, aeroFudgeFactor, out steer, out vel_air, out throttle);

                // Calculate suicide burn velocity (rocket only, no aero credit - the
                // credited criterion is applied in the post-pass once the ground
                // speed of the no-burn profile is known)
                // dvy in 2 seconds time (allowing time for engine start up)
                double dvy = Math.Sqrt((1 + suicideFactor) * av * (y - tgtAlt)) + touchdownSpeed;

                profileY.Add(y - tgtAlt);
                profileV.Add(vel_air.magnitude);
                if (f != null)
                    f.WriteLine(string.Format("{0} {1:F1} {2:F1} {3:F1}", T, y, vel_air.magnitude, dvy));

                // Equations of motion
                Vector3d a = (F / totalMass);
                r = r + v * dt + 0.5 * a * dt * dt;
                v = v + a * dt;

                T = T + dt;
            }

            // Aero-credited suicide criterion, walked UP from the ground: a burn
            // started at height h only has to kill the speed the atmosphere will
            // NOT scrub below h, so the safe ignition height is the LOWEST point
            // where vel <= dvy + credit, credit = (vel(h) - vel(ground)) * factor.
            //
            // The old top-down test (vel < dvy, no credit) kept the HIGHEST
            // crossing of the two profiles. A heavy booster still accelerating
            // at 35 km crosses there, so it ignited at 36 km and fought gravity
            // for 80+ s - flight 38 burned 96 t where the aero-assisted burn
            // from ~6 km needs ~40 t. Below the lowest safe point the
            // free-descent speed is already unstoppable, which is exactly the
            // "too late" boundary we want; a false pocket higher up does not
            // matter because the vessel falls through it unpowered anyway
            const double aeroCreditFactor = 0.7; // discount: burn changes the
                                                 // descent profile, so the full
                                                 // no-burn credit is not available
            if (profileY.Count > 0)
            {
                double vGround = profileV[profileV.Count - 1];
                for (int i = profileY.Count - 1; i >= 0; i--)
                {
                    double dvy2 = Math.Sqrt((1 + suicideFactor) * av * Math.Max(0, profileY[i])) + touchdownSpeed;
                    double aeroCredit = Math.Max(0, profileV[i] - vGround) * aeroCreditFactor;
                    if (profileV[i] <= dvy2 + aeroCredit)
                    {
                        LandingBurnHeight = profileY[i];
                        break;
                    }
                }
            }
            if (T >= maxT) // >= not >: a dt that divides maxT lands exactly ON it (flight 55: silent 300-row timeouts)
                Log.Info("Simulation time exceeds maxT=" + maxT);
            if (f != null)
            {
                f.WriteLine("# LandingBurnHeight=" + LandingBurnHeight + " amax=" + amax);
                f.Close();
            }
            return LandingBurnHeight;
        }
    }
}
