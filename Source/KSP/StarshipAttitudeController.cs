using System;
using UnityEngine;
using static BoosterGuidance.InitLog;

namespace BoosterGuidance
{
    // Starship attitude controller (design D5): the falcon9 profile commands
    // a direction through SAS.SetTargetOrientation, which has no roll channel
    // and cannot hold the belly to the wind. This PD instead builds a full
    // target orientation (belly axis -> airflow, nose laid over sideways)
    // and writes FlightCtrlState.pitch/yaw/roll directly in Fly.
    public class StarshipAttitudeController
    {
        // Cascade controller (the structure kOS SteeringManager and MechJeb
        // use, instead of a naive torque PD): the outer loop turns attitude
        // error into a desired body RATE capped at maxRate, the inner loop
        // drives the measured rate to that target. The rate cap removes the
        // saturation windup that produced flight 43's lightly-damped ~8s
        // limit cycle (direct PD kAtt*err saturates at 1 for tens of
        // seconds, then overshoots). Gains:
        //   kAtt    - desired rate per radian of attitude error
        //   maxRate - slew rate ceiling (rad/s); RCS-only starship did
        //             ~9deg/s peak in flight 43, 0.35 rad/s = 20deg/s asks
        //             a bit more and saturates gracefully
        //   kRate   - actuation per rad/s of rate error
        public double kAtt = 1.5;
        public double maxRate = 0.35;
        public double kRate = 2.0;

        private Quaternion prevRot = Quaternion.identity;
        private double prevTime = -1;
        private Vector3d prevNoseTarget = Vector3d.zero;
        // f94 yaw freeze: captured nose (heading) reference for the
        // near-vertical wind cone - see Update below
        private Vector3d frozenNoseTarget = Vector3d.zero;
        private Vector3d frozenBellyTarget = Vector3d.zero; // f118: whole-frame freeze - see Update
        private bool yawFrozen = false;

        // Belly axis in world space: ReferenceTransform.forward rolled by
        // bellyRollOffset about the vessel long axis (ReferenceTransform.up)
        public static Vector3d BellyAxisWorld(Vessel vessel, double rollOffsetDeg)
        {
            Transform rt = vessel.ReferenceTransform;
            if (rt == null)
                return vessel.transform.forward;
            return Quaternion.AngleAxis((float)rollOffsetDeg, rt.up) * rt.forward;
        }

        // Belly/nose target frame for a given into-wind axis w (the
        // direction the belly faces at 90 deg AoA, i.e. -steer) and radial
        // up. aoa=90 reproduces the pure broadside hold (belly=w, nose =
        // up-perp-w); lower aoa pitches the nose toward the velocity vector
        // (shuttle-style glide, Janus1992 profile). noseT is zero when
        // degenerate (wind axis straight up/down)
        public static void BellyFrame(Vector3d w, Vector3d up, double aoaDeg, out Vector3d bellyT, out Vector3d noseT)
        {
            w = Vector3d.Normalize(w);
            Vector3d e2 = up - Vector3d.Project(up, w); // up perpendicular to w
            if (e2.magnitude < 0.05)
            {
                bellyT = w;
                noseT = Vector3d.zero;
                return;
            }
            e2 = Vector3d.Normalize(e2);
            double a = aoaDeg * Mathf.Deg2Rad;
            bellyT = Math.Sin(a) * w - Math.Cos(a) * e2;
            noseT = Math.Cos(a) * w + Math.Sin(a) * e2;
        }

        // steer is the controller output with falcon9 semantics: the "tail
        // into the wind" axis (-vel_air + correction). The belly faces the
        // wind instead, so the wind axis is -steer. aoaDeg is the scheduled
        // belly-flop AoA (90 = broadside, 60 = cruise glide)
        public void Update(Vessel vessel, double rollOffsetDeg, Vector3d steer, double aoaDeg, FlightCtrlState state)
        {
            if (steer == Vector3d.zero)
                return;
            // NaN/Inf steer would poison the FlightCtrlState and NaN the
            // vessel's orbit (flight 42 crash) - refuse to write anything
            if (double.IsNaN(steer.x) || double.IsNaN(steer.y) || double.IsNaN(steer.z)
                || double.IsInfinity(steer.x) || double.IsInfinity(steer.y) || double.IsInfinity(steer.z))
                return;
            Transform rt = vessel.ReferenceTransform;
            if (rt == null)
                return;
            Vector3d up = Vector3d.Normalize(vessel.GetWorldPos3D() - vessel.mainBody.position);
            Vector3d bellyT, noseT;
            BellyFrame(-steer, up, aoaDeg, out bellyT, out noseT);
            // f94 yaw freeze: at low horizontal airspeed the up-projection's
            // azimuth is slaved 1:1 to the vh azimuth, which goes unstable
            // as vh dies - the f94
            // violent brake collapsed vh 102->10 m/s, the velocity azimuth
            // spun ~190 deg at ~12 deg/s, the nose command chased it, the
            // hull spin-coupled (user: 倾斜很严重甚至自旋), two attitude
            // aborts, and the tumbling hull's side force walked the
            // cross-track 104->2109 m (watchdog cut lateral, final miss
            // 1917 m). Freeze the nose (heading) reference while vh is low:
            // bellyT keeps tracking the wind VECTOR (a near-vertical w
            // barely moves whatever its azimuth, so the broadside attitude
            // stays put), and LookRotation resolves the frozen hint as a
            // constant heading for the vertical drop. Thresholds straddle
            // the f94 failure (departure onset vh~61, spin vh<10) and stay
            // clear of the correction's full-authority band
            // (bellyCorrectionVhFull=80).
            // Hysteresis: freeze at vh<70, release at vh>90.
            Vector3d vAir = vessel.GetObtVelocity() - vessel.mainBody.getRFrmVel(vessel.GetWorldPos3D());
            double vhMag = Vector3d.Exclude(up, vAir).magnitude;
            if (yawFrozen)
            {
                if (vhMag > 90)
                    yawFrozen = false;
            }
            else if ((vhMag < 70) && (noseT.magnitude > 0.05))
            {
                yawFrozen = true;
                frozenNoseTarget = noseT;
                frozenBellyTarget = bellyT;
            }
            if (yawFrozen && (frozenNoseTarget != Vector3d.zero))
            {
                noseT = frozenNoseTarget;
                // f118: freeze the WHOLE frame, not just the nose hint. Below
                // vh~70 the wind azimuth is noise on a widening cone (w is
                // still ~30 deg off vertical at the freeze line), the belly
                // correction is already faded to zero there
                // (bellyCorrectionVhFull=80 - no directional lift to steer
                // with), so tracking the wind vector further only leaks
                // noise into the resolved roll: f118's att_err grew 0 -> 37
                // deg over the last 3.7 km of the broadside hold (the user's
                // "越接近地面右倾越严重"). A rock-fixed frame gives the PD a
                // settling target and the flip re-acquires from a stable
                // platform
                if (frozenBellyTarget != Vector3d.zero)
                    bellyT = frozenBellyTarget;
            }
            // Degenerate when the wind axis points straight down/up (vertical
            // descent) - hold the last good nose target through that
            if (noseT == Vector3d.zero)
                noseT = (prevNoseTarget != Vector3d.zero) ? prevNoseTarget : Vector3d.Exclude(bellyT, (Vector3d)rt.up);
            if (noseT.magnitude < 0.05)
                return;
            noseT = Vector3d.Normalize(noseT);
            PdToFrame(vessel, rollOffsetDeg, bellyT, noseT, state);
        }

        // f83 (user directive): the pilot 逆向/顺向 swing and the overshoot-
        // brake swing fly the nose onto noseTarget (= +/-vel_air) PITCH-ONLY -
        // "调节飞船姿态不要出现左右转向来掰机身，只能通过改变俯仰角来调整姿态".
        // The old SAS path (SetTargetOrientation) has no roll channel, so the
        // shortest-arc swing yawed/rolled the hull. Here the wing axis is
        // locked to the horizontal axis perpendicular to the flight direction
        // (sign matched to the current wing, so no 180-deg roll flip): both
        // the endpoint and the shortest-arc path to it then stay in the pitch
        // plane, and the PD's roll channel actively kills any bank
        public void UpdateSwing(Vessel vessel, double rollOffsetDeg, Vector3d noseTarget, Vector3d up, FlightCtrlState state)
        {
            if (noseTarget == Vector3d.zero)
                return;
            if (double.IsNaN(noseTarget.x) || double.IsNaN(noseTarget.y) || double.IsNaN(noseTarget.z)
                || double.IsInfinity(noseTarget.x) || double.IsInfinity(noseTarget.y) || double.IsInfinity(noseTarget.z))
                return;
            Transform rt0 = vessel.ReferenceTransform;
            if (rt0 == null)
                return;
            Vector3d noseT = Vector3d.Normalize(noseTarget);
            Vector3d wingT = Vector3d.Cross(up, noseT); // horizontal, perpendicular to the flight direction
            if (wingT.magnitude < 0.05)
                wingT = rt0.right; // near-vertical flight: keep the current heading
            else
                wingT = Vector3d.Normalize(wingT);
            // f85 fix: the frame's ACTUAL wing axis is not wingT but
            // Cross(noseT, bellyT) = -wingT (LookRotation sets right =
            // Cross(upHint, fwd), and bellyT = Cross(noseT, wingT) here).
            // f83 matched wingT itself to rt.right - exactly inverted - so
            // the target frame always carried the OPPOSITE wing axis and the
            // shortest-arc swing rolled/yawed the hull sideways (user f85:
            // "改变攻角来实现顺逆向...现在是横着转的"), and rt.right sweeping
            // through perpendicular mid-swing flipped the sign back and
            // forth = the twitching ("抽搐"). Matching -wingT to rt.right
            // keeps the whole swing in the pitch plane (retro lands
            // belly-up, prograde belly-down), and the dot product then sits
            // near -1 for the entire maneuver - no mid-swing flip
            if (Vector3d.Dot(wingT, (Vector3d)rt0.right) > 0)
                wingT = -wingT; // frame right (= -wingT) matches the current wing side - never swing through a 180-deg roll
            Vector3d bellyT = Vector3d.Cross(noseT, wingT); // orthonormal, perpendicular to noseT by construction
            PdToFrame(vessel, rollOffsetDeg, bellyT, noseT, state);
        }

        // Shared cascade PD: fly the frame (bellyT, noseT). The local belly
        // axis (forward rolled by rollOffset about local up) lands on bellyT,
        // local up on noseT
        private void PdToFrame(Vessel vessel, double rollOffsetDeg, Vector3d bellyT, Vector3d noseT, FlightCtrlState state)
        {
            Transform rt = vessel.ReferenceTransform;
            if (rt == null)
                return;
            Quaternion qCur = rt.rotation;
            double now = vessel.missionTime;

            // Body rate from rotation history (world frame) - avoids the
            // KSP vessel.angularVelocity frame ambiguity. Warp/teleport
            // spikes are discarded
            Vector3d omega = Vector3d.zero;
            double dt = now - prevTime;
            if ((prevTime > 0) && (dt > 1e-4) && (dt < 1))
            {
                Quaternion dq = qCur * Quaternion.Inverse(prevRot);
                float angDeg;
                Vector3 axis;
                dq.ToAngleAxis(out angDeg, out axis);
                if (angDeg > 180)
                    angDeg -= 360;
                omega = (Vector3d)axis * (angDeg * Mathf.Deg2Rad) / dt;
                // ToAngleAxis computes acos(w): floating-point drift can push
                // |w| a hair past 1 and return NaN (CANARY catch, flight 44 -
                // this was the EntryCoast NaN-orbit crash source all along)
                if (float.IsNaN(angDeg) || float.IsNaN(axis.x) || float.IsNaN(axis.y) || float.IsNaN(axis.z)
                    || float.IsInfinity(angDeg) || float.IsInfinity(axis.x) || float.IsInfinity(axis.y) || float.IsInfinity(axis.z))
                    omega = Vector3d.zero;
                else if (omega.magnitude > 20)
                    omega = Vector3d.zero; // warp/teleport spikes
            }
            prevRot = qCur;
            prevTime = now;

            prevNoseTarget = noseT;

            // Target rotation: the local belly axis (forward rolled by
            // rollOffset about local up) lands on bellyT, local up on noseT
            Quaternion local = Quaternion.AngleAxis((float)rollOffsetDeg, Vector3.up);
            Quaternion qTgt = Quaternion.LookRotation((Vector3)bellyT, (Vector3)noseT) * Quaternion.Inverse(local);

            // Attitude error as axis-angle in world frame
            Quaternion qErr = qTgt * Quaternion.Inverse(qCur);
            float errDeg;
            Vector3 errAxis;
            qErr.ToAngleAxis(out errDeg, out errAxis);
            // Same acos(w>1) NaN trap as the omega estimate above - the
            // CANARY caught this one firing in EntryCoast on flight 45
            if (float.IsNaN(errDeg) || float.IsNaN(errAxis.x) || float.IsNaN(errAxis.y) || float.IsNaN(errAxis.z)
                || float.IsInfinity(errDeg) || float.IsInfinity(errAxis.x) || float.IsInfinity(errAxis.y) || float.IsInfinity(errAxis.z))
                return;
            if (errDeg > 180)
                errDeg -= 360;
            Vector3d errVec = (Vector3d)errAxis * (errDeg * Mathf.Deg2Rad);

            // Cascade: outer loop attitude error -> desired rate (capped),
            // inner loop rate error -> actuation. Channel mapping verified
            // against kOS SteeringManager (pid wiring + torque axes)
            // and airplane control semantics: positive pitch pulls the nose
            // toward the DORSAL side (-rt.forward), so the belly is +rt.forward
            // by construction; positive yaw = nose right (about -rt.forward);
            // positive roll = right wing down (about -rt.up)
            Vector3d tgtOmega = kAtt * errVec;
            if (tgtOmega.magnitude > maxRate)
                tgtOmega = Vector3d.Normalize(tgtOmega) * maxRate;
            Vector3d cmd = kRate * (tgtOmega - omega);
            Vector3d lc = Quaternion.Inverse(qCur) * cmd;
            state.pitch = Mathf.Clamp((float)-lc.x, -1, 1);
            state.yaw = Mathf.Clamp((float)-lc.z, -1, 1);
            state.roll = Mathf.Clamp((float)-lc.y, -1, 1);
        }
    }
}
